/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Route.hpp>

#include <rmf_traffic_msgs/msg/itinerary_set.hpp>
#include <rmf_traffic_msgs/msg/itinerary_extend.hpp>
#include <rmf_traffic_msgs/msg/itinerary_delay.hpp>
#include <rmf_traffic_msgs/msg/itinerary_reached.hpp>
#include <rmf_traffic_msgs/msg/itinerary_clear.hpp>

#include <rmf_traffic_msgs/msg/schedule_inconsistency.hpp>

#include <rmf_traffic_msgs/msg/fail_over_event.hpp>
#include <rmf_traffic_msgs/msg/participants.hpp>

#include <rmf_traffic_msgs/srv/register_participant.hpp>
#include <rmf_traffic_msgs/srv/unregister_participant.hpp>

#include <rmf_utils/RateLimiter.hpp>

using namespace std::chrono_literals;

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::Writer::Registration convert(
  const rmf_traffic_msgs::srv::RegisterParticipant::Response& msg)
{
  return rmf_traffic::schedule::Writer::Registration(
    msg.participant_id,
    msg.last_itinerary_version,
    msg.last_plan_id,
    msg.next_storage_base);
}

namespace schedule {

namespace {
//==============================================================================
class RectifierFactory
  : public rmf_traffic::schedule::RectificationRequesterFactory
{
public:


  struct RectifierData;

  class Requester : public rmf_traffic::schedule::RectificationRequester
  {
  public:
    std::shared_ptr<RectifierData> data;

    Requester(rmf_traffic::schedule::Rectifier rectifier_);

  };

  struct RectifierData
  {
    rmf_traffic::schedule::Rectifier rectifier;

    // This is used to detect if corrections are needed too frequently, since
    // that might indicate that there are conflicting upstream participant
    // sources.
    rmf_utils::RateLimiter correction_limiter;
  };

  using RectifierMap = std::unordered_map<
    rmf_traffic::schedule::ParticipantId,
    std::weak_ptr<RectifierData>
  >;

  RectifierMap rectifier_map;

  using InconsistencyMsg = rmf_traffic_msgs::msg::ScheduleInconsistency;
  using ParticipantsInfoMsg = rmf_traffic_msgs::msg::Participants;

  std::unique_ptr<rmf_traffic::schedule::RectificationRequester> make(
    rmf_traffic::schedule::Rectifier rectifier,
    rmf_traffic::schedule::ParticipantId participant_id) final
  {
    auto requester = std::make_unique<Requester>(std::move(rectifier));

    // It's okay to just override any entry that might have been in here before,
    // because the Database should never double-assign a ParticipantId
    rectifier_map[participant_id] = requester->data;

    return requester;
  }

  void check_inconsistencies(const InconsistencyMsg& msg)
  {
    if (msg.ranges.empty())
    {
      // This shouldn't generally happen, since empty ranges should not get
      // published, but we'll check here anyway.
      return;
    }

    const auto it = rectifier_map.find(msg.participant);
    if (it == rectifier_map.end())
      return;

    const auto& stub = it->second.lock();
    if (!stub)
    {
      // This participant has expired, so we should remove it from the map
      rectifier_map.erase(it);
      return;
    }

    using Range = rmf_traffic::schedule::Rectifier::Range;
    std::vector<Range> ranges;
    ranges.reserve(msg.ranges.size());
    for (const auto& r : msg.ranges)
      ranges.emplace_back(Range{r.lower, r.upper});

    stub->rectifier.retransmit(
      ranges, msg.last_known_itinerary, msg.last_known_progress);
  }

  struct ChangeID
  {
    std::size_t new_id;
    std::size_t old_id;
    std::shared_ptr<RectifierData> stub;
  };

  std::vector<std::weak_ptr<RectifierData>> validate_participant_information(
    const rclcpp::Node& node,
    const ParticipantsInfoMsg& msg)
  {
    std::vector<std::weak_ptr<RectifierData>> incorrect_descriptions;
    std::vector<ChangeID> incorrect_ids;
    for (const auto& s : rectifier_map)
    {
      const auto stub = s.second.lock();
      if (!stub)
      {
        // This participant has expired so ignore it
        continue;
      }

      const auto local_desc_opt = stub->rectifier.get_description();
      if (!local_desc_opt.has_value())
        continue;

      const auto& local_desc = *local_desc_opt;

      auto p = std::find_if(
        msg.participants.begin(),
        msg.participants.end(),
        [&local_desc](const auto& participant)
        {
          const auto& remote_desc = participant.description;
          return local_desc.owner() == remote_desc.owner
          && local_desc.name() == remote_desc.name;
        });

      if (p == msg.participants.end())
      {
        if (!stub->correction_limiter.reached_limit())
        {
          // This participant is unregistered, even though we expected it to be
          RCLCPP_WARN(
            node.get_logger(),
            "Participant [%s] of [%s] is not registered properly",
            local_desc.name().c_str(),
            local_desc.owner().c_str());

          // Re-register the participant
          incorrect_descriptions.push_back(stub);
        }
      }
      else
      {
        // This participant is registered, but we need to check that the ID is
        // correct
        const bool ids_match = s.first == p->id;
        const bool descriptions_match = local_desc == convert(p->description);
        if (!ids_match || !descriptions_match)
        {
          if (stub->correction_limiter.reached_limit())
          {
            RCLCPP_ERROR(
              node.get_logger(),
              "Participant [%s] of [%s] has had repeated correctness issues. "
              "This likely indicates conflicting duplicate participants in the "
              "schedule system.",
              local_desc.name().c_str(),
              local_desc.owner().c_str());

            continue;
          }
        }

        if (!ids_match)
        {
          RCLCPP_WARN(
            node.get_logger(),
            "[rmf_traffic_ros2::schedule::Writer] "
            "Participant IDs do not match: stub = %ld, p = %ld",
            s.first,
            p->id);

          if (const auto old_id = stub->rectifier.get_id())
            incorrect_ids.push_back(ChangeID{p->id, *old_id, stub });
        }

        if (!descriptions_match)
          incorrect_descriptions.push_back(stub);
      }
    }

    for (const auto& change : incorrect_ids)
    {
      // We need to modify the rectifier_map in a separate loop, because the
      // loop above is doing a range-for iteration through the map, which
      // assumes the map will not be losing or gaining entries while it loops.
      // If we modify the map while looping through it, we will have undefined
      // behavior.
      rectifier_map.erase(change.old_id);
      rectifier_map.insert({change.new_id, change.stub});
      change.stub->rectifier.correct_id(change.new_id);
    }

    return incorrect_descriptions;
  }
};

//==============================================================================
RectifierFactory::Requester::Requester(
  rmf_traffic::schedule::Rectifier rectifier_)
: data(std::make_shared<RectifierData>(
      RectifierData{
        std::move(rectifier_),
        rmf_utils::RateLimiter(1min, 3)
      }))
{
  // Do nothing
}

} // anonymous namespace

//==============================================================================
class Writer::Implementation
{
public:

  class Transport
    : public rmf_traffic::schedule::Writer,
    public std::enable_shared_from_this<Transport>
  {
  public:
    std::shared_ptr<RectifierFactory> rectifier_factory;

    using Set = rmf_traffic_msgs::msg::ItinerarySet;
    using Extend = rmf_traffic_msgs::msg::ItineraryExtend;
    using Delay = rmf_traffic_msgs::msg::ItineraryDelay;
    using Reached = rmf_traffic_msgs::msg::ItineraryReached;
    using Clear = rmf_traffic_msgs::msg::ItineraryClear;

    rclcpp::Publisher<Set>::SharedPtr set_pub;
    rclcpp::Publisher<Extend>::SharedPtr extend_pub;
    rclcpp::Publisher<Delay>::SharedPtr delay_pub;
    rclcpp::Publisher<Reached>::SharedPtr reached_pub;
    rclcpp::Publisher<Clear>::SharedPtr clear_pub;

    rclcpp::Context::SharedPtr context;

    using Register = rmf_traffic_msgs::srv::RegisterParticipant;
    using Unregister = rmf_traffic_msgs::srv::UnregisterParticipant;

    rclcpp::Client<Register>::SharedPtr register_client;
    rclcpp::Client<Unregister>::SharedPtr unregister_client;

    using FailOverEvent = rmf_traffic_msgs::msg::FailOverEvent;
    using FailOverEventSub = rclcpp::Subscription<FailOverEvent>::SharedPtr;
    FailOverEventSub fail_over_event_sub;

    using ParticipantsInfoMsg = rmf_traffic_msgs::msg::Participants;
    rclcpp::Subscription<ParticipantsInfoMsg>::SharedPtr participants_info_sub;

    using InconsistencyMsg = rmf_traffic_msgs::msg::ScheduleInconsistency;
    rclcpp::Subscription<InconsistencyMsg>::SharedPtr inconsistency_sub;

    std::weak_ptr<rclcpp::Node> weak_node;

    static std::shared_ptr<Transport> make(
      const std::shared_ptr<rclcpp::Node>& node)
    {
      auto transport = std::shared_ptr<Transport>(new Transport(node));

      const auto itinerary_qos =
        rclcpp::SystemDefaultsQoS()
        .reliable()
        .keep_last(100);

      transport->set_pub = node->create_publisher<Set>(
        ItinerarySetTopicName,
        itinerary_qos);

      transport->extend_pub = node->create_publisher<Extend>(
        ItineraryExtendTopicName,
        itinerary_qos);

      transport->delay_pub = node->create_publisher<Delay>(
        ItineraryDelayTopicName,
        itinerary_qos);

      transport->reached_pub = node->create_publisher<Reached>(
        ItineraryReachedTopicName,
        itinerary_qos);

      transport->clear_pub = node->create_publisher<Clear>(
        ItineraryClearTopicName,
        itinerary_qos);

      transport->context = node->get_node_options().context();

      transport->register_client =
        node->create_client<Register>(RegisterParticipantSrvName);

      transport->unregister_client =
        node->create_client<Unregister>(UnregisterParticipantSrvName);

      transport->fail_over_event_sub = node->create_subscription<FailOverEvent>(
        rmf_traffic_ros2::FailOverEventTopicName,
        rclcpp::SystemDefaultsQoS(),
        [w = transport->weak_from_this()](const FailOverEvent::SharedPtr)
        {
          if (const auto self = w.lock())
            self->reconnect_services();
        });

      transport->participants_info_sub =
        node->create_subscription<ParticipantsInfoMsg>(
        ParticipantsInfoTopicName,
        rclcpp::SystemDefaultsQoS().reliable().transient_local().keep_last(100),
        [w = transport->weak_from_this()](
          const ParticipantsInfoMsg::UniquePtr msg)
        {
          if (const auto self = w.lock())
            self->validate_participant_information(*msg);
        });

      transport->inconsistency_sub =
        node->create_subscription<InconsistencyMsg>(
        ScheduleInconsistencyTopicName,
        rclcpp::SystemDefaultsQoS().reliable(),
        [w = transport->weak_from_this()](const InconsistencyMsg::UniquePtr msg)
        {
          if (const auto self = w.lock())
            self->rectifier_factory->check_inconsistencies(*msg);
        });

      return transport;
    }

    void set(
      const rmf_traffic::schedule::ParticipantId participant,
      const PlanId plan,
      const Itinerary& itinerary,
      const StorageId storage,
      const rmf_traffic::schedule::ItineraryVersion version) final
    {
      set_pub->publish(
        rmf_traffic_msgs::build<Set>()
        .participant(participant)
        .plan(plan)
        .itinerary(convert(itinerary))
        .storage_base(storage)
        .itinerary_version(version));
    }

    void extend(
      const rmf_traffic::schedule::ParticipantId participant,
      const Itinerary& routes,
      const rmf_traffic::schedule::ItineraryVersion version) final
    {
      extend_pub->publish(
        rmf_traffic_msgs::build<Extend>()
        .participant(participant)
        .routes(convert(routes))
        .itinerary_version(version));
    }

    void delay(
      const rmf_traffic::schedule::ParticipantId participant,
      const rmf_traffic::Duration duration,
      const rmf_traffic::schedule::ItineraryVersion version) final
    {
      delay_pub->publish(
        rmf_traffic_msgs::build<Delay>()
        .participant(participant)
        .delay(duration.count())
        .itinerary_version(version));
    }

    void reached(
      const ParticipantId participant,
      const PlanId plan,
      const std::vector<CheckpointId>& reached_checkpoints,
      const ProgressVersion version) final
    {
      reached_pub->publish(
        rmf_traffic_msgs::build<Reached>()
        .participant(participant)
        .plan(plan)
        .reached_checkpoints(reached_checkpoints)
        .progress_version(version));
    }

    void clear(
      const rmf_traffic::schedule::ParticipantId participant,
      const rmf_traffic::schedule::ItineraryVersion version) final
    {
      clear_pub->publish(
        rmf_traffic_msgs::build<Clear>()
        .participant(participant)
        .itinerary_version(version));
    }

    Registration register_participant(
      rmf_traffic::schedule::ParticipantDescription participant_info) final
    {
      using namespace std::chrono_literals;

      auto request = std::make_shared<Register::Request>();
      request->description = convert(participant_info);

      auto future = register_client->async_send_request(std::move(request));
      while (future.wait_for(100ms) != std::future_status::ready)
      {
        if (!rclcpp::ok(context))
        {
          // *INDENT-OFF*
          throw std::runtime_error(
            "[rmf_traffic_ros2::schedule::Writer] Tearing down while waiting "
            "for a schedule participant to finish registering");
          // *INDENT-ON*
        }
      }

      const auto response = future.get();
      if (!response->error.empty())
      {
        // *INDENT-OFF*
        throw std::runtime_error(
          "[rmf_traffic_ros2::schedule::Writer] Error while attempting to "
          "register a participant: " + response->error);
        // *INDENT-ON*
      }

      return convert(*response);
    }

    void async_register_participant(
      rmf_traffic::schedule::ParticipantDescription participant_info,
      std::function<void(Registration)> callback)
    {
      auto request = std::make_shared<Register::Request>();
      request->description = convert(participant_info);

      using Response = std::shared_future<std::shared_ptr<Register::Response>>;
      std::function<void(Response)> cb =
        [callback = std::move(callback)](const Response& future_response)
        {
          if (future_response.wait_for(0s) == std::future_status::timeout)
            return;

          callback(convert(*future_response.get()));
        };

      register_client->async_send_request(std::move(request), std::move(cb));
    }

    void update_description(
      rmf_traffic::schedule::ParticipantId,
      rmf_traffic::schedule::ParticipantDescription participant_info)
    {
      // Since each robot is uniquely identified by its owner and name pair in
      // the ROS2 implementation, the registration service handles updating of
      // participant info as well.
      register_participant(std::move(participant_info));
    }

    void unregister_participant(
      const rmf_traffic::schedule::ParticipantId participant) final
    {
      auto request = std::make_shared<Unregister::Request>();
      request->participant_id = participant;

      unregister_client->async_send_request(
        request,
        [=](const rclcpp::Client<Unregister>::SharedFuture response_future)
        {
          const auto response = response_future.get();
          if (!response->error.empty())
          {
            // *INDENT-OFF*
            throw std::runtime_error(
              "[rmf_traffic_ros2::schedule::Writer] Error while attempting to "
              "unregister a participant: " + response->error);
            // *INDENT-ON*
          }
        });
    }

    void reconnect_services()
    {
      const auto node = weak_node.lock();
      if (!node)
        return;

      RCLCPP_INFO(
        node->get_logger(),
        "Reconnecting services for Writer::Transport");
      // Deleting the old services will shut them down
      register_client =
        node->create_client<Register>(RegisterParticipantSrvName);
      unregister_client =
        node->create_client<Unregister>(UnregisterParticipantSrvName);
    }

    void validate_participant_information(
      const ParticipantsInfoMsg& msg)
    {
      const auto node = weak_node.lock();

      const auto incorrect_descriptions =
        rectifier_factory->validate_participant_information(*node, msg);

      for (const auto& d : incorrect_descriptions)
      {
        const auto stub = d.lock();
        if (!stub)
          continue;

        auto description = stub->rectifier.get_description();
        auto old_id = stub->rectifier.get_id();
        if (!description.has_value() || !old_id.has_value())
          continue;

        auto callback = [rectifiers = rectifier_factory, d, old_id](
          Registration registration)
          {
            if (const auto stub = d.lock())
            {
              stub->rectifier.correct_id(registration.id());
              rectifiers->rectifier_map.erase(*old_id);
              rectifiers->rectifier_map.insert({registration.id(), stub});
            }
          };

        async_register_participant(*description, std::move(callback));
      }
    }

  private:
    Transport(const std::shared_ptr<rclcpp::Node>& node)
    : rectifier_factory(std::make_shared<RectifierFactory>()),
      weak_node(node)
    {
      // Use make to initialize
    }
  };

  Implementation(const std::shared_ptr<rclcpp::Node>& node)
  : transport(Transport::make(node))
  {
    // Do nothing
  }

  std::shared_ptr<Transport> transport;

  std::future<rmf_traffic::schedule::Participant> make_participant(
    rmf_traffic::schedule::ParticipantDescription description)
  {
    // TODO(MXG): This implementation assumes that the async promise will be
    // finished before the Writer instance is destructed. If that is not true,
    // then we could get undefined behavior from this implementation. However,
    // the Writer should only get destructed during the teardown of the whole
    // Node, which implies that the program is exiting.
    //
    // This shouldn't be a major concern, but it may be worth revisiting whether
    // a cleaner approach is possible.

    std::promise<rmf_traffic::schedule::Participant> promise;
    auto future = promise.get_future();
    std::thread worker(
      [this](
        rmf_traffic::schedule::ParticipantDescription description,
        std::promise<rmf_traffic::schedule::Participant> promise)
      {
        promise.set_value(rmf_traffic::schedule::make_participant(
          std::move(description), transport, transport->rectifier_factory));
      }, std::move(description), std::move(promise));

    worker.detach();

    return future;
  }

  void async_make_participant(
    rmf_traffic::schedule::ParticipantDescription description,
    std::function<void(rmf_traffic::schedule::Participant)> ready_callback)
  {
    std::thread worker(
      [description = std::move(description),
      this,
      ready_callback = std::move(ready_callback)]()
      {
        // TODO(MXG): We could probably make an implementation of the
        // RectifierFactory that allows us to pass the ready_callback along to
        // the service call so that it gets triggered when the service response
        // is received. That way we don't need to create an additional thread
        // here and worry about the threat of race conditions.
        auto participant = rmf_traffic::schedule::make_participant(
          std::move(description), transport, transport->rectifier_factory);

        if (ready_callback)
          ready_callback(std::move(participant));
      });

    worker.detach();
  }
};

//==============================================================================
std::shared_ptr<Writer> Writer::make(
  const std::shared_ptr<rclcpp::Node>& node)
{
  auto writer = std::shared_ptr<Writer>(new Writer);
  writer->_pimpl = rmf_utils::make_unique_impl<Implementation>(std::move(node));
  return writer;
}

//==============================================================================
bool Writer::ready() const
{
  return _pimpl->transport->register_client->service_is_ready()
    && _pimpl->transport->unregister_client->service_is_ready();
}

//==============================================================================
void Writer::wait_for_service() const
{
  _pimpl->transport->register_client->wait_for_service();
  _pimpl->transport->unregister_client->wait_for_service();
}

//==============================================================================
bool Writer::wait_for_service(rmf_traffic::Time stop) const
{
  bool ready = true;

  ready &= _pimpl->transport->register_client->wait_for_service(
    stop - std::chrono::steady_clock::now());

  ready &= _pimpl->transport->unregister_client->wait_for_service(
    stop - std::chrono::steady_clock::now());

  return ready;
}

//==============================================================================
std::future<rmf_traffic::schedule::Participant> Writer::make_participant(
  rmf_traffic::schedule::ParticipantDescription description)
{
  return _pimpl->make_participant(std::move(description));
}

//==============================================================================
void Writer::async_make_participant(
  rmf_traffic::schedule::ParticipantDescription description,
  std::function<void(rmf_traffic::schedule::Participant)> ready_callback)
{
  _pimpl->async_make_participant(
    std::move(description), std::move(ready_callback));
}

//==============================================================================
Writer::Writer()
{
  // Do nothing
}

} // namespace schedule
} // namespace rmf_traffic_ros2
