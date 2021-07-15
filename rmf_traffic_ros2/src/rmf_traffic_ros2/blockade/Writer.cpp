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

#include <rmf_traffic_ros2/blockade/Writer.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic_msgs/msg/blockade_cancel.hpp>
#include <rmf_traffic_msgs/msg/blockade_heartbeat.hpp>
#include <rmf_traffic_msgs/msg/blockade_reached.hpp>
#include <rmf_traffic_msgs/msg/blockade_ready.hpp>
#include <rmf_traffic_msgs/msg/blockade_release.hpp>
#include <rmf_traffic_msgs/msg/blockade_set.hpp>

#include <unordered_set>

namespace rmf_traffic_ros2 {
namespace blockade {

namespace {
//==============================================================================
class RectifierFactory
  : public rmf_traffic::blockade::RectificationRequesterFactory
{
public:

  using ParticipantId = rmf_traffic::blockade::ParticipantId;
  using ReservationId = rmf_traffic::blockade::ReservationId;
  using ReservedRange = rmf_traffic::blockade::ReservedRange;
  using NewRangeCallback = Writer::NewRangeCallback;

  struct RectifierStub;

  class Requester : public rmf_traffic::blockade::RectificationRequester
  {
  public:

    std::shared_ptr<RectifierStub> stub;

    Requester(
      rmf_traffic::blockade::Rectifier rectifier,
      NewRangeCallback callback)
    : stub(std::make_shared<RectifierStub>(
          RectifierStub{
            std::move(rectifier),
            std::nullopt,
            std::move(callback)
          }))
    {
      // Do nothing
    }

  };

  struct RectifierStub
  {
    rmf_traffic::blockade::Rectifier rectifier;
    std::optional<ReservationId> last_reservation_id;
    NewRangeCallback range_cb;
  };

  using StubMap = std::unordered_map<
    rmf_traffic::blockade::ParticipantId,
    std::weak_ptr<RectifierStub>
  >;

  std::weak_ptr<rmf_traffic::blockade::Writer> weak_writer;
  StubMap stub_map;
  std::unordered_set<ParticipantId> dead_set;

  std::unordered_map<ParticipantId, NewRangeCallback> pending_callbacks;

  using HeartbeatMsg = rmf_traffic_msgs::msg::BlockadeHeartbeat;
  rclcpp::Subscription<HeartbeatMsg>::SharedPtr heartbeat_sub;

  // NOTE(MXG): Because of some awkwardness in the design of the rectification
  // factory, we can only allow one participant to be constructed at a time.
  // This mutex enforces that.
  //
  // TODO(MXG): Consider other ways of designing the construction of blockade
  // participants to simplify this design.
  std::mutex factory_mutex;

  RectifierFactory(
    rclcpp::Node& node,
    std::shared_ptr<rmf_traffic::blockade::Writer> writer)
  : weak_writer(std::move(writer))
  {
    heartbeat_sub = node.create_subscription<HeartbeatMsg>(
      BlockadeHeartbeatTopicName,
      rclcpp::SystemDefaultsQoS().reliable(),
      [&](const HeartbeatMsg::UniquePtr msg)
      {
        check_status(*msg);
      });
  }

  std::unique_ptr<rmf_traffic::blockade::RectificationRequester> make(
    rmf_traffic::blockade::Rectifier rectifier,
    rmf_traffic::blockade::ParticipantId participant_id) final
  {
    auto c_it = pending_callbacks.find(participant_id);
    assert(c_it != pending_callbacks.end());
    auto callback = std::move(c_it->second);
    pending_callbacks.erase(c_it);
    assert(pending_callbacks.empty());

    auto requester = std::make_unique<Requester>(
      std::move(rectifier), std::move(callback));

    stub_map.insert({participant_id, requester->stub});

    return requester;
  }

  void bring_out_your_dead()
  {
    auto s_it = stub_map.begin();
    while (s_it != stub_map.end())
    {
      const auto stub = s_it->second.lock();
      if (stub)
      {
        ++s_it;
      }
      else
      {
        dead_set.insert(s_it->first);
        stub_map.erase(s_it++);
      }
    }
  }

  using StatusMsg = rmf_traffic_msgs::msg::BlockadeStatus;
  rmf_traffic::blockade::Status convert(const StatusMsg& msg)
  {
    rmf_traffic::blockade::Status output;
    output.reservation = msg.reservation;
    if (msg.any_ready)
      output.last_ready = msg.last_ready;

    output.last_reached = msg.last_reached;

    return output;
  }

  rmf_traffic::blockade::ReservedRange get_range(const StatusMsg& msg)
  {
    rmf_traffic::blockade::ReservedRange range;
    range.begin = msg.assignment_begin;
    range.end = msg.assignment_end;
    return range;
  }

  void check_status(const HeartbeatMsg& heartbeat)
  {
    const auto writer = weak_writer.lock();
    if (!writer)
      return;

    std::unique_lock<std::mutex> lock(factory_mutex);

    bring_out_your_dead();
    std::unordered_set<rmf_traffic::blockade::ParticipantId> not_dead_yet;

    auto stub_map_copy = stub_map;
    for (const auto& status : heartbeat.statuses)
    {
      const auto it = stub_map_copy.find(status.participant);
      if (it == stub_map.end())
      {
        const auto d_it = dead_set.find(status.participant);
        if (d_it != dead_set.end())
        {
          writer->cancel(status.participant);
          not_dead_yet.insert(status.participant);
        }

        continue;
      }

      const auto stub = it->second.lock();
      if (!stub)
      {
        // We hit a race condition where this stub died just moments after we
        // swept through looking for which stubs died. As an easy way to deal
        // with this, we will simply skip this iteration and let any necessary
        // corrections happen the next time we get a heartbeat.
        continue;
      }

      stub->rectifier.check(convert(status));

      const auto range = get_range(status);
      stub->last_reservation_id = status.reservation;
      stub->range_cb(status.reservation, range);

      stub_map_copy.erase(it);
    }

    for (const auto& s : stub_map_copy)
    {
      // Check on the remaining stubs to make sure they shouldn't have any
      // active reservations.
      const auto stub = s.second.lock();
      if (!stub)
      {
        // We hit a race condition where this stub died just moments after we
        // wiped away the dead stubs. Just skip this for now and deal with
        // cleaning it up on the next round.
        continue;
      }

      stub->rectifier.check();
    }

    dead_set = not_dead_yet;
  }
};

} // anonymous namespace

//==============================================================================
class Writer::Implementation
{
public:

  class Transport
    : public rmf_traffic::blockade::Writer,
    public std::enable_shared_from_this<Transport>
  {
  public:
    std::shared_ptr<RectifierFactory> rectifier_factory;

    using Set = rmf_traffic_msgs::msg::BlockadeSet;
    using Ready = rmf_traffic_msgs::msg::BlockadeReady;
    using Reached = rmf_traffic_msgs::msg::BlockadeReached;
    using Release = rmf_traffic_msgs::msg::BlockadeRelease;
    using Cancel = rmf_traffic_msgs::msg::BlockadeCancel;
    using Checkpoint = rmf_traffic_msgs::msg::BlockadeCheckpoint;

    rclcpp::Publisher<Set>::SharedPtr set_pub;
    rclcpp::Publisher<Ready>::SharedPtr ready_pub;
    rclcpp::Publisher<Release>::SharedPtr release_pub;
    rclcpp::Publisher<Reached>::SharedPtr reached_pub;
    rclcpp::Publisher<Cancel>::SharedPtr cancel_pub;

    static std::shared_ptr<Transport> make(rclcpp::Node& node)
    {
      auto transport = std::make_shared<Transport>(node);
      transport->rectifier_factory =
        std::make_shared<RectifierFactory>(node, transport);

      return transport;
    }

    Transport(rclcpp::Node& node)
    {
      set_pub = node.create_publisher<Set>(
        BlockadeSetTopicName,
        rclcpp::SystemDefaultsQoS().best_effort());

      ready_pub = node.create_publisher<Ready>(
        BlockadeReadyTopicName,
        rclcpp::SystemDefaultsQoS().best_effort());

      reached_pub = node.create_publisher<Reached>(
        BlockadeReachedTopicName,
        rclcpp::SystemDefaultsQoS().best_effort());

      release_pub = node.create_publisher<Release>(
        BlockadeReleaseTopicName,
        rclcpp::SystemDefaultsQoS().best_effort());

      cancel_pub = node.create_publisher<Cancel>(
        BlockadeCancelTopicName,
        rclcpp::SystemDefaultsQoS().best_effort());
    }

    using ParticipantId = rmf_traffic::blockade::ParticipantId;
    using ReservationId = rmf_traffic::blockade::ReservationId;
    using CheckpointId = rmf_traffic::blockade::CheckpointId;

    rmf_traffic::blockade::Participant make_participant(
      ParticipantId id,
      double radius,
      NewRangeCallback range_cb)
    {
      std::unique_lock<std::mutex> lock(rectifier_factory->factory_mutex);
      rectifier_factory->pending_callbacks.insert({id, std::move(range_cb)});
      return rmf_traffic::blockade::make_participant(
        id, radius, shared_from_this(), rectifier_factory);
    }

    void set(
      const ParticipantId participant_id,
      const ReservationId reservation_id,
      const Reservation& reservation) final
    {
      std::vector<Checkpoint> checkpoints;
      checkpoints.reserve(reservation.path.size());
      for (const auto& c : reservation.path)
      {
        Checkpoint c_msg;
        c_msg.position[0] = c.position[0];
        c_msg.position[1] = c.position[1];
        c_msg.map_name = c.map_name;
        c_msg.can_hold = c.can_hold;

        checkpoints.emplace_back(std::move(c_msg));
      }

      auto msg =
        rmf_traffic_msgs::build<rmf_traffic_msgs::msg::BlockadeSet>()
        .participant(participant_id)
        .reservation(reservation_id)
        .radius(reservation.radius)
        .path(std::move(checkpoints));

      set_pub->publish(msg);
    }

    void ready(
      const ParticipantId participant_id,
      const ReservationId reservation_id,
      const CheckpointId checkpoint) final
    {
      auto msg =
        rmf_traffic_msgs::build<rmf_traffic_msgs::msg::BlockadeReady>()
        .participant(participant_id)
        .reservation(reservation_id)
        .checkpoint(checkpoint);

      ready_pub->publish(msg);
    }

    void release(
      ParticipantId participant_id,
      ReservationId reservation_id,
      CheckpointId checkpoint) final
    {
      auto msg =
        rmf_traffic_msgs::build<rmf_traffic_msgs::msg::BlockadeRelease>()
        .participant(participant_id)
        .reservation(reservation_id)
        .checkpoint(checkpoint);

      release_pub->publish(msg);
    }

    void reached(
      const ParticipantId participant_id,
      const ReservationId reservation_id,
      const CheckpointId checkpoint) final
    {
      auto msg =
        rmf_traffic_msgs::build<rmf_traffic_msgs::msg::BlockadeReached>()
        .participant(participant_id)
        .reservation(reservation_id)
        .checkpoint(checkpoint);

      reached_pub->publish(msg);
    }

    void cancel(
      const ParticipantId participant_id,
      const ReservationId reservation_id) final
    {
      auto msg =
        rmf_traffic_msgs::build<rmf_traffic_msgs::msg::BlockadeCancel>()
        .participant(participant_id)
        .all_reservations(false)
        .reservation(reservation_id);

      cancel_pub->publish(msg);
    }

    void cancel(ParticipantId participant_id) final
    {
      auto msg =
        rmf_traffic_msgs::build<rmf_traffic_msgs::msg::BlockadeCancel>()
        .participant(participant_id)
        .all_reservations(true)
        .reservation(0);

      cancel_pub->publish(msg);
    }
  };

  Implementation(rclcpp::Node& node)
  : transport(Transport::make(node))
  {
    // Do nothing
  }

  std::shared_ptr<Transport> transport;
};

//==============================================================================
std::shared_ptr<Writer> Writer::make(rclcpp::Node& node)
{
  return std::shared_ptr<Writer>(new Writer(node));
}

//==============================================================================
rmf_traffic::blockade::Participant Writer::make_participant(
  const rmf_traffic::blockade::ParticipantId id,
  const double radius,
  NewRangeCallback new_range_cb)
{
  return _pimpl->transport->make_participant(
    id, radius, std::move(new_range_cb));
}

//==============================================================================
Writer::Writer(rclcpp::Node& node)
: _pimpl(rmf_utils::make_unique_impl<Implementation>(node))
{
  // Do nothing
}

} // namespace blockade
} // namespace rmf_traffic_ros2
