/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <chrono>

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rmf_utils/Modular.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Patch.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>

#include <rmf_traffic_msgs/msg/mirror_update.hpp>
#include <rmf_traffic_msgs/msg/participant.hpp>
#include <rmf_traffic_msgs/msg/participants.hpp>
#include <rmf_traffic_msgs/msg/schedule_query.hpp>
#include <rmf_traffic_msgs/msg/schedule_queries.hpp>

#include <rmf_traffic_msgs/msg/fail_over_event.hpp>

#include <rmf_traffic_msgs/srv/register_query.hpp>
#include <rmf_traffic_msgs/srv/request_changes.hpp>

using namespace std::chrono_literals;

namespace rmf_traffic_ros2 {
namespace schedule {

using SingleParticipantInfo = rmf_traffic_msgs::msg::Participant;
using ParticipantsInfo = rmf_traffic_msgs::msg::Participants;
using ParticipantsInfoSub = rclcpp::Subscription<ParticipantsInfo>::SharedPtr;

using MirrorUpdate = rmf_traffic_msgs::msg::MirrorUpdate;
using MirrorUpdateSub = rclcpp::Subscription<MirrorUpdate>::SharedPtr;

using RequestChanges = rmf_traffic_msgs::srv::RequestChanges;
using RequestChangesFuture = rclcpp::Client<RequestChanges>::SharedFuture;
using RequestChangesClient = rclcpp::Client<RequestChanges>::SharedPtr;

using RegisterQuery = rmf_traffic_msgs::srv::RegisterQuery;
using RegisterQueryClient = rclcpp::Client<RegisterQuery>::SharedPtr;
using RegisterQueryFuture = rclcpp::Client<RegisterQuery>::SharedFuture;

using ScheduleQuery = rmf_traffic_msgs::msg::ScheduleQuery;
using ScheduleQueries = rmf_traffic_msgs::msg::ScheduleQueries;

using FailOverEvent = rmf_traffic_msgs::msg::FailOverEvent;
using FailOverEventSub = rclcpp::Subscription<FailOverEvent>::SharedPtr;

namespace {
bool is_new_version(const uint64_t expected_version, const uint64_t msg_version)
{
  return rmf_utils::modular(expected_version).less_than(msg_version);
}
}

//==============================================================================
class MirrorManager::Implementation
{
public:

  // TODO(MXG): Remove all use of [&] and [this] from this implementation by
  // migrating the fields into a Shared structure and capturing that in the
  // lambdas.

  std::weak_ptr<rclcpp::Node> weak_node;
  rmf_traffic::schedule::Query query;
  uint64_t query_id = 0;
  bool require_query_validation = false;
  uint64_t expected_node_version = 0;
  std::list<MirrorUpdate::SharedPtr> stashed_query_updates;
  Options options;
  FailOverEventSub fail_over_event_sub;
  MirrorUpdateSub mirror_update_sub;
  ParticipantsInfoSub participants_info_sub;
  rclcpp::Subscription<ScheduleQueries>::SharedPtr queries_info_sub;
  RequestChangesClient request_changes_client;
  rclcpp::TimerBase::SharedPtr update_timer;
  rclcpp::TimerBase::SharedPtr redo_query_registration_timer;
  RegisterQueryClient register_query_client;

  std::shared_ptr<rmf_traffic::schedule::Mirror> mirror;

  bool initial_update = true;

  rmf_traffic::schedule::Version next_minimum_version = 0;

  Implementation(
    const std::shared_ptr<rclcpp::Node>& node,
    rmf_traffic::schedule::Query _query,
    Options _options,
    uint64_t _query_id)
  : weak_node(node),
    query(std::move(_query)),
    query_id(_query_id),
    options(std::move(_options)),
    mirror(std::make_shared<rmf_traffic::schedule::Mirror>())
  {
    setup_update_topics();
    setup_queries_sub();

    request_changes_client = node->create_client<RequestChanges>(
      rmf_traffic_ros2::RequestChangesServiceName);

    fail_over_event_sub = node->create_subscription<FailOverEvent>(
      rmf_traffic_ros2::FailOverEventTopicName,
      rclcpp::SystemDefaultsQoS(),
      [&](const FailOverEvent::SharedPtr msg)
      {
        handle_fail_over_event(msg->new_schedule_node_version);
      });
  }

  void setup_queries_sub()
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    queries_info_sub = node->create_subscription<ScheduleQueries>(
      rmf_traffic_ros2::QueriesInfoTopicName,
      rclcpp::SystemDefaultsQoS().reliable().keep_last(100).transient_local(),
      [=](const ScheduleQueries::SharedPtr msg)
      {
        const bool is_old_version = rmf_utils::modular(
          msg->node_version).less_than(expected_node_version);

        if (is_old_version)
        {
          // This is an outdated schedule node version, so we will ignore it
          return;
        }

        // In case the new schedule node version is higher, make sure we update
        // the node version that we're expecting.
        expected_node_version = msg->node_version;

        RCLCPP_INFO(
          node->get_logger(),
          "Mirror handling new sync of %lu queries "
          "from schedule node version [%lu]",
          msg->queries.size(),
          msg->node_version);

        // Find what should be our query based on our query ID
        std::optional<uint64_t> our_query = std::nullopt;
        for (std::size_t ii = 0; ii < msg->queries.size(); ++ii)
        {
          if (msg->ids[ii] == query_id)
          {
            our_query = ii;
            break;
          }
        }

        if (our_query.has_value())
        {
          // Confirm that the schedule node's query with our query ID is
          // actually our query
          if (rmf_traffic_ros2::convert(msg->queries[*our_query]) != query)
          {
            // The schedule node has someone else's query registered for our
            // query ID
            RCLCPP_ERROR(
              node->get_logger(),
              "Mismatched query ID detected from schedule node; "
              "re-registering query");
            dump_stashed_queries();
            // Re-register our query to get a (possibly new) correct query ID
            redo_query_registration();
          }
          else
          {
            // Query is correct
            require_query_validation = false;
            process_stashed_queries();
          }
        }
        else
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "Missing query ID; re-registering query");
          dump_stashed_queries();
          redo_query_registration();
        }
      });
  }

  void setup_update_topics()
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    participants_info_sub = node->create_subscription<ParticipantsInfo>(
      ParticipantsInfoTopicName,
      rclcpp::SystemDefaultsQoS().reliable().keep_last(100).transient_local(),
      [&](const ParticipantsInfo::SharedPtr msg)
      {
        handle_participants_info(msg);
      });

    RCLCPP_DEBUG(node->get_logger(), "Registering to query topic %s",
      (QueryUpdateTopicNameBase + std::to_string(query_id)).c_str());
    mirror_update_sub = node->create_subscription<MirrorUpdate>(
      QueryUpdateTopicNameBase + std::to_string(query_id),
      rclcpp::SystemDefaultsQoS(),
      [this](const MirrorUpdate::SharedPtr msg)
      {
        handle_update(msg);
      });
    // At this point we know we have the correct ID for our query
    require_query_validation = false;
    process_stashed_queries();

    update_timer = node->create_wall_timer(
      5s,
      [this]() -> void
      {
        handle_update_timeout();
      });
  }

  void handle_participants_info(const ParticipantsInfo::SharedPtr msg)
  {
    try
    {
      std::mutex* update_mutex = options.update_mutex();
      if (update_mutex)
      {
        std::lock_guard<std::mutex> lock(*update_mutex);
        mirror->update_participants_info(convert(*msg));
      }
      else
      {
        mirror->update_participants_info(convert(*msg));
      }
    }
    catch (const std::exception& e)
    {
      if (const auto node = weak_node.lock())
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "[rmf_traffic_ros2::MirrorManager] Failed to update participant info: %s",
          e.what());
      }
    }
  }

  void process_stashed_queries()
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    RCLCPP_DEBUG(node->get_logger(), "Processing stashed queries");
    for (auto&& msg: stashed_query_updates)
    {
      RCLCPP_DEBUG(
        node->get_logger(),
        "  Processing stashed query for DB update %lu",
        msg->patch.latest_version);
      // TODO(Geoff): If somehow require_query_validation gets set back to true
      // while processing this loop, it will enter an infinite loop. Add a
      // counter or use a counter-based loop to prevent that? Or is it
      // impossible for require_query_validation to go true while in here?
      handle_update(std::move(msg));
    }
    stashed_query_updates.clear();
  }

  // Call this if the stashed queries are found to be for an incorrect query
  void dump_stashed_queries()
  {
    stashed_query_updates.clear();
  }

  void handle_update(const MirrorUpdate::SharedPtr msg)
  {
    update_timer->reset();
    const auto node = weak_node.lock();
    if (!node)
      return;

    // Verify that the expected schedule node version sent the update

    if (rmf_utils::modular(expected_node_version).less_than(msg->node_version))
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Received query update from unexpected schedule node version %lu "
        "(<%lu); ignoring update",
        msg->node_version,
        expected_node_version);
      return;
    }
    else if (msg->node_version > expected_node_version)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Received query update from unexpected schedule node version %lu "
        "(>%lu); validating query registration",
        msg->node_version,
        expected_node_version);
      require_query_validation = true;
      expected_node_version = msg->node_version;
      stashed_query_updates.clear();
    }
    else if (msg->node_version != expected_node_version)
    {
      // Ignore this message because it's coming from an out-of-date version
      return;
    }

    if (require_query_validation)
    {
      // Stash this query update until the query has been verified as correct
      RCLCPP_DEBUG(
        node->get_logger(),
        "Stashing suspect query for DB version %lu",
        msg->patch.latest_version);
      stashed_query_updates.push_back(msg);
      return;
    }

    try
    {
      const rmf_traffic::schedule::Patch patch = convert(msg->patch);

      std::mutex* update_mutex = options.update_mutex();
      if (update_mutex)
      {
        std::lock_guard<std::mutex> lock(*update_mutex);
        if (!mirror->update(patch) && !msg->is_remedial_update)
        {
          RCLCPP_WARN(
            node->get_logger(),
            "Failed to update using patch for DB version %lu; "
            "requesting new update",
            patch.latest_version());
          request_update(mirror->latest_version());
        }
      }
      else
      {
        if (!mirror->update(patch) && !msg->is_remedial_update)
        {
          RCLCPP_WARN(
            node->get_logger(),
            "Failed to update using patch for DB version %lu; "
            "requesting new update",
            patch.latest_version());
          request_update(mirror->latest_version());
        }
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "[rmf_traffic_ros2::MirrorManager] Failed to deserialize Patch "
        "message: %s",
        e.what());
      // Get a full update in case we're just missing some information
      request_update();
    }
  }

  void handle_update_timeout()
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    RCLCPP_DEBUG(node->get_logger(), "Update timed out");
    request_update(mirror->latest_version());
  }

  void request_update(std::optional<uint64_t> minimum_version = std::nullopt)
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    RequestChanges::Request request;
    request.query_id = query_id;
    if (minimum_version.has_value())
    {
      request.version = minimum_version.value();
      request.full_update = false;
    }
    else
    {
      RCLCPP_INFO(
        node->get_logger(),
        "[rmf_traffic_ros2::MirrorManager::request_update] Requesting changes "
        "for query ID [%ld] since beginning of recorded history",
        request.query_id);
      request.version = 0;
      request.full_update = true;
    }

    request_changes_client->async_send_request(
      std::make_shared<RequestChanges::Request>(request),
      [this, minimum_version](const RequestChangesFuture response)
      {
        // Check how the schedule node handled the request. The actual queries
        // update will come separately over the query update topic; this is
        // just whether the request was handled successfully or not.
        auto value = *response.get();
        if (value.result == RequestChanges::Response::UNKNOWN_QUERY_ID)
        {
          redo_query_registration();
        }
        else if (value.result == RequestChanges::Response::ERROR)
        {
          const auto node = weak_node.lock();
          if (node)
          {
            if (minimum_version.has_value())
            {
              RCLCPP_ERROR(
                node->get_logger(),
                "[MirrorManager::request_update] Failed to request an update "
                "for query ID [%ld] up from version [%lu]. Error message: %s",
                query_id,
                minimum_version.value(),
                value.error.c_str());
            }
            else
            {
              RCLCPP_ERROR(
                node->get_logger(),
                "[MirrorManager::request_updpate] Failed to request an update "
                "for query ID [%ld] from the beginning of recorded history. "
                "Error message: %s",
                query_id,
                value.error.c_str());
            }
          }
        }
      });
  }

  void redo_query_registration()
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    RCLCPP_DEBUG(node->get_logger(), "Redoing query registration");
    // Make sure nothing is truly coming in on this topic and triggering a
    // callback while we are remaking it
    mirror_update_sub.reset();
    // Also make sure we don't try to handle another update of queries,
    // or it might cause a particularly icky cycle of never-ending redos
    queries_info_sub.reset();

    register_query_client =
      node->create_client<RegisterQuery>(RegisterQueryServiceName);
    redo_query_registration_timer = node->create_wall_timer(
      100ms,
      std::bind(
        &MirrorManager::Implementation::redo_query_registration_callback,
        this));
  }

  void redo_query_registration_callback()
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    if (register_query_client->service_is_ready())
    {
      RCLCPP_DEBUG(
        node->get_logger(),
        "Redoing query registration: Calling service");
      RegisterQuery::Request register_query_request;
      register_query_request.query = convert(query);
      register_query_client->async_send_request(
        std::make_shared<RegisterQuery::Request>(register_query_request),
        [this](const RegisterQueryFuture response)
        {
          const auto node = weak_node.lock();
          if (!node)
            return;

          const auto msg = response.get();
          if (is_new_version(this->expected_node_version, msg->node_version))
          {
            this->expected_node_version = msg->node_version;
          }

          this->query_id = msg->query_id;
          RCLCPP_DEBUG(
            node->get_logger(),
            "Redoing query registration: Got new ID %lu",
            query_id);
          setup_update_topics();
          setup_queries_sub();
          this->register_query_client.reset();

          // Finish by requesting an update on this newly subscribed query topic
          request_update();
        });

      redo_query_registration_timer.reset();
    }
    else
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Failed to get query registry service");
    }
  }

  void handle_fail_over_event(uint64_t new_schedule_node_version)
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    RCLCPP_INFO(
      node->get_logger(),
      "Handling fail over event. New expected schedule node version [%ld].",
      new_schedule_node_version);

    // We need to validate that the replacement schedule node has our query
    // correctly. This will be reset to false once we have received the query
    // info for the new version and confirmed that it has our query stored in it
    // correctly.
    //
    // While true, all query updates received will be stashed in a list, and
    // will be popped in FIFO order and processed when this is reset to false.
    if (is_new_version(expected_node_version, new_schedule_node_version))
    {
      require_query_validation = true;
      // The new schedule node will be one version higher
      expected_node_version = new_schedule_node_version;
    }
  }

  template<typename... Args>
  static MirrorManager make(Args&& ... args)
  {
    MirrorManager mgr;
    mgr._pimpl = rmf_utils::make_unique_impl<Implementation>(
      std::forward<Args>(args)...);

    return mgr;
  }
};

//==============================================================================
class MirrorManager::Options::Implementation
{
public:

  std::mutex* update_mutex;

  bool update_on_wakeup;

};

//==============================================================================
MirrorManager::Options::Options(
  std::mutex* update_mutex,
  bool update_on_wakeup)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        update_mutex,
        update_on_wakeup
      }))
{
  // Do nothing
}

//==============================================================================
std::mutex* MirrorManager::Options::update_mutex() const
{
  return _pimpl->update_mutex;
}

//==============================================================================
auto MirrorManager::Options::update_mutex(std::mutex* mutex) -> Options&
{
  _pimpl->update_mutex = mutex;
  return *this;
}

//==============================================================================
bool MirrorManager::Options::update_on_wakeup() const
{
  return _pimpl->update_on_wakeup;
}

//==============================================================================
auto MirrorManager::Options::update_on_wakeup(bool choice) -> Options&
{
  _pimpl->update_on_wakeup = choice;
  return *this;
}

//==============================================================================
std::shared_ptr<const rmf_traffic::schedule::Mirror>
MirrorManager::view() const
{
  return _pimpl->mirror;
}

//==============================================================================
void MirrorManager::update()
{
  _pimpl->request_update(_pimpl->mirror->latest_version());
}

//==============================================================================
auto MirrorManager::get_options() const -> const Options&
{
  return _pimpl->options;
}

//==============================================================================
MirrorManager& MirrorManager::set_options(Options options)
{
  _pimpl->options = std::move(options);
  return *this;
}

//==============================================================================
rmf_traffic::schedule::Database MirrorManager::fork() const
{
  return _pimpl->mirror->fork();
}

//==============================================================================
MirrorManager::MirrorManager()
{
  // Do nothing
}

//==============================================================================
class MirrorManagerFuture::Implementation
{
public:

  std::weak_ptr<rclcpp::Node> weak_node;
  rmf_traffic::schedule::Query query;
  MirrorManager::Options options;

  RegisterQueryClient register_query_client;

  std::atomic_bool abandon_discovery;
  std::atomic_bool registration_sent;
  std::thread discovery_thread;

  std::future<RegisterQuery::Response> registration_future;
  std::promise<RegisterQuery::Response> registration_promise;

  Implementation(
    const std::shared_ptr<rclcpp::Node>& node,
    rmf_traffic::schedule::Query _query,
    MirrorManager::Options _options)
  : weak_node(node),
    query(std::move(_query)),
    options(std::move(_options)),
    abandon_discovery(false),
    registration_sent(false)
  {
    register_query_client =
      node->create_client<RegisterQuery>(RegisterQueryServiceName);

    registration_future = registration_promise.get_future();

    discovery_thread = std::thread([=]() { this->discover(); });
  }

  void discover()
  {
    const auto timeout = std::chrono::milliseconds(10);
    bool ready = false;
    while (!abandon_discovery && !ready)
    {
      ready = register_query_client->wait_for_service(timeout);
    }

    // If the schedule node falls over right now, the promise will never be
    // fulfilled. The user must handle this possibility.
    if (ready && !abandon_discovery)
    {
      RegisterQuery::Request register_query_request;
      register_query_request.query = convert(query);
      register_query_client->async_send_request(
        std::make_shared<RegisterQuery::Request>(register_query_request),
        [&](const RegisterQueryFuture response)
        {
          const auto node = weak_node.lock();
          if (!node)
            return;

          try
          {
            registration_promise.set_value(*response.get());
          }
          catch (const std::exception& e)
          {
            RCLCPP_ERROR(
              node->get_logger(),
              "[rmf_traffic_ros2::MirrorManagerFuture] Exception while "
              "registering a query: %s",
              e.what());
          }
        });
      registration_sent = true;
    }
  }

  void wait() const
  {
    registration_future.wait();
  }

  std::future_status wait_for(const rmf_traffic::Duration& timeout) const
  {
    return registration_future.wait_for(timeout);
  }

  std::future_status wait_until(const rmf_traffic::Time& time) const
  {
    return registration_future.wait_until(time);
  }

  bool valid() const
  {
    return registration_future.valid();
  }

  MirrorManager get()
  {
    const auto node = weak_node.lock();
    if (!node)
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[MirrorManagerFuture::get] "
        "Node expired before the future could be retrieved");
      // *INDENT-ON*
    }

    const auto registration = registration_future.get();

    return MirrorManager::Implementation::make(
      node,
      std::move(query),
      std::move(options),
      registration.query_id);
  }

  ~Implementation()
  {
    abandon_discovery = true;

    assert(discovery_thread.joinable());
    discovery_thread.join();
  }

  template<typename... Args>
  static MirrorManagerFuture make(Args&& ... args)
  {
    MirrorManagerFuture mmf;
    mmf._pimpl = rmf_utils::make_unique_impl<Implementation>(
      std::forward<Args>(args)...);

    return mmf;
  }
};

//==============================================================================
void MirrorManagerFuture::wait() const
{
  _pimpl->wait();
}

//==============================================================================
std::future_status MirrorManagerFuture::wait_for(
  const rmf_traffic::Duration& timeout) const
{
  return _pimpl->wait_for(timeout);
}

//==============================================================================
std::future_status MirrorManagerFuture::wait_until(
  const rmf_traffic::Time& time) const
{
  return _pimpl->wait_until(time);
}

//==============================================================================
bool MirrorManagerFuture::valid() const
{
  return _pimpl->valid();
}

//==============================================================================
MirrorManager MirrorManagerFuture::get()
{
  return _pimpl->get();
}

//==============================================================================
MirrorManagerFuture::MirrorManagerFuture()
{
  // Do nothing
}

//==============================================================================
MirrorManagerFuture make_mirror(
  const std::shared_ptr<rclcpp::Node>& node,
  rmf_traffic::schedule::Query query,
  MirrorManager::Options options)
{
  return MirrorManagerFuture::Implementation::make(
    node, std::move(query), std::move(options));
}

} // namespace schedule
} // namespace rmf_traffic_ros2
