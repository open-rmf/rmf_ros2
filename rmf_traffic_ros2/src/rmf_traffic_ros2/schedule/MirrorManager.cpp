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
#include <rmf_traffic_ros2/schedule/ScheduleIdentity.hpp>

#include <rmf_traffic_msgs/msg/mirror_update.hpp>
#include <rmf_traffic_msgs/msg/participant.hpp>
#include <rmf_traffic_msgs/msg/participants.hpp>
#include <rmf_traffic_msgs/msg/schedule_query.hpp>
#include <rmf_traffic_msgs/msg/schedule_queries.hpp>

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

using ScheduleIdentity = rmf_traffic_msgs::msg::ScheduleIdentity;
using ScheduleIdentitySub = rclcpp::Subscription<ScheduleIdentity>::SharedPtr;

//==============================================================================
class MirrorManager::Implementation
{
public:

  // TODO(MXG): Remove all use of [&] and [this] from this implementation by
  // migrating the fields into a Shared structure and capturing that in the
  // lambdas.

  std::weak_ptr<rclcpp::Node> weak_node;
  rmf_traffic::schedule::Query query;
  uint64_t query_id;
  rmf_traffic_msgs::msg::ScheduleIdentity schedule_node_id;
  bool require_query_validation = false;
  std::list<MirrorUpdate::SharedPtr> stashed_query_updates;
  Options options;
  ScheduleIdentitySub schedule_startup_sub;
  MirrorUpdateSub mirror_update_sub;
  ParticipantsInfoSub participants_info_sub;
  rclcpp::Subscription<ScheduleQueries>::SharedPtr queries_info_sub;
  RequestChangesClient request_changes_client;
  rclcpp::TimerBase::SharedPtr update_timer;
  rclcpp::TimerBase::SharedPtr redo_query_registration_timer;
  rclcpp::TimerBase::SharedPtr reconnect_services_timer;
  RegisterQueryClient register_query_client;

  std::shared_ptr<rmf_traffic::schedule::Mirror> mirror;

  bool initial_update = true;

  rmf_traffic::schedule::Version next_minimum_version = 0;

  Implementation(
    const std::shared_ptr<rclcpp::Node>& node,
    rmf_traffic::schedule::Query _query,
    Options _options,
    uint64_t _query_id,
    ScheduleIdentity _schedule_node_id)
  : weak_node(node),
    query(std::move(_query)),
    query_id(_query_id),
    schedule_node_id(_schedule_node_id),
    options(std::move(_options)),
    mirror(std::make_shared<rmf_traffic::schedule::Mirror>())
  {
    setup_update_topics();
    setup_queries_sub();

    request_changes_client = node->create_client<RequestChanges>(
      RequestChangesServiceName);

    schedule_startup_sub = node->create_subscription<ScheduleIdentity>(
      rmf_traffic_ros2::ScheduleStartupTopicName,
      rclcpp::SystemDefaultsQoS(),
      [&](const ScheduleIdentity::SharedPtr msg)
      {
        handle_startup_event(*msg);
      });
  }

  bool reconnect_schedule(
    const rmf_traffic_msgs::msg::ScheduleIdentity& node_id)
  {
    const bool need_reconnect =
      schedule::reconnect_schedule(schedule_node_id, node_id);
    if (need_reconnect)
      reconnect_services();

    return need_reconnect;
  }

  bool validate_service_response(
    const rmf_traffic_msgs::msg::ScheduleIdentity& node_id)
  {
    if (schedule_node_id.node_uuid != node_id.node_uuid)
    {
      if (!reconnect_schedule(node_id))
      {
        // If we don't need to reconnect to this new schedule then we at
        // least need to reconnect our services, because this service
        // response came from an old schedule node.
        reconnect_services();
      }
      return false;
    }

    return true;
  }

  bool validate_meta_update(
    const rmf_traffic_msgs::msg::ScheduleIdentity& node_id)
  {
    // This function returns true if:
    // 1. The node of the incoming msg is the same as the one we're expecting
    // 2. The node of the incoming msg is supplanting the one we're expecting
    const bool expected_node = schedule_node_id.node_uuid == node_id.node_uuid;
    const bool new_node = reconnect_schedule(node_id);

    if (new_node)
    {
      // If a new node has appeared, we should reconnect the services.
      reconnect_services();
    }

    return expected_node || new_node;
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
        if (!validate_meta_update(msg->node_id))
          return;

        RCLCPP_INFO(
          node->get_logger(),
          "Mirror handling new sync of %lu queries "
          "from schedule node [%s]",
          msg->queries.size(),
          msg->node_id.node_uuid.c_str());

        // Find what should be our query based on our query ID
        std::optional<uint64_t> our_query = std::nullopt;
        for (std::size_t ii = 0; ii < msg->queries.size(); ++ii)
        {
          if (msg->query_ids[ii] == query_id)
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

    RCLCPP_INFO(node->get_logger(), "Registering to query topic %s",
      (QueryUpdateTopicNameBase + std::to_string(query_id)).c_str());
    mirror_update_sub = node->create_subscription<MirrorUpdate>(
      QueryUpdateTopicNameBase + std::to_string(query_id),
      rclcpp::ServicesQoS().reliable().keep_last(5000),
      [this](const MirrorUpdate::SharedPtr msg)
      {
        handle_update(msg);
      });

    // At this point we know we have the correct ID for our query
    require_query_validation = false;
    process_stashed_queries();

    update_timer = node->create_wall_timer(
      20s,
      [this]() -> void
      {
        handle_update_timeout();
      });
  }

  void handle_participants_info(const ParticipantsInfo::SharedPtr msg)
  {
    if (!validate_meta_update(msg->node_id))
      return;

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
      handle_update(std::move(msg));
    }
    stashed_query_updates.clear();
  }

  // Call this if the stashed queries are found to be for an incorrect query
  void dump_stashed_queries()
  {
    stashed_query_updates.clear();
  }

  void apply_patch(
    const std::shared_ptr<rclcpp::Node>& node,
    const rmf_traffic_msgs::msg::SchedulePatch& msg,
    const bool is_remedial)
  {
    const rmf_traffic::schedule::Patch patch = convert(msg);
    if (!mirror->update(patch) && !is_remedial)
    {
      std::string patch_base = patch.base_version() ?
        std::to_string(*patch.base_version()) : std::string("any");
      std::string mirror_version = mirror->latest_version() ?
        std::to_string(*mirror->latest_version()) : std::string("none");
      RCLCPP_WARN(
        node->get_logger(),
        "Failed to update using patch for DB version %lu "
        "(mirror version: %s, patch base: %s); requesting new update",
        patch.latest_version(),
        mirror_version.c_str(),
        patch_base.c_str());

      request_update(mirror->latest_version());
    }
  }

  void handle_update(const MirrorUpdate::SharedPtr msg)
  {
    update_timer->reset();
    const auto node = weak_node.lock();
    if (!node)
      return;

    // Verify that the expected schedule node version sent the update
    if (need_reconnection(schedule_node_id, msg->node_id))
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Received query update from unexpected newer schedule node [%s], "
        "expected [%s]; validating query registration",
        msg->node_id.node_uuid.c_str(),
        schedule_node_id.node_uuid.c_str());
      require_query_validation = true;
      stashed_query_updates.clear();
    }
    else if (schedule_node_id.node_uuid != msg->node_id.node_uuid)
    {
      // Ignore this message because it's coming from an out-of-date version
      RCLCPP_WARN(
        node->get_logger(),
        "Received query update from outdated schedule node [%s], "
        "expected [%s]; ignoring",
        msg->node_id.node_uuid.c_str(),
        schedule_node_id.node_uuid.c_str());
      return;
    }

    if (require_query_validation)
    {
      // Stash this query update until the query has been verified as correct
      RCLCPP_INFO(
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
        apply_patch(node, msg->patch, msg->is_remedial_update);
      }
      else
      {
        apply_patch(node, msg->patch, msg->is_remedial_update);
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

    RCLCPP_INFO(
      node->get_logger(),
      "Requesting new schedule update because update timed out");
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
      RCLCPP_INFO(
        node->get_logger(),
        "[rmf_traffic_ros2::MirrorManager::request_update] Requesting changes "
        "for query ID [%ld] since version [%lu]",
        request.query_id,
        request.version);
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

    if (request_changes_client && request_changes_client->service_is_ready())
    {
      request_changes_client->async_send_request(
        std::make_shared<RequestChanges::Request>(request),
        [this, minimum_version](const RequestChangesFuture response)
        {
          // Check how the schedule node handled the request. The actual queries
          // update will come separately over the query update topic; this is
          // just whether the request was handled successfully or not.
          auto value = *response.get();
          if (!validate_service_response(value.node_id))
            return;

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
                  "[MirrorManager::request_update] Failed to request an "
                  "update for query ID [%ld] from the beginning of recorded "
                  "history. Error message: %s",
                  query_id,
                  value.error.c_str());
              }
            }
          }
        });
    }
    else
    {
      RCLCPP_WARN(
        node->get_logger(),
        "[MirrorManager::request_update] Waiting for change request service "
        "to reconnect for [%s] before sending a change request.",
        schedule_node_id.node_uuid.c_str());
    }
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
      1s,
      std::bind(
        &MirrorManager::Implementation::redo_query_registration_callback,
        this));
  }

  void redo_query_registration_callback()
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    if (register_query_client && register_query_client->service_is_ready())
    {
      RCLCPP_INFO(
        node->get_logger(),
        "[MirrorManager] Redoing query registration: Calling service");
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
          if (!validate_service_response(msg->node_id))
            return;

          this->query_id = msg->query_id;
          RCLCPP_INFO(
            node->get_logger(),
            "[MirrorManager] Redoing query registration: Got new ID %lu",
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
      RCLCPP_WARN(
        node->get_logger(),
        "[MirrorManager::redo_query_registration] Waiting for schedule "
        "services to connect to [%s] to register a query",
        schedule_node_id.node_uuid.c_str());
    }
  }

  void reconnect_services()
  {
    register_query_client = nullptr;
    request_changes_client = nullptr;
    mirror->reset();

    const auto node = weak_node.lock();
    if (!node)
      return;

    // We will wait one second before attempting discovery. This should give
    // enough time for the old traffic schedule to shut down and not be
    // discovered by our client. This is probably not the most robust way to
    // reconnect services, so we should reconsider this strategy when time
    // permits.
    reconnect_services_timer = node->create_wall_timer(
      1s,
      [this]()
      {
        const auto node = this->weak_node.lock();
        if (!node)
          return;

        register_query_client =
        node->create_client<RegisterQuery>(RegisterQueryServiceName);

        request_changes_client = node->create_client<RequestChanges>(
          RequestChangesServiceName);

        reconnect_services_timer = nullptr;
      });
  }

  void handle_startup_event(const ScheduleIdentity& node_id)
  {
    const auto node = weak_node.lock();
    if (!node)
      return;

    if (!reconnect_schedule(node_id))
      return;

    // We need to validate that the replacement schedule node has our query
    // correctly. This will be reset to false once we have received the query
    // info for the new version and confirmed that it has our query stored in it
    // correctly.
    //
    // While true, all query updates received will be stashed in a list, and
    // will be popped in FIFO order and processed when this is reset to false.
    require_query_validation = true;

    RCLCPP_INFO(
      node->get_logger(),
      "Handling schedule startup event. "
      "New expected schedule node [%s].",
      node_id.node_uuid.c_str());
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
      registration.query_id,
      registration.node_id);
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
