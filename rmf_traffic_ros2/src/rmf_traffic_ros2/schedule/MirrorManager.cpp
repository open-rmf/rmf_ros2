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

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Patch.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>

#include <rmf_traffic_msgs/msg/mirror_update.hpp>
#include <rmf_traffic_msgs/msg/participant.hpp>
#include <rmf_traffic_msgs/msg/participants.hpp>

#include <rmf_traffic_msgs/msg/fail_over_event.hpp>

#include <rmf_traffic_msgs/srv/register_query.hpp>
#include <rmf_traffic_msgs/srv/unregister_query.hpp>
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

using UnregisterQuery = rmf_traffic_msgs::srv::UnregisterQuery;
using UnregisterQueryClient = rclcpp::Client<UnregisterQuery>::SharedPtr;

using FailOverEvent = rmf_traffic_msgs::msg::FailOverEvent;
using FailOverEventSub = rclcpp::Subscription<FailOverEvent>::SharedPtr;


//==============================================================================
class MirrorManager::Implementation
{
public:

  rclcpp::Node& node;
  rmf_traffic::schedule::Query query;
  Options options;
  FailOverEventSub fail_over_event_sub;
  UnregisterQueryClient unregister_query_client;
  MirrorUpdateSub mirror_update_sub;
  ParticipantsInfoSub participants_info_sub;
  RequestChangesClient request_changes_client;
  uint64_t query_id = 0;
  rclcpp::TimerBase::SharedPtr update_timer;
  rclcpp::TimerBase::SharedPtr redo_query_registration_timer;
  RegisterQueryClient register_query_client;

  std::shared_ptr<rmf_traffic::schedule::Mirror> mirror;

  bool initial_update = true;

  rmf_traffic::schedule::Version next_minimum_version = 0;

  Implementation(
    rclcpp::Node& _node,
    rmf_traffic::schedule::Query _query,
    Options _options,
    uint64_t _query_id,
    UnregisterQueryClient _unregister_query_client)
  : node(_node),
    query(std::move(_query)),
    options(std::move(_options)),
    unregister_query_client(std::move(_unregister_query_client)),
    query_id(_query_id),
    mirror(std::make_shared<rmf_traffic::schedule::Mirror>())
  {
    setup_update_topics();

    request_changes_client = node.create_client<RequestChanges>(
      rmf_traffic_ros2::RequestChangesServiceName);

    fail_over_event_sub = node.create_subscription<FailOverEvent>(
      rmf_traffic_ros2::FailOverEventTopicName,
      rclcpp::SystemDefaultsQoS(),
      [&](const FailOverEvent::SharedPtr)
      {
        handle_fail_over_event();
      });
  }

  void setup_update_topics()
  {
    participants_info_sub = node.create_subscription<ParticipantsInfo>(
      ParticipantsInfoTopicName,
      rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local(),
      [&](const ParticipantsInfo::SharedPtr msg)
      {
        handle_participants_info(msg);
      });

    RCLCPP_DEBUG(node.get_logger(), "Registering to query topic %s",
      (QueryUpdateTopicNameBase + std::to_string(query_id)).c_str());
    mirror_update_sub = node.create_subscription<MirrorUpdate>(
      QueryUpdateTopicNameBase + std::to_string(query_id),
      rclcpp::SystemDefaultsQoS(),
      [&](const MirrorUpdate::SharedPtr msg)
      {
        handle_update(msg);
      });

    update_timer = node.create_wall_timer(5s, [&]() -> void
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
      RCLCPP_ERROR(
        node.get_logger(),
        "[rmf_traffic_ros2::MirrorManager] Failed to update participant info: %s",
        e.what());
    }
  }

  void handle_update(const MirrorUpdate::SharedPtr msg)
  {
    update_timer->reset();

    // Validate that this patch is for the query we are expecting
    if (convert(msg->query) != query)
    {
      // The schedule node is sending us someone else's query.
      RCLCPP_ERROR(
        node.get_logger(),
        "Received update for incorrect query; re-registering query");
      // Re-register ours to get a new topic.
      redo_query_registration();
      return;
    }

    try
    {
      const rmf_traffic::schedule::Patch patch = convert(msg->patch);

      std::mutex* update_mutex = options.update_mutex();
      if (update_mutex)
      {
        std::lock_guard<std::mutex> lock(*update_mutex);
        if (!mirror->update(patch))
        {
          request_update(mirror->latest_version());
        }
      }
      else
      {
        if (!mirror->update(patch))
        {
          request_update(mirror->latest_version());
        }
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        node.get_logger(),
        "[rmf_traffic_ros2::MirrorManager] Failed to deserialize Patch "
        "message: %s",
        e.what());
      // Get a full update in case we're just missing some information
      request_update();
    }
  }

  void handle_update_timeout()
  {
    RCLCPP_DEBUG(node.get_logger(), "Update timed out");
    request_update();
  }

  void request_update(std::optional<uint64_t> minimum_version = std::nullopt)
  {
    RequestChanges::Request request;
    request.query_id = query_id;
    if (minimum_version.has_value())
    {
      request.version = minimum_version.value();
      request.full_update = false;
      RCLCPP_INFO(
        node.get_logger(),
        "[rmf_traffic_ros2::MirrorManager::request_update] Requesting changes "
        "for query ID [%ld] since version [%ld]",
        request.query_id,
        request.version);
    }
    else
    {
      RCLCPP_INFO(
        node.get_logger(),
        "[rmf_traffic_ros2::MirrorManager::request_update] Requesting changes "
        "for query ID [%ld] since beginning of recorded history",
        request.query_id);
      request.version = 0;
      request.full_update = true;
    }
    request_changes_client->async_send_request(
      std::make_shared<RequestChanges::Request>(request),
      [&](const RequestChangesFuture response)
      {
        // Check how the schedule node handled the request. The actual queries
        // update will come separately over the query update topic; this is
        // just whether the request was handled successfully or not.
        auto value = *response.get();
        if (value.result == RequestChanges::Response::UNKNOWN_QUERY_ID)
        {
          redo_query_registration();
        }
      });
  }

  void redo_query_registration()
  {
    RCLCPP_DEBUG(node.get_logger(), "Redoing query registration");
    // Make sure nothing is truly coming in on this topic and triggering a
    // callback while we are remaking it
    mirror_update_sub.reset();

    register_query_client =
      node.create_client<RegisterQuery>(RegisterQueryServiceName);
    redo_query_registration_timer = node.create_wall_timer(
      100ms,
      std::bind(
        &MirrorManager::Implementation::redo_query_registration_callback,
        this));
  }

  void redo_query_registration_callback()
  {
    if (register_query_client->service_is_ready())
    {
      RegisterQuery::Request register_query_request;
      register_query_request.query = convert(query);
      register_query_client->async_send_request(
        std::make_shared<RegisterQuery::Request>(register_query_request),
        [this](const RegisterQueryFuture response)
        {
          this->query_id = response.get()->query_id;
          setup_update_topics();
          this->register_query_client.reset();
        });
      redo_query_registration_timer.reset();
    }
    else
    {
      RCLCPP_ERROR(
        node.get_logger(),
        "Failed to get query registry service");
    }
  }

  void handle_fail_over_event()
  {
    RCLCPP_INFO(node.get_logger(), "Handling fail over event for mirror");

    // Deleting the old one will shut it down
    unregister_query_client =
      node.create_client<UnregisterQuery>(UnregisterQueryServiceName);
    unregister_query_client->wait_for_service(std::chrono::milliseconds(100));
    // TODO(Geoff): If the above does not return a valid service, the destructor
    // will fail to send the unregister request. Do we care about that? Do we
    // even need to bother waiting for the service here?
  }

  ~Implementation()
  {
    // Only send the unregister request if the service is actually available.
    // This avoids a race condition between a replacement schedule node
    // starting up while this node shuts down.
    if (unregister_query_client->service_is_ready())
    {
      RCLCPP_WARN(node.get_logger(), "Unregistering query");
      UnregisterQuery::Request msg;
      msg.query_id = query_id;
      unregister_query_client->async_send_request(
        std::make_shared<UnregisterQuery::Request>(std::move(msg)));
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
const rmf_traffic::schedule::Viewer& MirrorManager::viewer() const
{
  return *_pimpl->mirror;
}

//==============================================================================
std::shared_ptr<rmf_traffic::schedule::Snappable>
MirrorManager::snapshot_handle() const
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

  rclcpp::Node& node;
  rmf_traffic::schedule::Query query;
  MirrorManager::Options options;

  using UnregisterQueryFuture = rclcpp::Client<UnregisterQuery>::SharedFuture;
  RegisterQueryClient register_query_client;
  UnregisterQueryClient unregister_query_client;

  std::atomic_bool abandon_discovery;
  std::atomic_bool registration_sent;
  std::thread discovery_thread;

  std::future<RegisterQuery::Response> registration_future;
  std::promise<RegisterQuery::Response> registration_promise;

  Implementation(
    rclcpp::Node& _node,
    rmf_traffic::schedule::Query _query,
    MirrorManager::Options _options)
  : node(_node),
    query(std::move(_query)),
    options(std::move(_options)),
    abandon_discovery(false),
    registration_sent(false)
  {
    register_query_client =
      node.create_client<RegisterQuery>(RegisterQueryServiceName);

    unregister_query_client =
      node.create_client<UnregisterQuery>(UnregisterQueryServiceName);

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
      ready = ready && unregister_query_client->wait_for_service(timeout);
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
          try
          {
            registration_promise.set_value(*response.get());
          }
          catch (const std::exception& e)
          {
            RCLCPP_ERROR(
              node.get_logger(),
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
    const auto registration = registration_future.get();

    return MirrorManager::Implementation::make(
      node,
      std::move(query),
      std::move(options),
      registration.query_id,
      std::move(unregister_query_client));
  }

  ~Implementation()
  {
    abandon_discovery = true;

    assert(discovery_thread.joinable());
    discovery_thread.join();

    if (registration_sent && registration_future.valid())
    {
      // If the future is still valid, then the MirrorManager was never created,
      // so its destructor will never be called. Therefore we should unregister
      // the query in this destructor instead if the query was successfully
      // registered.
      if (std::future_status::ready ==
        registration_future.wait_for(std::chrono::milliseconds(10)))
      {
        const auto registration_response = registration_future.get();
        if (!registration_response.error.empty())
        {
          RCLCPP_WARN(
            node.get_logger(),
            "[rmf_traffic_ros2::~MirrorManagerFuture] Error received "
            "while trying to register the query a MirrorManager: %s",
            registration_response.error.c_str());
        }
        else
        {
          UnregisterQuery::Request msg;
          msg.query_id = registration_response.query_id;
          unregister_query_client->async_send_request(
            std::make_shared<UnregisterQuery::Request>(std::move(msg)),
            [&](const UnregisterQueryFuture response_future)
            {
              const auto response = response_future.get();
              if (!response->error.empty())
              {
                RCLCPP_WARN(
                  node.get_logger(),
                  "[rmf_traffic_ros2::~MirrorManagerFuture] Error received "
                  "while trying to unregister the query of an uninstantiated "
                  "MirrorManager: %s",
                  response->error.c_str());
              }
            });
        }
      }
      else
      {
        RCLCPP_WARN(
          node.get_logger(),
          "[rmf_traffic_ros2::~MirrorManagerFuture] Timeout while waiting "
          "for the acknowlegment of a query registration.");
      }
    }
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
  rclcpp::Node& node,
  rmf_traffic::schedule::Query query,
  MirrorManager::Options options)
{
  return MirrorManagerFuture::Implementation::make(
    node, std::move(query), std::move(options));
}

} // namespace schedule
} // namespace rmf_traffic_ros2
