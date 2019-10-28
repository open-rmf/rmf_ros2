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

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Patch.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>

#include <rmf_traffic_msgs/msg/mirror_wakeup.hpp>

#include <rmf_traffic_msgs/srv/mirror_update.hpp>
#include <rmf_traffic_msgs/srv/register_query.hpp>
#include <rmf_traffic_msgs/srv/unregister_query.hpp>

#include <rclcpp/logging.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

using MirrorUpdate = rmf_traffic_msgs::srv::MirrorUpdate;
using MirrorUpdateClient = rclcpp::Client<MirrorUpdate>::SharedPtr;
using MirrorUpdateFuture = rclcpp::Client<MirrorUpdate>::SharedFuture;

using UnregisterQuery = rmf_traffic_msgs::srv::UnregisterQuery;
using UnregisterQueryClient = rclcpp::Client<UnregisterQuery>::SharedPtr;

using MirrorWakeup = rmf_traffic_msgs::msg::MirrorWakeup;
using MirrorWakeupSub = rclcpp::Subscription<MirrorWakeup>::SharedPtr;

//==============================================================================
class MirrorManager::Implementation
{
public:

  rclcpp::Node& node;
  Options options;
  MirrorUpdateClient mirror_update_client;
  UnregisterQueryClient unregister_query_client;
  MirrorWakeupSub mirror_wakeup_sub;

  MirrorUpdate::Request::SharedPtr request_msg;

  rmf_traffic::schedule::Mirror mirror;

  Implementation(
      rclcpp::Node& _node,
      Options _options,
      uint64_t _query_id,
      MirrorUpdateClient _mirror_update_client,
      UnregisterQueryClient _unregister_query_client)
    : node(_node),
      options(std::move(_options)),
      mirror_update_client(std::move(_mirror_update_client)),
      unregister_query_client(std::move(_unregister_query_client)),
      request_msg(std::make_shared<MirrorUpdate::Request>())
  {
    mirror_wakeup_sub = node.create_subscription<MirrorWakeup>(
          MirrorWakeupTopicName, rclcpp::SystemDefaultsQoS(),
          [&](const MirrorWakeup::SharedPtr msg)
    {
      trigger_wakeup(msg->latest_version);
    });

    request_msg->query_id = _query_id;
  }

  void trigger_wakeup(uint64_t minimum_version)
  {
    if(options.get_update_on_wakeup())
      update(minimum_version);
  }

  void update(
      uint64_t minimum_version,
      const rmf_traffic::Duration wait = rmf_traffic::Duration(0))
  {
    request_msg->latest_mirror_version = mirror.latest_version();
    request_msg->minimum_patch_version = minimum_version;
    const auto future = mirror_update_client->async_send_request(
          request_msg,
          [&](const MirrorUpdateFuture response_future)
    {
      const auto response = response_future.get();

      RCLCPP_INFO(
            node.get_logger(),
            "Updating mirror [" + std::to_string(response->patch.latest_version)
            + "]");

      try
      {
        mirror.update(convert(response->patch));
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(
              node.get_logger(),
              "[rmf_traffic_ros2::MirrorManager] Failed to deserialize Patch "
              "message: " + std::string(e.what()));
      }
    });

    if(wait > rmf_traffic::Duration(0))
      future.wait_for(wait);
  }

  ~Implementation()
  {
    UnregisterQuery::Request msg;
    msg.query_id = request_msg->query_id;
    unregister_query_client->async_send_request(
          std::make_shared<UnregisterQuery::Request>(std::move(msg)));
  }

  template<typename... Args>
  static MirrorManager make(Args&&... args)
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

  bool update_on_wakeup;

};

//==============================================================================
MirrorManager::Options::Options(bool update_on_wakeup)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{update_on_wakeup}))
{
  // Do nothing
}

//==============================================================================
bool MirrorManager::Options::get_update_on_wakeup() const
{
  return _pimpl->update_on_wakeup;
}

//==============================================================================
auto MirrorManager::Options::set_update_on_wakeup(bool choice) -> Options&
{
  _pimpl->update_on_wakeup = choice;
  return *this;
}

//==============================================================================
const rmf_traffic::schedule::Viewer& MirrorManager::viewer() const
{
  return _pimpl->mirror;
}

//==============================================================================
void MirrorManager::update(const rmf_traffic::Duration wait)
{
  _pimpl->update(_pimpl->mirror.latest_version(), wait);
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
MirrorManager::MirrorManager()
{
  // Do nothing
}

//==============================================================================
class MirrorManagerFuture::Implementation
{
public:

  rclcpp::Node& node;
  rmf_traffic::schedule::Query::Spacetime spacetime;
  MirrorManager::Options options;

  using RegisterQuery = rmf_traffic_msgs::srv::RegisterQuery;
  using RegisterQueryClient = rclcpp::Client<RegisterQuery>::SharedPtr;
  using RegisterQueryFuture = rclcpp::Client<RegisterQuery>::SharedFuture;
  using UnregisterQueryFuture = rclcpp::Client<UnregisterQuery>::SharedFuture;
  RegisterQueryClient register_query_client;
  MirrorUpdateClient mirror_update_client;
  UnregisterQueryClient unregister_query_client;

  std::atomic_bool abandon_discovery;
  std::atomic_bool registration_sent;
  std::thread discovery_thread;

  std::future<RegisterQuery::Response> registration_future;
  std::promise<RegisterQuery::Response> registration_promise;

  Implementation(
      rclcpp::Node& _node,
      rmf_traffic::schedule::Query::Spacetime _spacetime,
      MirrorManager::Options _options)
    : node(_node),
      spacetime(std::move(_spacetime)),
      options(std::move(_options)),
      abandon_discovery(false),
      registration_sent(false)
  {
    register_query_client =
        node.create_client<RegisterQuery>(RegisterQueryServiceName);

    mirror_update_client =
        node.create_client<MirrorUpdate>(MirrorUpdateServiceName);

    unregister_query_client =
        node.create_client<UnregisterQuery>(UnregisterQueryServiceName);

    registration_future = registration_promise.get_future();

    discovery_thread = std::thread([=](){ this->discover(); });
  }

  void discover()
  {
    const auto timeout = std::chrono::milliseconds(10);
    bool ready = false;
    while(!abandon_discovery && !ready)
    {
      ready = register_query_client->wait_for_service(timeout);
      ready = ready && mirror_update_client->wait_for_service(timeout);
      ready = ready && unregister_query_client->wait_for_service(timeout);
    }

    if(ready && !abandon_discovery)
    {
      RegisterQuery::Request register_query_request;
      register_query_request.query = convert(spacetime);
      register_query_client->async_send_request(
            std::make_shared<RegisterQuery::Request>(register_query_request),
            [&](const RegisterQueryFuture response)
      {
        try
        {
          registration_promise.set_value(*response.get());
        }
        catch(const std::exception& e)
        {
          RCLCPP_ERROR(
                node.get_logger(),
                "[rmf_traffic_ros2::MirrorManagerFuture] Exception while "
                "registering a query: " + std::string(e.what()));
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
          std::move(options),
          registration.query_id,
          std::move(mirror_update_client),
          std::move(unregister_query_client));
  }

  ~Implementation()
  {
    abandon_discovery = true;

    assert(discovery_thread.joinable());
    discovery_thread.join();

    if(registration_sent && registration_future.valid())
    {
      // If the future is still valid, then the MirrorManager was never created,
      // so its destructor will never be called. Therefore we should unregister
      // the query in this destructor instead if the query was successfully
      // registered.
      if(std::future_status::ready ==
         registration_future.wait_for(std::chrono::milliseconds(10)))
      {
        const auto registration_response = registration_future.get();
        if(!registration_response.error.empty())
        {
          RCLCPP_WARN(
                node.get_logger(),
                "[rmf_traffic_ros2::~MirrorManagerFuture] Error received "
                "while trying to register the query a MirrorManager: "
                + registration_response.error);
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
            if(!response->error.empty())
            {
              RCLCPP_WARN(
                    node.get_logger(),
                    "[rmf_traffic_ros2::~MirrorManagerFuture] Error received "
                    "while trying to unregister the query of an uninstantiated "
                    "MirrorManager: " + response->error);
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
  static MirrorManagerFuture make(Args&&... args)
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
    rmf_traffic::schedule::Query::Spacetime spacetime,
    MirrorManager::Options options)
{
  return MirrorManagerFuture::Implementation::make(
        node, std::move(spacetime), std::move(options));
}

} // namespace schedule
} // namespace rmf_traffic_ros2
