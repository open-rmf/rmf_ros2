/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "BroadcastClient.hpp"

#include "agv/internal_FleetUpdateHandle.hpp"

namespace rmf_fleet_adapter {
//==============================================================================
std::shared_ptr<BroadcastClient> BroadcastClient::make(
  const std::string& uri,
  std::weak_ptr<agv::FleetUpdateHandle> fleet_handle)
{
  std::shared_ptr<BroadcastClient> client(new BroadcastClient());
  client->_uri = std::move(uri);
  client->_fleet_handle = fleet_handle;
  client->_shutdown = false;
  client->_connected = false;

  // Initialize the Asio transport policy
  client->_client.clear_access_channels(websocketpp::log::alevel::all);
  client->_client.clear_error_channels(websocketpp::log::elevel::all);
  client->_client.init_asio();
  client->_client.start_perpetual();
  client->_client_thread = std::thread(
    [c = client]()
    {
      c->_client.run();
    });

  client->_client.set_open_handler(
    [c = client](websocketpp::connection_hdl)
    {
      c->_connected = true;
      const auto fleet = c->_fleet_handle.lock();
      if (!fleet)
        return;
      const auto& impl = agv::FleetUpdateHandle::Implementation::get(*fleet);
      for (const auto& [conext, mgr] : impl.task_managers)
      {
        // Publish all task logs to the server
        c->publish(mgr->task_log_updates());
      }
      RCLCPP_INFO(
        impl.node->get_logger(),
        "BroadcastClient successfully connected to uri: [%s]",
        c->_uri.c_str());
    });

  client->_client.set_close_handler(
    [c = client](websocketpp::connection_hdl)
    {
      c->_connected = false;
    });

  client->_client.set_fail_handler(
    [c = client](websocketpp::connection_hdl)
    {
      c->_connected = false;
    });

  client->_processing_thread = std::thread(
    [c = client]()
    {
      while (!c->_shutdown)
      {
        const auto fleet = c->_fleet_handle.lock();
        if (!fleet)
        {
          continue;
        }
        const auto& impl = agv::FleetUpdateHandle::Implementation::get(*fleet);

        // Try to connect to the server if we are not connected yet
        if (!c->_connected)
        {
          websocketpp::lib::error_code ec;
          WebsocketClient::connection_ptr con = c->_client.get_connection(
            c->_uri, ec);

          if (con)
          {
            c->_hdl = con->get_handle();
            c->_client.connect(con);
            // TOD(YV): Without sending a test payload, ec seems to be 0 even
            // when the client has not connected. Avoid sending this message.
            c->_client.send(c->_hdl, "Hello", websocketpp::frame::opcode::text,
            ec);
          }

          if (!con || ec)
          {
            RCLCPP_WARN(
              impl.node->get_logger(),
              "BroadcastClient unable to connect to [%s]. Please make sure "
              "server is running. Error msg: %s",
              c->_uri.c_str(),
              ec.message().c_str());
            c->_connected = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            continue;
          }

          RCLCPP_INFO(
            impl.node->get_logger(),
            "BroadcastClient successfully connected to [%s]",
            c->_uri.c_str());
          c->_connected = true;
        }

        std::unique_lock<std::mutex> lock(c->_wait_mutex);
        c->_cv.wait(lock,
        [c]()
        {
          return !c->_queue.empty();
        });

        while (!c->_queue.empty())
        {
          std::lock_guard<std::mutex> lock(c->_queue_mutex);
          websocketpp::lib::error_code ec;
          const std::string& msg = c->_queue.front().dump();
          c->_client.send(c->_hdl, msg, websocketpp::frame::opcode::text, ec);
          if (ec)
          {
            RCLCPP_ERROR(
              impl.node->get_logger(),
              "BroadcastClient unable to publish message: %s",
              ec.message().c_str());
            // TODO(YV): Check if we should re-connect to server
            break;
          }
          c->_queue.pop();
        }
      }
    });

  return client;

}

//==============================================================================
void BroadcastClient::publish(const nlohmann::json& msg)
{
  {
    const auto fleet = _fleet_handle.lock();
    if (!fleet)
      return;

    auto& impl = agv::FleetUpdateHandle::Implementation::get(*fleet);
    std::unique_lock<std::mutex> lock(*impl.update_callback_mutex);
    if (impl.update_callback)
      impl.update_callback(msg);
  }

  std::lock_guard<std::mutex> lock(_queue_mutex);
  _queue.push(msg);
  _cv.notify_all();
}

//==============================================================================
void BroadcastClient::publish(const std::vector<nlohmann::json>& msgs)
{
  {
    const auto fleet = _fleet_handle.lock();
    if (!fleet)
      return;

    auto& impl = agv::FleetUpdateHandle::Implementation::get(*fleet);
    std::unique_lock<std::mutex> lock(*impl.update_callback_mutex);
    if (impl.update_callback)
    {
      for (const auto& msg : msgs)
        impl.update_callback(msg);
    }
  }

  std::lock_guard<std::mutex> lock(_queue_mutex);
  for (const auto& msg : msgs)
    _queue.push(msg);
  _cv.notify_all();
}

//==============================================================================
BroadcastClient::BroadcastClient()
{
  // Do nothing
}

//==============================================================================
BroadcastClient::~BroadcastClient()
{
  _shutdown = true;
  if (_processing_thread.joinable())
  {
    _processing_thread.join();
  }
  if (_client_thread.joinable())
  {
    _client_thread.join();
  }
  _client.stop_perpetual();
}

} // namespace rmf_fleet_adapter
