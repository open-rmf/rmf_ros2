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

#include "BroadcastClient.hpp"

#include "agv/internal_FleetUpdateHandle.hpp"

namespace rmf_fleet_adapter {
//==============================================================================
std::shared_ptr<BroadcastClient> BroadcastClient::make(
  const std::string& uri,
  std::shared_ptr<agv::FleetUpdateHandle> fleet_handle)
{
  std::shared_ptr<BroadcastClient> client(new BroadcastClient());
  client->_uri = std::move(uri);
  client->_fleet_handle = std::move(fleet_handle);
  client->_shutdown = false;
  client->_connected = false;

  client->_client = std::make_shared<WebsocketClient>();
  // Initialize the Asio transport policy
  client->_client->init_asio();

  client->_client->set_open_handler(
    [c = client](websocketpp::connection_hdl)
    {
      c->_connected = true;
      const auto fleet = c->_fleet_handle.lock();
      if (!fleet)
        return;
      const auto impl = agv::FleetUpdateHandle::get(*fleet);
      for (const auto& [conext, mgr] : impl.task_managers)
      {
        // TODO(YV): Publish latest state and log
      }
      RCLCPP_INFO(
        impl.node->get_logger(),
        "BroadcastClient successfully connected to uri: [%s]",
        c->_uri.c_str());
    });

  client->_client->set_close_handler(
    [c = client](websocketpp::connection_hdl)
    {
      c->_connected = false;
    });

  client->_client->set_fail_handler()
  [c = client](websocketpp::connection_hdl)
  {
    c->_connected = false;
  };

  client->_thread = std::thread(
    [c = client]()
    {
      while (!shutdown)
      {
        const auto fleet = c->_fleet_handle.lock();
        if (!fleet)
          continue;
        const auto impl = agv::FleetUpdateHandle::get(*fleet);

        // Try to connect to the server if we are not connected yet
        if (!c->_connected)
        {
          websocketpp::lib::error_code ec;
          WebsocketClient::connection_ptr con = c->_client->get_connection(
            c->_uri, ec);
          if (ec)
          {
            RCLCPP_WARN(
              impl.node->get_logger(),
              "BroadcastClient unable to connect to [%s]. Please make sure "
              "server is running.",
              c->_uri.c_str());
            c->_connected = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            continue;
          }

          c->_handle = con->get_handle();
          c->_client->connect(con);
          // TODO(YV): Start asio io_service event loop
          c->_connected = true;
        }

        std::unique_lock<std::mutex> lock(c->_mutex);
        c->_cv.wait(lock,
          [c]()
          {
            return !c->_queue.empty();
          });

        while (!c->_queue.empty())
        {
          websocketpp::lib::error_code ec;
          const std::string& msg = c->_queue.front().dump();
          c->_client->send(c->_hdl, msg, websocketpp::frame::opcode::text, ec);
          if (ec)
          {
            RCLCPP_ERROR(
              impl.node->get_logger(),
              "BroadcastClient unable to publish message");
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
  // TODO(YV): lock a mutex
  _queue.push(msg);
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
  if (_thread.joinable)
  {
    _thread.join();
  }
}

} // namespace rmf_fleet_adapter
