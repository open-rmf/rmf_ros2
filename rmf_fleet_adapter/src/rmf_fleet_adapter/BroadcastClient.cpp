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

#include <iostream>

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
  client->_client.init_asio();
  client->_client.start_perpetual();

  client->_client.set_open_handler(
    [w = client->weak_from_this()](websocketpp::connection_hdl)
    {
      const auto c = w.lock();
      if (!c)
        return;
      c->_connected = true;
      const auto fleet = c->_fleet_handle.lock();
      if (!fleet)
        return;
      const auto impl = agv::FleetUpdateHandle::Implementation::get(*fleet);
      for (const auto& [conext, mgr] : impl.task_managers)
      {
        // TODO(YV): Publish latest state and log
      }
      RCLCPP_INFO(
        impl.node->get_logger(),
        "BroadcastClient successfully connected to uri: [%s]",
        c->_uri.c_str());
    });

  client->_client.set_close_handler(
    [w = client->weak_from_this()](websocketpp::connection_hdl)
    {
      const auto c = w.lock();
      if (!c)
        return;
      c->_connected = false;
    });

  client->_client.set_fail_handler(
  [w = client->weak_from_this()](websocketpp::connection_hdl)
  {
    const auto c = w.lock();
    if (!c)
      return;
    c->_connected = false;
  });

  client->_processing_thread = std::thread(
    [w = client->weak_from_this()]()
    {
      const auto c = w.lock();
      if (!c)
      {
        std::cout << "Unable to lock weak_from_this()" << std::endl;
        return;
      }
      std::cout << "Able to lock weak_from_this()" << std::endl;
      while (!c->_shutdown)
      {
        std::cout << "Inside while" << std::endl;
        const auto fleet = c->_fleet_handle.lock();
        if (!fleet)
        {
          std::cout << "Unable to lock fleet handle" << std::endl;
          continue;
        }
        std::cout << "Able to lock fleet handle" << std::endl;
        const auto impl = agv::FleetUpdateHandle::Implementation::get(*fleet);

        // Try to connect to the server if we are not connected yet
        if (!c->_connected)
        {
          std::cout << "Trying to connect to server " << c->_uri << std::endl;
          websocketpp::lib::error_code ec;
          WebsocketClient::connection_ptr con = c->_client.get_connection(
            c->_uri, ec);
          std::cout << "ec: " << ec << std::endl;
          if (ec)
          {
            std::cout << "Inside ec connection" << std::endl;
            RCLCPP_ERROR(
              impl.node->get_logger(),
              "BroadcastClient unable to connect to [%s]. Please make sure "
              "server is running.",
              c->_uri.c_str());
            c->_connected = false;
            std::cout << "Sleeping thread" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            std::cout << "Done sleeping thread" << std::endl;
            continue;
          }

          std::cout << "Connection successful" << std::endl;
          c->_client.send(c->_hdl, "Hello", websocketpp::frame::opcode::text, ec);
          std::cout << "ec: " << ec << std::endl;

          c->_hdl = con->get_handle();
          c->_client.connect(con);
          // TODO(YV): Start asio io_service event loop
          c->_connected = true;
        }

        std::cout << "Attending to items in queue" << std::endl;
        std::unique_lock<std::mutex> lock(c->_mutex);
        std::cout << "Locked mutex" << std::endl;
        c->_cv.wait(lock,
          [c]()
          {
            return !c->_queue.empty();
          });

        std::cout << "_queue not empty" << std::endl;

        while (!c->_queue.empty())
        {
          websocketpp::lib::error_code ec;
          const std::string& msg = c->_queue.front().dump();
          c->_client.send(c->_hdl, msg, websocketpp::frame::opcode::text, ec);
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

  std::cout << "Returning client" << std::endl;

  return client;

}

//==============================================================================
void BroadcastClient::publish(const nlohmann::json& msg)
{
  // TODO(YV): lock a mutex
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
  _client.stop_perpetual();
}

} // namespace rmf_fleet_adapter
