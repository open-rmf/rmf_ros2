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

#ifndef SRC__RMF_FLEET_ADAPTER__BROADCASTCLIENT_HPP
#define SRC__RMF_FLEET_ADAPTER__BROADCASTCLIENT_HPP

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <nlohmann/json.hpp>

#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>

#include <memory>
#include <set>
#include <queue>
#include <mutex>

namespace rmf_fleet_adapter {
//==============================================================================
// A wrapper around a websocket client for broadcasting states and logs for
// fleets, robots and tasks. A queue of json msgs is maintained and published
// in an internal thread whenever connection to a server is established.
class BroadcastClient : public std::enable_shared_from_this<BroadcastClient>
{
public:
  using WebsocketClient =
    websocketpp::client<websocketpp::config::asio_client>;
  using WebsocketMessagePtr = WebsocketClient::message_ptr;
  using ConnectionHDL = websocketpp::connection_hdl;
  using Connections = std::set<ConnectionHDL, std::owner_less<ConnectionHDL>>;

  static std::shared_ptr<BroadcastClient> make(
    std::size_t port,
    std::weak_ptr<TaskManager> task_managers);

  // This will add the json message to the internal _queue
  void publish(const nlohmann::json& msg);

  ~BroadcastClient();

private:
  BroadcastClient();
  std::weak_ptr<FleetUpdateHandle> _fleet_handle;
  std::shared_ptr<WebsocketClient> _server;
  Connections _connections;
  std::mutex _mutex;
};

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__BROADCASTCLIENT_HPP