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
#include <thread>
#include <atomic>

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

  /// \param[in] uri
  ///   "ws://localhost:9000"
  static std::shared_ptr<BroadcastClient> make(
    const std::string& uri,
    std::weak_ptr<agv::FleetUpdateHandle> fleet_handle);

  // Publish a single message
  void publish(const nlohmann::json& msg);

  // Publish a vector of messages
  void publish(const std::vector<nlohmann::json>& msgs);

  ~BroadcastClient();

private:
  BroadcastClient();
  std::string _uri;
  std::weak_ptr<agv::FleetUpdateHandle> _fleet_handle;
  WebsocketClient _client;
  websocketpp::connection_hdl _hdl;
  std::mutex _wait_mutex;
  std::mutex _queue_mutex;
  std::condition_variable _cv;
  std::queue<nlohmann::json> _queue;
  std::thread _processing_thread;
  std::thread _client_thread;
  std::atomic_bool _connected;
  std::atomic_bool _shutdown;
};

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__BROADCASTCLIENT_HPP
