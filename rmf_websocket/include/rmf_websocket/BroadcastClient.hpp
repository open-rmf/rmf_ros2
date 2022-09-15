/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef RMF_WEBSOCKET__BROADCAST_CLIENT_HPP
#define RMF_WEBSOCKET__BROADCAST_CLIENT_HPP

#include <nlohmann/json.hpp>

#include <rclcpp/node.hpp>
#include <rmf_utils/impl_ptr.hpp>

#include <memory>
#include <set>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

namespace rmf_websocket {
//==============================================================================
// A wrapper around a websocket client for broadcasting states and logs for
// fleets, robots and tasks. A queue of json msgs is maintained and published
// in an internal thread whenever connection to a server is established.
class BroadcastClient : public std::enable_shared_from_this<BroadcastClient>
{
public:
  using ProvideJsonUpdates = std::function<std::vector<nlohmann::json>()>;

  /// \param[in] uri
  ///   "ws://localhost:9000"
  ///
  /// \param[in] node
  ///
  /// \param[in] on_open_connection_fn
  ///   Provided function callback will be called whenever the ws client
  ///   is connected to the server
  static std::shared_ptr<BroadcastClient> make(
    const std::string& uri,
    const std::shared_ptr<rclcpp::Node>& node,
    ProvideJsonUpdates on_open_connection_fn = nullptr);

  // Publish a single message
  void publish(const nlohmann::json& msg);

  // Publish a vector of messages
  void publish(const std::vector<nlohmann::json>& msgs);

  class Implementation;

private:
  BroadcastClient();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_websocket

#endif // RMF_WEBSOCKET__BROADCAST_CLIENT_HPP
