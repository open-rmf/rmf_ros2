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

#ifndef RMF_WEBSOCKET__BROADCAST_SERVER_HPP
#define RMF_WEBSOCKET__BROADCAST_SERVER_HPP

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <nlohmann/json.hpp>

#include <optional>
#include <rmf_utils/impl_ptr.hpp>

using Server = websocketpp::server<websocketpp::config::asio>;

namespace rmf_websocket {

//==============================================================================
/// This BroadcastServer is a wrapper of a websocket server. User need to
/// specify the api_msg_type, and provide a callback function. The provided
/// callback will be called when the specified msg_type is received. Note that
/// this will spawn a seperate thread to host the web socket server
class BroadcastServer
{
public:
  enum class ApiMsgType
  {
    TaskStateUpdate,
    TaskLogUpdate,
    FleetStateUpdate,
    FleetLogUpdate,
  };

  using ApiMessageCallback = std::function<void(const nlohmann::json&)>;

  /// Add a callback to convert from a Description into an active Task.
  ///
  /// \param[in] port
  ///   server url port number
  ///
  /// \param[in] callback
  ///   callback function when the message is received
  ///
  /// \param[in] msg_selection
  ///   selected msg type to listen. Will listen to all msg if nullopt
  ///
  static std::shared_ptr<BroadcastServer> make(
    const int port,
    ApiMessageCallback callback,
    std::optional<ApiMsgType> msg_selection = std::nullopt);

  /// Start Server
  void start();

  /// Stop Server
  void stop();

  /// Get the string name of the ApiMsgType
  static const std::string to_string(const ApiMsgType& type);

  class Implementation;

private:
  BroadcastServer();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_websocket

#endif // RMF_WEBSOCKET__BROADCAST_SERVER_HPP
