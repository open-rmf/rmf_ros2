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

#ifndef RMF_FLEET_ADAPTER__TEST__MOCK__WEBSOCKETSERVER
#define RMF_FLEET_ADAPTER__TEST__MOCK__WEBSOCKETSERVER

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <nlohmann/json.hpp>

#include <iostream>
#include <exception>
#include <functional>
#include <thread>

using Server = websocketpp::server<websocketpp::config::asio>;

namespace rmf_fleet_adapter_test {

//==============================================================================
/// This MockWebSocketServer is a wrapper of a websocket server. User need to
/// specify the api_msg_type, and provide a callback function. The provided
/// callback will be called when the specified msg_type is received. Note that
/// this will spawn a seperate thread to host the web socket server
class MockWebSocketServer
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
  MockWebSocketServer(
    const int port,
    ApiMessageCallback callback,
    std::optional<ApiMsgType> msg_selection = std::nullopt)
  : _data(std::make_shared<Data>(std::move(callback), std::move(msg_selection)))
  {
    std::cout << "Run websocket server with port " << port << std::endl;
    try
    {
      // Hide all logs from websocketpp
      _data->echo_server.clear_access_channels(websocketpp::log::alevel::all);
      // enable reuse of address if server is respawned
      _data->echo_server.set_reuse_addr(true);
      _data->echo_server.init_asio();

      // Register our message handler
      using websocketpp::lib::placeholders::_1;
      using websocketpp::lib::placeholders::_2;
      _data->echo_server.set_message_handler(
        [w = _data->weak_from_this()](const auto& hdl, const auto& msg)
        {
          if (const auto data = w.lock())
            data->on_message(hdl, msg);
        });


      _data->echo_server.listen(port);
      _data->echo_server.start_accept();
    }
    catch (const websocketpp::exception& e)
    {
      std::cout << e.what() << std::endl;
    }
    catch (...)
    {
      std::cout << "other exception" << std::endl;
    }
  }

  /// Start Server
  void start()
  {
    std::cout << "Start Mock Server" << std::endl;
    // Start the ASIO io_service run loop
    _server_thread = std::thread(
      [data = _data]() { data->echo_server.run(); });
  }

  /// Stop Server
  void stop()
  {
    std::cout << "Stop Mock Server" << std::endl;
    if (_server_thread.joinable())
    {
      _data->echo_server.stop_listening();
      _data->echo_server.stop();
      // TODO: properly close all connections
      _server_thread.join();
    }
  }

  ~MockWebSocketServer()
  {
    stop();
  }

  /// Get the string name of the ApiMsgType
  static const std::string to_string(const ApiMsgType& type)
  {
    const std::unordered_map<ApiMsgType, std::string> map(
      {{ApiMsgType::TaskStateUpdate, "task_state_update"},
        {ApiMsgType::TaskLogUpdate, "task_log_update"},
        {ApiMsgType::FleetStateUpdate, "fleet_state_update"},
        {ApiMsgType::FleetLogUpdate, "fleet_log_update"}});
    return map.at(type);
  }

private:

  struct Data : public std::enable_shared_from_this<Data>
  {
    Data(ApiMessageCallback callback, std::optional<ApiMsgType> msg_selection)
    : selection(std::move(msg_selection)),
      msg_callback(std::move(callback))
    {
      // Do nothing
    }

    /// private class variables
    Server echo_server;
    std::optional<ApiMsgType> selection;
    ApiMessageCallback msg_callback;

    /// Define an internal callback to handle incoming messages
    void on_message(websocketpp::connection_hdl, Server::message_ptr msg)
    {
      const auto msg_string = msg->get_payload();
      if (!msg_string.empty())
      {
        const nlohmann::json msg_json = nlohmann::json::parse(msg_string);

        if (selection)
        {
          const auto target_msg_type = to_string(*selection);
          const auto type = msg_json.at("type");
          if (type == target_msg_type)
            msg_callback(msg_json.at("data"));
        }
        else
          msg_callback(msg_json);
      }
    }
  };
  std::shared_ptr<Data> _data;
  std::thread _server_thread;
};


} // namespace rmf_fleet_adapter_test

#endif // RMF_FLEET_ADAPTER__TEST__MOCK__WEBSOCKETSERVER
