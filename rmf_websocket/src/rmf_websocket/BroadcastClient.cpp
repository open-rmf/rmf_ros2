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

#include <rmf_websocket/BroadcastClient.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

namespace rmf_websocket {

//==============================================================================
class BroadcastClient::Implementation
{
public:
  using WebsocketClient =
    websocketpp::client<websocketpp::config::asio_client>;
  using WebsocketMessagePtr = WebsocketClient::message_ptr;
  using ConnectionHDL = websocketpp::connection_hdl;
  using Connections = std::set<ConnectionHDL, std::owner_less<ConnectionHDL>>;

  Implementation(
    const std::string& uri,
    const std::shared_ptr<rclcpp::Node>& node,
    ProvideJsonUpdates get_json_updates_cb)
  : _uri{std::move(uri)},
    _node{std::move(node)},
    _get_json_updates_cb{std::move(get_json_updates_cb)}
  {
    _shutdown = false;
    _connected = false;

    // Initialize the Asio transport policy
    _client.clear_access_channels(websocketpp::log::alevel::all);
    _client.clear_error_channels(websocketpp::log::elevel::all);
    _client.init_asio();
    _client.start_perpetual();
    _client_thread = std::thread(
      [c = this]()
      {
        c->_client.run();
      });

    _client.set_open_handler(
      [c = this](websocketpp::connection_hdl)
      {
        c->_connected = true;

        if (c->_get_json_updates_cb)
          c->publish(c->_get_json_updates_cb());

        RCLCPP_INFO(
          c->_node->get_logger(),
          "BroadcastClient successfully connected to uri: [%s]",
          c->_uri.c_str());
      });

    _client.set_close_handler(
      [c = this](websocketpp::connection_hdl)
      {
        c->_connected = false;
      });

    _client.set_fail_handler(
      [c = this](websocketpp::connection_hdl)
      {
        c->_connected = false;
      });

    _processing_thread = std::thread(
      [c = this]()
      {
        while (!c->_shutdown)
        {
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
              c->_client.send(c->_hdl, "Hello",
              websocketpp::frame::opcode::text,
              ec);
            }

            if (!con || ec)
            {
              RCLCPP_WARN(
                c->_node->get_logger(),
                "BroadcastClient unable to connect to [%s]. Please make sure "
                "server is running. Error msg: %s",
                c->_uri.c_str(),
                ec.message().c_str());
              c->_connected = false;
              std::this_thread::sleep_for(std::chrono::milliseconds(2000));
              continue;
            }

            RCLCPP_INFO(
              c->_node->get_logger(),
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
                c->_node->get_logger(),
                "BroadcastClient unable to publish message: %s",
                ec.message().c_str());
              // TODO(YV): Check if we should re-connect to server
              break;
            }
            c->_queue.pop();
          }
        }
      });
  }

  // Publish a single message
  void publish(const nlohmann::json& msg)
  {
    std::lock_guard<std::mutex> lock(_queue_mutex);
    _queue.push(msg);
    _cv.notify_all();
  }

  // Publish a vector of messages
  void publish(const std::vector<nlohmann::json>& msgs)
  {
    std::lock_guard<std::mutex> lock(_queue_mutex);
    for (const auto& msg : msgs)
      _queue.push(msg);
    _cv.notify_all();
  }

  ~Implementation()
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

private:

  // create pimpl
  std::string _uri;
  std::shared_ptr<rclcpp::Node> _node;
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
  ProvideJsonUpdates _get_json_updates_cb;
};

//==============================================================================
std::shared_ptr<BroadcastClient> BroadcastClient::make(
  const std::string& uri,
  const std::shared_ptr<rclcpp::Node>& node,
  ProvideJsonUpdates on_open_connection_fn)
{
  auto client = std::shared_ptr<BroadcastClient>(new BroadcastClient());
  client->_pimpl =
    rmf_utils::make_unique_impl<Implementation>(
    uri, node, on_open_connection_fn);
  return client;
}

//==============================================================================
void BroadcastClient::publish(const nlohmann::json& msg)
{
  _pimpl->publish(msg);
}

//==============================================================================
void BroadcastClient::publish(const std::vector<nlohmann::json>& msgs)
{
  _pimpl->publish(msgs);
}

//==============================================================================
BroadcastClient::BroadcastClient()
{
  // Do nothing
}

} // namespace rmf_websocket
