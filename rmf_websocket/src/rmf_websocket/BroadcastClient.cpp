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

#include <atomic>
#include <functional>
#include <optional>
#include <rmf_websocket/BroadcastClient.hpp>
#include <thread>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include "utils/RingBuffer.hpp"
#include "client/ClientWebSocketEndpoint.hpp"
#include <vector>

namespace rmf_websocket {

//==============================================================================
class BroadcastClient::Implementation
{
public:
  Implementation(
    const std::string& uri,
    const std::shared_ptr<rclcpp::Node>& node,
    ProvideJsonUpdates get_json_updates_cb)
  : _endpoint(_uri,
      std::bind(&BroadcastClient::Implementation::log, this,
      std::placeholders::_1)),
    _uri{std::move(uri)},
    _node{std::move(node)},
    _get_json_updates_cb{std::move(get_json_updates_cb)},
    _queue(1000)
  {
    _consumer_thread = std::thread([this]()
        {
          this->_processing_thread();
        });
  }

  void log(const std::string& str)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(),
      "%s",
      str.c_str()
    );
  }

  //============================================================================
  void publish(const nlohmann::json& msg)
  {
    /// _queue is thread safe. No need to lock.
    _queue.push(msg);
  }

  //============================================================================
  void publish(const std::vector<nlohmann::json>& msgs)
  {
    for (auto msg: msgs)
    {
      /// _queue is thread safe. No need to lock.
      _queue.push(msg);
    }
  }

  //============================================================================
  void set_queue_limit(std::optional<std::size_t> limit)
  {
    /// _queue is thread safe. No need to lock.
    if (limit.has_value())
      _queue.resize(limit.value());
  }

  //============================================================================
  ~Implementation()
  {
    _stop = true;
    _endpoint.interrupt_waits();
    _consumer_thread.join();
  }

private:
  //============================================================================
  /// Background consumer thread
  void _processing_thread()
  {
    _endpoint.connect();
    _check_conn_status_and_send(std::nullopt);
    while (!_stop)
    {

      auto item = _queue.pop_item();
      if (!item.has_value())
      {
        _queue.wait_for_message();
        continue;
      }
      _check_conn_status_and_send(item);
    }
  }

  //============================================================================
  /// Checks the connection status before sending the message
  void _check_conn_status_and_send(const std::optional<nlohmann::json>& item)
  {
    ConnectionMetadata::ptr metadata = _endpoint.get_metadata();
    if (metadata->get_status() != "Open")
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "Connection was lost.\n %s",
        metadata->debug_data().c_str());

      RCLCPP_INFO(
        _node->get_logger(),
        "Attempting reconnection");

      _endpoint.connect();
      _endpoint.wait_for_ready();

      RCLCPP_INFO(
        _node->get_logger(),
        "Connection Ready");

      /// Resend every time we reconnect. Useful for long term messages
      if (_get_json_updates_cb)
      {
        auto msgs = _get_json_updates_cb();
        for (auto msg: msgs)
        {
          _endpoint.send(msg.dump());
        }
      }
    }
    if (item.has_value())
    {
      _endpoint.send(item->dump());
    }
  }

  // create pimpl
  std::string _uri;
  std::shared_ptr<rclcpp::Node> _node;
  RingBuffer<nlohmann::json> _queue;
  ProvideJsonUpdates _get_json_updates_cb;
  ClientWebSocketEndpoint _endpoint;
  std::atomic<bool> _stop;
  std::thread _consumer_thread;
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
void BroadcastClient::set_queue_limit(std::optional<std::size_t> limit)
{
  _pimpl->set_queue_limit(limit);
}

//==============================================================================
BroadcastClient::BroadcastClient()
{
  // Do nothing
}

} // namespace rmf_websocket
