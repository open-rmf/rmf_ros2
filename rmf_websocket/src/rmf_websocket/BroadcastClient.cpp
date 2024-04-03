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
  : _uri{std::move(uri)},
    _node{std::move(node)},
    _get_json_updates_cb{std::move(get_json_updates_cb)},
    _queue(1000),
    _io_service{},
    _endpoint(_uri,
      std::bind(&BroadcastClient::Implementation::log, this,
      std::placeholders::_1),
      &_io_service,
      std::bind(&BroadcastClient::Implementation::on_connect, this))
  {
    _consumer_thread = std::thread([this]()
        {
          _io_service.run();
        });

    _io_service.dispatch([this]()
      {
        _endpoint.connect();
      });
  }

  //============================================================================
  Implementation(Implementation&& other) = delete;

  //============================================================================
  Implementation& operator=(Implementation&& other) = delete;

  //============================================================================
  Implementation(const Implementation& other) = delete;

  //============================================================================
  Implementation operator=(const Implementation& other) = delete;

  //============================================================================
  void on_connect()
  {
    RCLCPP_INFO(
      this->_node->get_logger(),
      "Connected to server");

    if (_get_json_updates_cb)
    {
      auto messages = _get_json_updates_cb();

      for (auto queue_item : messages)
      {
        auto status = _endpoint.get_status();
        if (!status.has_value())
        {
          log("Endpoint has not yet been initiallized.");
          return;
        }

        if (status != ConnectionMetadata::ConnectionStatus::OPEN &&
          status != ConnectionMetadata::ConnectionStatus::CONNECTING)
        {
          // Attempt reconnect
          log("Disconnected during init.");
          return;
        }

        // Send
        auto ec = _endpoint.send(queue_item.dump());
        if (ec)
        {
          if (status != ConnectionMetadata::ConnectionStatus::CONNECTING)
          {
            log("Send failed. Attempting reconnection.");
          }
          return;
        }
      }
      RCLCPP_INFO(
        this->_node->get_logger(),
        "Sent all updates");
    }
    RCLCPP_INFO(
      this->_node->get_logger(),
      "Attempting queue flush if connected");
    _flush_queue_if_connected();
  }

  //============================================================================
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
    _io_service.dispatch([this]()
      {
        _flush_queue_if_connected();
      });
  }

  //============================================================================
  void publish(const std::vector<nlohmann::json>& msgs)
  {
    for (auto msg: msgs)
    {
      _queue.push(msg);
    }
    _io_service.dispatch([this]()
      {
        _flush_queue_if_connected();
      });
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
    _io_service.stop();
    _consumer_thread.join();
  }

private:
  //============================================================================
  void _flush_queue_if_connected()
  {
    while (!_queue.empty())
    {
      auto status = _endpoint.get_status();
      if (!status.has_value())
      {
        log("Endpoint has not yet been initiallized.");
        return;
      }

      if (status != ConnectionMetadata::ConnectionStatus::OPEN &&
        status != ConnectionMetadata::ConnectionStatus::CONNECTING)
      {
        return;
      }
      auto queue_item = _queue.front();
      if (!queue_item.has_value())
      {
        // Technically this should be unreachable as long as the client is
        // single threaded
        log("The queue was modified when it shouldnt have been");
        return;
      }
      auto ec = _endpoint.send(queue_item->dump());
      if (ec)
      {
        log("Sending message failed. Maybe due to intermediate disconnection");
        return;
      }
      _queue.pop_item();
    }
  }
  // create pimpl
  std::string _uri;
  boost::asio::io_service _io_service;
  std::shared_ptr<rclcpp::Node> _node;
  RingBuffer<nlohmann::json> _queue;
  ProvideJsonUpdates _get_json_updates_cb;
  std::atomic<bool> _stop;
  ClientWebSocketEndpoint _endpoint;

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
