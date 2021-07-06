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

#ifndef RMF_RXCPP__TRANSPORTDETAIL_HPP
#define RMF_RXCPP__TRANSPORTDETAIL_HPP

#include <rclcpp/rclcpp.hpp>

namespace rmf_rxcpp {
namespace detail {

template<typename Message>
class SubscriptionWrapper
{
public:

  SubscriptionWrapper(rclcpp::Node::WeakPtr node, std::string topic_name,
    const rclcpp::QoS& qos)
  : _node{std::move(node)}, _topic_name{std::move(topic_name)}, _qos{qos}
  {
    // no op
  }

  template<typename Subscriber>
  void operator()(const Subscriber& s)
  {
    if (auto node = _node.lock())
    {
      _subscription = node->template create_subscription<Message>(
        _topic_name,
        _qos,
        [s](typename Message::SharedPtr msg)
        {
          s.on_next(msg);
        });
    }
    else
    {
      s.on_completed();
    }
  }

private:

  rclcpp::Node::WeakPtr _node;
  std::string _topic_name;
  rclcpp::QoS _qos;
  typename rclcpp::Subscription<Message>::SharedPtr _subscription;
};

} // namespace detail
} // namespace rmf_rxcpp

#endif //RMF_RXCPP__TRANSPORTDETAIL_HPP
