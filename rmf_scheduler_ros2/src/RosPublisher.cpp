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

#include <rclcpp/serialization.hpp>

#include <rmf_scheduler_msgs/msg/serialized_message_payload.hpp>

#include "RosPublisher.hpp"

namespace rmf::scheduler {

RosPublisher::RosPublisher(rclcpp::Node* node, const std::string& topic,
  const std::string& message_type)
: node(node)
{
  this->_rcl_publisher = this->node->create_generic_publisher(topic,
      message_type,
      rclcpp::SystemDefaultsQoS{});
}

void RosPublisher::publish(uint8_t type, const PayloadData& data)
{
  switch (type)
  {
    case rmf_scheduler_msgs::msg::Payload::PAYLOAD_TYPE_SERIALIZED_MESSAGE:
      return this->_publish_serialized_message(data);
    default:
      RCLCPP_ERROR(
        this->node->get_logger(), "Unsupported payload type: %d", type);
  }
}

void RosPublisher::_publish_serialized_message(const PayloadData& data)
{
  using Serializer =
    rclcpp::Serialization<rmf_scheduler_msgs::msg::SerializedMessagePayload>;
  static Serializer serializer;

  rclcpp::SerializedMessage payload_sermsg{data.size()};
  auto& rcl_payload =
    payload_sermsg.get_rcl_serialized_message();
  std::copy(&data.front(), &data.back(), rcl_payload.buffer);
  rcl_payload.buffer_length = data.size();

  this->_rcl_publisher->publish(payload_sermsg);
}

RosPublisherFactory::RosPublisherFactory(rclcpp::Node* node)
: node(node)
{
}

RosPublisher RosPublisherFactory::operator()(const std::string& topic,
  const std::string& message_type)
const
{
  return RosPublisher{this->node, topic, message_type};
}

}
