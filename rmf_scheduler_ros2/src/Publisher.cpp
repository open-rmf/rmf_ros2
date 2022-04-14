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

#include <rmf_scheduler_msgs/msg/serialized_message.hpp>

#include "Publisher.hpp"

namespace rmf::scheduler {

Publisher::Publisher(rclcpp::Node* node)
: node(node)
{
}

void Publisher::publish(const rmf_scheduler_msgs::msg::Payload& payload)
{
  switch (payload.type)
  {
    case rmf_scheduler_msgs::msg::Payload::PAYLOAD_TYPE_SERIALIZED_MESSAGE:
      return this->_publish_serialized_message(payload.data);
    default:
      RCLCPP_ERROR(
        this->node->get_logger(), "Unsupported payload type: %d", payload.type);
  }
}

void Publisher::_publish_serialized_message(const PayloadData& data)
{
  // This function can be confusing due to the multiple layers of serialization.
  //
  // First, the data from the param is the serialized rmf_scheduler_msgs/SerializedMessage.
  // We need to deserialize this to get the concrete rmf_scheduler_msgs/SerializedMessage.
  //
  // Within rmf_scheduler_msgs/SerializedMessage, there is another blob which contains the
  // serialized form of the message to publish. This serialized message will be published
  // with the generic publisher.

  using Serializer =
    rclcpp::Serialization<rmf_scheduler_msgs::msg::SerializedMessage>;
  static Serializer serializer;

  rclcpp::SerializedMessage payload_sermsg{data.size()};
  auto& rcl_payload =
    payload_sermsg.get_rcl_serialized_message();
  std::copy(&data.front(), &data.back(), rcl_payload.buffer);
  rcl_payload.buffer_length = data.size();

  // deserialize the serialized rmf_scheduler_msgs/SerializedMessage
  rmf_scheduler_msgs::msg::SerializedMessage rmf_msg;
  auto asd = payload_sermsg.size();
  serializer.deserialize_message(&payload_sermsg, &rmf_msg);

  rclcpp::SerializedMessage inner_sermsg{rmf_msg.data.size()};
  auto& rcl_inner =
    inner_sermsg.get_rcl_serialized_message();
  std::copy(rmf_msg.data.data(),
    rmf_msg.data.data() + rmf_msg.data.size(), rcl_inner.buffer);
  rcl_inner.buffer_length = rmf_msg.data.size();

  auto pub = this->node->create_generic_publisher(rmf_msg.topic_name,
      rmf_msg.message_type,
      rclcpp::SystemDefaultsQoS{});
  pub->publish(inner_sermsg);
}

}
