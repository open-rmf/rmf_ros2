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

Publisher::Publisher()
: rclcpp::Node("rmf_scheduler")
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
        this->get_logger(), "Unsupported payload type: %d", payload.type);
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

  // Technically we could use rmw_* functions here to avoid a copy, at the expense of
  // memory safety.
  rclcpp::SerializedMessage payload_sermsg{data.size()};
  std::copy(data.begin(), data.end(),
    payload_sermsg.get_rcl_serialized_message().buffer);

  // deserialize the serialized rmf_scheduler_msgs/SerializedMessage
  rmf_scheduler_msgs::msg::SerializedMessage rmf_msg;
  serializer.deserialize_message(&payload_sermsg, &rmf_msg);

  auto pub = this->create_generic_publisher(rmf_msg.topic_name,
      rmf_msg.message_type,
      rclcpp::SystemDefaultsQoS{});
  rclcpp::SerializedMessage inner_sermsg{rmf_msg.data.size()}; // this is the serialized form of the message to publish.
  std::copy(rmf_msg.data.begin(),
    rmf_msg.data.end(), inner_sermsg.get_rcl_serialized_message().buffer);
  pub->publish(inner_sermsg);
}

}
