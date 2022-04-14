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

#pragma once

#include <rmf_scheduler_msgs/msg/payload.hpp>
#include <rmf_scheduler_msgs/msg/serialized_message.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <string_view>

namespace rmf::scheduler {

/// Creates a serialized message payload.
template<typename T>
rmf_scheduler_msgs::msg::Payload make_serialized_message(
  std::string_view topic_name,
  const T& message
)
{
  rclcpp::Serialization<T> inner_ser;
  rclcpp::SerializedMessage inner_sermsg;
  inner_ser.serialize_message(&message, &inner_sermsg);

  static rclcpp::Serialization<rmf_scheduler_msgs::msg::SerializedMessage>
  outer_ser;
  rmf_scheduler_msgs::msg::SerializedMessage outer_msg;
  outer_msg.message_type = rosidl_generator_traits::name<T>();;
  outer_msg.topic_name = topic_name;
  auto& rcl_inner = inner_sermsg.get_rcl_serialized_message();
  outer_msg.data =
    std::vector<uint8_t>{rcl_inner.buffer,
    rcl_inner.buffer + rcl_inner.buffer_length};

  rclcpp::SerializedMessage outer_sermsg;
  outer_ser.serialize_message(&outer_msg, &outer_sermsg);

  rmf_scheduler_msgs::msg::Payload payload;
  payload.type =
    rmf_scheduler_msgs::msg::Payload::PAYLOAD_TYPE_SERIALIZED_MESSAGE;
  auto& rcl_outer = outer_sermsg.get_rcl_serialized_message();
  using PayloadData = decltype(rmf_scheduler_msgs::msg::Payload::data);
  payload.data =
    PayloadData{rcl_outer.buffer, rcl_outer.buffer + rcl_outer.buffer_length};

  return payload;
}

}
