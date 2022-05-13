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
#include <rmf_scheduler_msgs/msg/serialized_message_payload.hpp>

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
  using PayloadData = decltype(rmf_scheduler_msgs::msg::Payload::data);

  rclcpp::Serialization<T> inner_ser;
  rclcpp::SerializedMessage inner_sermsg;
  inner_ser.serialize_message(&message, &inner_sermsg);
  auto& rcl_msg = inner_sermsg.get_rcl_serialized_message();

  rmf_scheduler_msgs::msg::Payload payload;
  payload.type =
    rmf_scheduler_msgs::msg::Payload::PAYLOAD_TYPE_SERIALIZED_MESSAGE;
  payload.topic = topic_name;
  payload.message_type = rosidl_generator_traits::name<T>();
  payload.data = PayloadData{rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length};

  return payload;
}

}
