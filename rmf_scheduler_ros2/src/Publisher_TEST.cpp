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

#define CATCH_CONFIG_MAIN
#include <rmf_utils/catch.hpp>

#include "Payload.hpp"
#include "Publisher.hpp"

#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <future>

namespace rmf::scheduler::test {

TEST_CASE("publish serialized message", "[Publisher]")
{
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_node");
  Publisher publisher{node};
  std::promise<bool> done;

  auto sub = node->create_subscription<std_msgs::msg::String>("test_topic",
      rclcpp::SystemDefaultsQoS{}, [&done](std_msgs::msg::String::SharedPtr msg)
      {
        REQUIRE(msg->data == "hello world");
        done.set_value(true);
      });

  std_msgs::msg::String msg;
  msg.data = "hello world";
  auto payload = make_serialized_message("std_msgs/String", "test_topic", msg);
  publisher.publish(payload);

  auto result = rclcpp::spin_until_future_complete(node,
      done.get_future(), std::chrono::seconds{1});
  REQUIRE(result == rclcpp::FutureReturnCode::SUCCESS);
}

}
