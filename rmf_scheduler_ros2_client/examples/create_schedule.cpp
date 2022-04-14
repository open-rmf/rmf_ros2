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

#include <rclcpp/rclcpp.hpp>

#include <rmf_scheduler_client/payload.hpp>

#include <rmf_scheduler_msgs/srv/create_schedule.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("example_node");
  auto client = node->create_client<rmf_scheduler_msgs::srv::CreateSchedule>(
    "create_schedule");
  client->wait_for_service(5s);

  std_msgs::msg::String msg;
  msg.data = "hello world";

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::CreateSchedule::Request>();
  req->schedule.name = "my_schedule";
  req->schedule.schedule = "* * * * * *";
  req->schedule.payload = rmf::scheduler::make_serialized_message("test_topic",
      msg);

  auto resp_fut = client->async_send_request(req);
  rclcpp::spin_until_future_complete(node, resp_fut);
  if (resp_fut.get()->success)
  {
    std::cout << "success" << std::endl;
  }
}
