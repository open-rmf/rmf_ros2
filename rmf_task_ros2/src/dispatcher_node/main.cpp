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

#include <rclcpp/rclcpp.hpp>
#include <rmf_task_ros2/Dispatcher.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "~Initializing Dispatcher Node~" << std::endl;

  auto dispatcher = rmf_task_ros2::Dispatcher::make_node(
    "rmf_dispatcher_node");

  const auto& node = dispatcher->node();
  RCLCPP_INFO(node->get_logger(), "Starting task dispatcher node");
  dispatcher->spin();
  RCLCPP_INFO(node->get_logger(), "Closing down task dispatcher");
  rclcpp::shutdown();
}
