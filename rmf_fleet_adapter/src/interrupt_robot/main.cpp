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

#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include <rmf_fleet_msgs/msg/interrupt_request.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

int help()
{
  std::cout <<
    R"raw(
  Usage:
    interrupt_robot [--resume] <id> <fleet_name> <robot_name> [labels...]
)raw";
  std::cout << std::endl;

  return 1;
}

int main(int argc, char* argv[])
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  if (args.size() <= 1)
    return help();

  bool resuming = false;
  if (args[1] == "--resume")
    resuming = true;

  if (resuming && args.size() < 5)
    return help();
  else if (args.size() < 4)
    return help();

  const std::size_t offset = resuming ? 1 : 0;

  const std::size_t id_index = 1 + offset;
  const std::size_t fleet_index = 2 + offset;
  const std::size_t robot_index = 3 + offset;
  const std::size_t label_index = 4 + offset;

  std::vector<std::string> labels;
  for (std::size_t i = label_index; i < args.size(); ++i)
    labels.push_back(args[i]);

  using Request = rmf_fleet_msgs::msg::InterruptRequest;
  auto request = rmf_fleet_msgs::build<Request>()
    .fleet_name(args[fleet_index])
    .robot_name(args[robot_index])
    .interrupt_id(args[id_index])
    .labels(std::move(labels))
    .type(resuming ? Request::TYPE_RESUME : Request::TYPE_INTERRUPT);

  const auto node = std::make_shared<rclcpp::Node>(
    args[fleet_index] + "_" + args[robot_index] + "_interrupt_robot");

  const auto publisher =
    node->create_publisher<Request>(
    rmf_fleet_adapter::InterruptRequestTopicName,
    rclcpp::SystemDefaultsQoS());

  publisher->publish(request);

  const auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [request, publisher]()
    {
      publisher->publish(request);
    });

  rclcpp::spin(node);
}
