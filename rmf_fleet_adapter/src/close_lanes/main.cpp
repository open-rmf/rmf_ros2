/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/closed_lanes.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <iostream>
#include <vector>
#include <unordered_set>

//==============================================================================
int help(const std::string& extra_message)
{
  if (!extra_message.empty())
    std::cout << extra_message << "\n";

  std::cout <<
    R"raw(
  Usage:
    close_lanes <fleet_name> <lane0 [lane1 [lane2 ...]]>
)raw";
  std::cout << std::endl;

  return 1;
}

//==============================================================================
int main(int argc, char* argv[])
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (args.size() <= 2)
    return help("Not enough arguments");

  const std::string fleet_name = std::string(args[1]);

  std::vector<uint64_t> lanes;
  std::vector<std::size_t> failed_conversions;
  for (std::size_t i = 2; i < args.size(); ++i)
  {
    try
    {
      lanes.push_back(std::stoll(argv[i]));
    }
    catch (const std::exception&)
    {
      failed_conversions.push_back(i);
    }
  }

  if (!failed_conversions.empty())
  {
    std::string extra_message = "Could not convert argument";
    if (failed_conversions.size() > 1)
      extra_message += "s";

    for (const auto& f : failed_conversions)
      extra_message += " " + std::to_string(f);

    extra_message + " into integers";
    return help(extra_message);
  }

  std::cout << "Requesting [" << fleet_name << "] to close lanes { ";
  for (const auto& l : lanes)
    std::cout << l << " ";
  std::cout << "}..." << std::endl;

  const auto node = std::make_shared<rclcpp::Node>(fleet_name + "_close_lanes");
  std::promise<void> request_complete;
  std::shared_future<void> future = request_complete.get_future();

  rmf_fleet_msgs::msg::LaneRequest request;
  request.fleet_name = fleet_name;
  request.close_lanes = lanes;

  const auto publisher =
    node->create_publisher<rmf_fleet_msgs::msg::LaneRequest>(
    rmf_fleet_adapter::LaneClosureRequestTopicName,
    rclcpp::SystemDefaultsQoS().transient_local());

  publisher->publish(request);

  const auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [request, publisher]()
    {
      publisher->publish(request);
    });

  std::unordered_set<uint64_t> close_lanes(lanes.begin(), lanes.end());
  const auto listener =
    node->create_subscription<rmf_fleet_msgs::msg::ClosedLanes>(
    rmf_fleet_adapter::ClosedLaneTopicName,
    rclcpp::SystemDefaultsQoS().transient_local(),
    [&request_complete, fleet_name, close_lanes = std::move(close_lanes)](
      std::unique_ptr<rmf_fleet_msgs::msg::ClosedLanes> msg)
    {
      if (msg->fleet_name != fleet_name && !fleet_name.empty())
        return;

      auto still_open = close_lanes;
      for (const auto& l : msg->closed_lanes)
      {
        still_open.erase(l);
      }

      if (still_open.empty())
        request_complete.set_value();
    });

  rclcpp::spin_until_future_complete(node, future);

  if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    std::cout << "\n... The requested lanes are closed!" << std::endl;
  else
    std::cout << "\n... Interrupted" << std::endl;
}
