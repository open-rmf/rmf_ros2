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

#include <rmf_traffic_ros2/schedule/MonitorNode.hpp>
#include <rmf_traffic_ros2/schedule/Node.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::promise<std::shared_ptr<rclcpp::Node>> active_node_promise;
  auto active_node_future = active_node_promise.get_future().share();
  auto monitor_node = rmf_traffic_ros2::schedule::make_monitor_node(
    [&active_node_promise](
      std::shared_ptr<rclcpp::Node> new_active_schedule_node)
    {
      active_node_promise.set_value(new_active_schedule_node);
    });

  rclcpp::spin_until_future_complete(monitor_node, active_node_future);

  using namespace std::chrono_literals;
  if (active_node_future.wait_for(0s) == std::future_status::ready
    && rclcpp::ok())
  {
    auto active_schedule_node = active_node_future.get();
    // Delete the monitor to prevent it reacting to any future events
    monitor_node.reset();

    RCLCPP_INFO(
      active_schedule_node->get_logger(),
      "Spinning up replacement schedule node");
    rclcpp::spin(active_schedule_node);
    RCLCPP_INFO(
      active_schedule_node->get_logger(),
      "Shutting down replacement schedule node");
  }

  rclcpp::shutdown();
}
