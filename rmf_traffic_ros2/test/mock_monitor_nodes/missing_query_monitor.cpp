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

#include <chrono>
#include <thread>

#include <rmf_traffic_ros2/schedule/internal_MonitorNode.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MissingQueryMonitorNode : public rmf_traffic_ros2::schedule::MonitorNode
{
public:
  MissingQueryMonitorNode(
    std::function<void(std::shared_ptr<rclcpp::Node>)> callback,
    const rclcpp::NodeOptions& options)
    : MonitorNode(callback, options, MonitorNode::no_automatic_setup)
  {}

  void announce_fail_over() override
  {
    // We want to wait a while before announcing the fail over because we want
    // the change in query information to be the trigger for mirrors to
    // recognize the fail over. But if we sleep here before issuing the fail
    // over notice, then the new schedule node won't be able to start until
    // after the fail over notice is sent. Instead we will do nothing at all
    // here and create a wall timer in main() that will issue the message.
  }

  std::shared_ptr<rclcpp::Node> create_new_schedule_node() override
  {
    auto database =
      std::make_shared<rmf_traffic::schedule::Database>(mirror.value().fork());

    // Drop the first registered query to trigger a missing query situation
    auto modified_registered_queries = registered_queries;
    modified_registered_queries.erase(modified_registered_queries.begin());

    auto node = std::make_shared<rmf_traffic_ros2::schedule::ScheduleNode>(
      1, // Bump the node version by one
      database,
      modified_registered_queries,
      rclcpp::NodeOptions());
    return node;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::promise<std::shared_ptr<rclcpp::Node>> active_node_promise;
  auto active_node_future = active_node_promise.get_future().share();

  auto node = std::make_shared<MissingQueryMonitorNode>(
    [&active_node_promise](
      std::shared_ptr<rclcpp::Node> new_active_schedule_node)
      {
        active_node_promise.set_value(new_active_schedule_node);
      },
    rclcpp::NodeOptions());
  node->setup();

  auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
    node, rmf_traffic::schedule::query_all());
  using namespace std::chrono_literals;
  bool success = false;
  const auto stop_time = std::chrono::steady_clock::now() + 10s;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    if (mirror_future.wait_for(0s) == std::future_status::ready)
    {
      RCLCPP_INFO(node->get_logger(), "Got mirror for monitor node");
      node->mirror = mirror_future.get();
      success = true;
      break;
    }
  }
  if (!success)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to start mirror");
    std::exit(1);
  }

  rclcpp::spin_until_future_complete(node, active_node_future);

  using namespace std::chrono_literals;
  if (active_node_future.wait_for(0s) == std::future_status::ready
    && rclcpp::ok())
  {
    auto active_schedule_node = active_node_future.get();
    // Delete the monitor to prevent it reacting to any future events
    auto fail_over_event_pub = node->fail_over_event_pub;
    node.reset();

    rclcpp::TimerBase::SharedPtr fail_over_timer;
    fail_over_timer = active_schedule_node->create_wall_timer(
      std::chrono::seconds(5),
      [&fail_over_timer, fail_over_event_pub]()
      {
        fail_over_event_pub->publish(
          rmf_traffic_msgs::build<rmf_traffic_msgs::msg::FailOverEvent>()
          .new_schedule_node_version(1));
        fail_over_timer.reset();
      });

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
