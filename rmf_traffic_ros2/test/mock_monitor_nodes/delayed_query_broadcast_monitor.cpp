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

#include <rmf_traffic_ros2/schedule/internal_MonitorNode.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

// This specialised schedule node delays broadcasting of its registered queries
// for five seconds from startup. It is used to test that MirrorManagers will
// correctly handle the case of a fail-over event occuring and the
// MirrorManager not receiving a registered queries broadcast (to validate the
// new schedule node has its query registered correctly), and so making a
// manual query itself.
class DelayedQueryBroadcastScheduleNode
  : public rmf_traffic_ros2::schedule::ScheduleNode
{
public:
  DelayedQueryBroadcastScheduleNode(
    NodeVersion node_version_,
    std::shared_ptr<rmf_traffic::schedule::Database> database_,
    QueryMap registered_queries_,
    const rclcpp::NodeOptions& options)
    : ScheduleNode(
        node_version_,
        database_,
        registered_queries_,
        options)
  {
    timer = create_wall_timer(5s, [this]() -> void
      {
        RCLCPP_WARN(
          get_logger(),
          "Enabling query broadcasts and doing one broadcast");
        broadcast_enabled = true;
        broadcast_queries();
        timer.reset();
      });
  }

  void broadcast_queries() override
  {
    if (broadcast_enabled)
    {
      ScheduleNode::broadcast_queries();
    }
  }

  bool broadcast_enabled = false;
  rclcpp::TimerBase::SharedPtr timer;
};


// This specialised version of the monitor node will launch the above
// DelayedQueryBroadcastScheduleNode instead of a regular schedule node
// when a fail-over event occurs.
class DelayedQueryBroadcastMonitorNode
  : public rmf_traffic_ros2::schedule::MonitorNode
{
public:
  DelayedQueryBroadcastMonitorNode(
    std::function<void(std::shared_ptr<rclcpp::Node>)> callback,
    const rclcpp::NodeOptions& options)
    : MonitorNode(callback, options, MonitorNode::no_automatic_setup)
  {}

  std::shared_ptr<rclcpp::Node> create_new_schedule_node() override
  {
    auto database =
      std::make_shared<rmf_traffic::schedule::Database>(mirror.value().fork());

    auto node =
      std::make_shared<DelayedQueryBroadcastScheduleNode>(
        1, // Bump the node version by one
        database,
        registered_queries,
        rclcpp::NodeOptions());
    return node;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::promise<std::shared_ptr<rclcpp::Node>> active_node_promise;
  auto active_node_future = active_node_promise.get_future().share();

  auto node = std::make_shared<DelayedQueryBroadcastMonitorNode>(
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
    node.reset();

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
