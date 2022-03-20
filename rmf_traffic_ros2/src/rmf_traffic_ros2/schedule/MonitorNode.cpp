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

#include "internal_MonitorNode.hpp"

#include "internal_Node.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>

#include <rclcpp/executors.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
MonitorNode::MonitorNode(
  std::function<void(std::shared_ptr<rclcpp::Node>)> callback,
  const rclcpp::NodeOptions& options,
  NoAutomaticSetup)
: Node("rmf_traffic_schedule_monitor", options),
  heartbeat_qos_profile(1),
  on_fail_over_callback(callback)
{
}

//==============================================================================
MonitorNode::MonitorNode(
  std::function<void(std::shared_ptr<rclcpp::Node>)> callback,
  const rclcpp::NodeOptions& options)
: MonitorNode(callback, options, no_automatic_setup)
{
  setup();
}

//==============================================================================
void MonitorNode::setup()
{
  // Period, in milliseconds, for listening for a heartbeat signal from the
  // primary node in the redundant pair
  declare_parameter<int>("heartbeat_period", 1000);
  heartbeat_period = std::chrono::milliseconds(
    get_parameter("heartbeat_period").as_int());

  // Version number to use for the replacement schedule node.
  // The default is 1, given the original schedule node starts with 0
  declare_parameter<int>("next_version", 1);
  next_schedule_node_version = get_parameter("next_version").as_int();

  start_heartbeat_listener();
  start_fail_over_event_broadcaster();
  start_data_synchronisers();
}

//==============================================================================
void MonitorNode::start_heartbeat_listener()
{
  heartbeat_qos_profile
  .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
  .liveliness_lease_duration(heartbeat_period);

  heartbeat_sub_options.event_callbacks.liveliness_callback =
    [this](rclcpp::QOSLivelinessChangedInfo& event) -> void
    {
      RCLCPP_INFO(
        get_logger(),
        "Liveliness changed event:");
      RCLCPP_INFO(
        get_logger(),
        "  alive_count: %d",
        event.alive_count);
      RCLCPP_INFO(
        get_logger(),
        "  not_alive_count: %d",
        event.not_alive_count);
      RCLCPP_INFO(
        get_logger(),
        "  alive_count_change: %d",
        event.alive_count_change);
      RCLCPP_INFO(
        get_logger(),
        "  not_alive_count_change: %d",
        event.not_alive_count_change);
      if ( // The ideal pattern is 0, -1, 1, 1
        event.alive_count == 0 && event.alive_count_change < 0 &&
        event.not_alive_count > 0 && event.not_alive_count_change > 0)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Detected death of primary schedule node");
        on_fail_over_callback(create_new_schedule_node());
        announce_fail_over();
      }
    };
  heartbeat_sub = create_subscription<Heartbeat>(
    rmf_traffic_ros2::HeartbeatTopicName,
    heartbeat_qos_profile,
    [this](const typename Heartbeat::SharedPtr msg) -> void
    {
      (void) msg;
      RCLCPP_INFO(
        get_logger(),
        "Received heartbeat from primary schedule node");
    },
    heartbeat_sub_options);
  RCLCPP_INFO(
    get_logger(),
    "Set up heartbeat listener on %s with liveliness lease duration of %lu ms",
    heartbeat_sub->get_topic_name(),
    heartbeat_period.count());
}

//==============================================================================
void MonitorNode::start_fail_over_event_broadcaster()
{
  fail_over_event_pub = create_publisher<FailOverEvent>(
    rmf_traffic_ros2::FailOverEventTopicName,
    rclcpp::ServicesQoS().reliable());
}

//==============================================================================
void MonitorNode::announce_fail_over()
{
  if (rclcpp::ok())
  {
    RCLCPP_INFO(get_logger(), "Announcing fail over");
    auto message = FailOverEvent();
    fail_over_event_pub->publish(message);
  }
  else
  {
    // Skip announcing the fail over because other nodes shouldn't bother doing
    // anything to handle it when the system is shutting down.
    RCLCPP_INFO(
      get_logger(),
      "Not announcing fail over because ROS is shutting down");
  }
}


//==============================================================================
void MonitorNode::start_data_synchronisers()
{
  queries_info_sub = create_subscription<ScheduleQueries>(
    rmf_traffic_ros2::QueriesInfoTopicName,
    rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local(),
    [=](const ScheduleQueries::SharedPtr msg)
    {
      RCLCPP_INFO(
        get_logger(),
        "Handling new sync of %ld queries from primary node",
        msg->queries.size());
      // Delete past sync'd data
      registered_queries.clear();

      // Fill up with the new sync'd data
      for (uint64_t ii = 0; ii < msg->ids.size(); ++ii)
      {
        registered_queries.insert(
          {msg->ids[ii], rmf_traffic_ros2::convert(msg->queries[ii])});
      }
    });
}

//==============================================================================
std::shared_ptr<rclcpp::Node> MonitorNode::create_new_schedule_node()
{
  auto database = std::make_shared<Database>(mirror.value().fork());
  auto node = std::make_shared<rmf_traffic_ros2::schedule::ScheduleNode>(
    next_schedule_node_version,
    database,
    registered_queries,
    rclcpp::NodeOptions());
  return node;
}

//==============================================================================
std::shared_ptr<rclcpp::Node> make_monitor_node(
  std::function<void(std::shared_ptr<rclcpp::Node>)> callback,
  const rclcpp::NodeOptions& options,
  std::chrono::seconds startup_timeout)
{
  auto node = std::make_shared<MonitorNode>(callback, options);

  auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
    node, rmf_traffic::schedule::query_all());

  using namespace std::chrono_literals;

  const auto stop_time = std::chrono::steady_clock::now() + startup_timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    if (mirror_future.wait_for(0s) == std::future_status::ready)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Got mirror for monitor node");
      node->mirror = mirror_future.get();
      return node;
    }
  }

  RCLCPP_WARN(
    node->get_logger(),
    "Timeout while trying to connect to traffic schedule");
  return nullptr;
}

} // namespace schedule
} // namespace rmf_traffic_ros2
