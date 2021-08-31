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

#ifndef SRC__RMF_TRAFFIC_SCHEDULE__INTERNAL_MONITORNODE_HPP
#define SRC__RMF_TRAFFIC_SCHEDULE__INTERNAL_MONITORNODE_HPP

#include "internal_Node.hpp"  // For QueryMap and QuerySubscriberCountMap

#include <rclcpp/node.hpp>

#include <rmf_traffic_msgs/msg/heartbeat.hpp>
#include <rmf_traffic_msgs/msg/fail_over_event.hpp>
#include <rmf_traffic_msgs/msg/participant.hpp>
#include <rmf_traffic_msgs/msg/participants.hpp>
#include <rmf_traffic_msgs/msg/schedule_query.hpp>
#include <rmf_traffic_msgs/msg/schedule_queries.hpp>

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>

#include <optional>
#include <unordered_map>

namespace rmf_traffic_ros2 {
namespace schedule {

using namespace std::chrono_literals;

//==============================================================================
class MonitorNode : public rclcpp::Node
{
public:
  static struct NoAutomaticSetup{} no_automatic_setup;

  MonitorNode(
    std::function<void(std::shared_ptr<rclcpp::Node>)> callback,
    const rclcpp::NodeOptions& options,
    NoAutomaticSetup);

  MonitorNode(
    std::function<void(std::shared_ptr<rclcpp::Node>)> callback,
    const rclcpp::NodeOptions& options);

  void setup();

  std::chrono::milliseconds heartbeat_period = 1s;
  rclcpp::QoS heartbeat_qos_profile;
  rclcpp::SubscriptionOptions heartbeat_sub_options;
  using Heartbeat = rmf_traffic_msgs::msg::Heartbeat;
  using HeartbeatSub = rclcpp::Subscription<Heartbeat>;
  HeartbeatSub::SharedPtr heartbeat_sub;

  void start_heartbeat_listener();

  using FailOverEvent = rmf_traffic_msgs::msg::FailOverEvent;
  using FailOverEventPub = rclcpp::Publisher<FailOverEvent>;
  FailOverEventPub::SharedPtr fail_over_event_pub;

  void start_fail_over_event_broadcaster();
  virtual void announce_fail_over();

  using ScheduleQuery = rmf_traffic_msgs::msg::ScheduleQuery;
  using ScheduleQueries = rmf_traffic_msgs::msg::ScheduleQueries;
  rclcpp::Subscription<ScheduleQueries>::SharedPtr queries_info_sub;

  void start_data_synchronisers();

  int next_schedule_node_version = 1;
  virtual std::shared_ptr<rclcpp::Node> create_new_schedule_node();

  std::optional<rmf_traffic_ros2::schedule::MirrorManager> mirror;
  std::function<void(std::shared_ptr<rclcpp::Node>)> on_fail_over_callback;
  ScheduleNode::QueryMap registered_queries;
};

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // SRC__RMF_TRAFFIC_SCHEDULE__INTERNAL_MONITORNODE_HPP
