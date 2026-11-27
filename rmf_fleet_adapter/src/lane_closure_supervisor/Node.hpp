/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef SRC__LANE_CLOSURE_SUPERVISOR__NODE_HPP
#define SRC__LANE_CLOSURE_SUPERVISOR__NODE_HPP

#include "LaneHolds.hpp"

#include <rclcpp/node.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/lane_closure_request.hpp>
#include <rmf_fleet_msgs/msg/lane_closure_supervisor_heartbeat.hpp>
#include <rmf_fleet_msgs/msg/lane_states.hpp>
#include <rmf_fleet_msgs/msg/robot_lane_request.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace rmf_fleet_adapter {
namespace lane_closure_supervisor {

//==============================================================================
/// Arbitrates lane closures and sends one RobotLaneRequest per robot.
/// No retry timer: requesters retry by re-sending.
class Node : public rclcpp::Node
{
public:

  explicit Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:

  using LaneClosureRequest = rmf_fleet_msgs::msg::LaneClosureRequest;
  using Heartbeat = rmf_fleet_msgs::msg::LaneClosureSupervisorHeartbeat;
  using RobotLaneRequest = rmf_fleet_msgs::msg::RobotLaneRequest;
  using LaneStates = rmf_fleet_msgs::msg::LaneStates;
  using FleetState = rmf_fleet_msgs::msg::FleetState;

  void _closure_request_update(const LaneClosureRequest& msg);

  /// Learn robots (never sends)
  void _fleet_state_update(const FleetState& msg);

  /// Store the report, confirm reopens (never sends)
  void _lane_states_update(const LaneStates& msg);

  void _send(
    const RobotId& key,
    const std::vector<uint64_t>& open_lanes,
    const std::vector<uint64_t>& close_lanes);

  void _publish_heartbeat();

  /// Lanes last reported closed for a robot
  const std::set<uint64_t>& _reported_closed_for(const RobotId& key) const;

  rclcpp::Subscription<LaneClosureRequest>::SharedPtr _closure_request_sub;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Subscription<LaneStates>::SharedPtr _lane_states_sub;
  rclcpp::Publisher<RobotLaneRequest>::SharedPtr _robot_lane_request_pub;
  rclcpp::Publisher<Heartbeat>::SharedPtr _heartbeat_pub;

  LaneHolds _lane_holds;

  /// Fleet -> robots in its latest FleetState
  std::map<std::string, std::set<std::string>> _present_robots;

  /// Fleet -> robot -> lanes reported closed (no entry: no report yet)
  std::map<std::string, std::map<std::string, std::set<uint64_t>>>
  _reported_closed;
};

} // namespace lane_closure_supervisor
} // namespace rmf_fleet_adapter

#endif // SRC__LANE_CLOSURE_SUPERVISOR__NODE_HPP
