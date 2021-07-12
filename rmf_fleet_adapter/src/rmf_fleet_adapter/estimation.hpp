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

#ifndef SRC__FULL_CONTROL__ESTIMATION_HPP
#define SRC__FULL_CONTROL__ESTIMATION_HPP

#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>

#include <rmf_fleet_msgs/msg/robot_state.hpp>

#include <rclcpp/node.hpp>

//==============================================================================
struct TravelInfo
{
  using ArrivalEstimator =
    rmf_fleet_adapter::agv::RobotCommandHandle::ArrivalEstimator;
  using RequestCompleted =
    rmf_fleet_adapter::agv::RobotCommandHandle::RequestCompleted;

  std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints;
  ArrivalEstimator next_arrival_estimator;
  RequestCompleted path_finished_callback;
  rmf_utils::optional<std::size_t> last_known_wp;
  rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater;
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits;

  std::string fleet_name;
  std::string robot_name;

  std::optional<std::size_t> target_plan_index = std::nullopt;
};

//==============================================================================
void check_path_finish(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::RobotState& state,
  TravelInfo& info);

//==============================================================================
void estimate_path_traveling(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::RobotState& state,
  TravelInfo& info);

//==============================================================================
void estimate_midlane_state(
  const rmf_fleet_msgs::msg::Location& l,
  rmf_utils::optional<std::size_t> lane_start,
  std::size_t next_index,
  TravelInfo& info);

//==============================================================================
void estimate_state(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::Location& state,
  TravelInfo& info);

//==============================================================================
void estimate_waypoint(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::Location& state,
  TravelInfo& info);

#endif // SRC__FULL_CONTROL__ESTIMATION_HPP
