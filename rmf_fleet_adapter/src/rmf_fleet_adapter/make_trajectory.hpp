/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__MAKE_TRAJECTORY_HPP
#define SRC__RMF_FLEET_ADAPTER__MAKE_TRAJECTORY_HPP

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Route.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rmf_fleet_msgs/msg/robot_state.hpp>

#include <optional>

//==============================================================================
rmf_traffic::Trajectory make_trajectory(
  const rmf_fleet_msgs::msg::RobotState& state,
  const rmf_traffic::agv::VehicleTraits& traits,
  bool& is_sitting);

//==============================================================================
/// Find the checkpoint of a trajectory made by make_trajectory(...) that a
/// robot has reached, given a later state of that robot: its location and the
/// points of its path that it still has to visit.
///
/// A checkpoint is a waypoint index of the trajectory, not a path index:
/// Interpolate::positions puts several waypoints between two stops (turning,
/// accelerating, cruising, braking) and leaves out path points that lie on a
/// straight line. The robot's location is projected onto the trajectory, up to
/// the next path point that it still has to visit, and the result is the last
/// waypoint that the robot has passed. If the robot is at a stop, where it may
/// still be turning, the result is the waypoint where it arrived there.
///
/// Returns std::nullopt if the trajectory has fewer than two waypoints.
std::optional<std::size_t> reached_checkpoint(
  const rmf_traffic::Trajectory& trajectory,
  const rmf_fleet_msgs::msg::RobotState& state);

//==============================================================================
rmf_traffic::Trajectory make_trajectory(
  const rmf_traffic::Time start_time,
  const std::vector<rmf_fleet_msgs::msg::Location>& path,
  const rmf_traffic::agv::VehicleTraits& traits);

//==============================================================================
rmf_traffic::Trajectory make_timed_trajectory(
  const std::vector<rmf_fleet_msgs::msg::Location>& path,
  const rmf_traffic::agv::VehicleTraits& traits);

//==============================================================================
rmf_traffic::Route make_route(
  const rmf_fleet_msgs::msg::RobotState& state,
  const rmf_traffic::agv::VehicleTraits& traits,
  bool& is_sitting);

//==============================================================================
rmf_traffic::Trajectory make_hold(
  const rmf_fleet_msgs::msg::Location& location,
  const rmf_traffic::Time time,
  rmf_traffic::Duration duration);

#endif // SRC__RMF_FLEET_ADAPTER__MAKE_TRAJECTORY_HPP
