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

//==============================================================================
rmf_traffic::Trajectory make_trajectory(
  const rmf_fleet_msgs::msg::RobotState& state,
  const rmf_traffic::agv::VehicleTraits& traits,
  bool& is_sitting);

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
