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

#include "make_trajectory.hpp"

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>

#include <iostream>

//==============================================================================
rmf_traffic::Trajectory make_trajectory(
    const rmf_fleet_msgs::msg::RobotState& state,
    const rmf_traffic::agv::VehicleTraits& traits,
    bool& is_sitting)
{
  // TODO(MXG): Account for the multi-floor use case
  const std::string& map_name = state.location.level_name;

  std::vector<Eigen::Vector3d> positions;
  positions.push_back({state.location.x, state.location.y, state.location.yaw});
  for (const auto& location : state.path)
    positions.push_back({location.x, location.y, location.yaw});

  const auto start_time = rmf_traffic_ros2::convert(state.location.t);

  auto trajectory = rmf_traffic::agv::Interpolate::positions(
        map_name, traits, start_time, positions);

  if (trajectory.size() < 2)
  {
    // If a robot state results in a single-point trajectory, then we should
    // make a temporary sitting trajectory.
    const Eigen::Vector3d p = positions.front();
    rmf_traffic::Trajectory sitting{map_name};
    sitting.insert(
          start_time, traits.get_profile(), p, Eigen::Vector3d::Zero());

    const auto finish_time = start_time + std::chrono::seconds(10);
    sitting.insert(
          finish_time, traits.get_profile(), p, Eigen::Vector3d::Zero());

    is_sitting = true;
    return sitting;
  }

  is_sitting = false;

  return trajectory;
}

//==============================================================================
rmf_traffic::Trajectory make_hold(
    const rmf_fleet_msgs::msg::Location& l,
    const rmf_traffic::Time t,
    rmf_traffic::Duration duration,
    const rmf_traffic::agv::VehicleTraits& traits)
{
  rmf_traffic::Trajectory hold{l.level_name};
  const Eigen::Vector3d p{l.x, l.y, l.yaw};
  const Eigen::Vector3d v = Eigen::Vector3d::Zero();

  const auto start = t;
  const auto finish = start + duration;

  hold.insert(start, traits.get_profile(), p, v);
  hold.insert(finish, traits.get_profile(), p, v);

  return hold;
}
