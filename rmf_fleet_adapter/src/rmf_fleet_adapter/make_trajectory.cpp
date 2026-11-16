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

#include <algorithm>
#include <iostream>
#include <limits>

//==============================================================================
rmf_traffic::Trajectory make_trajectory(
  const rmf_fleet_msgs::msg::RobotState& state,
  const rmf_traffic::agv::VehicleTraits& traits,
  bool& is_sitting)
{
  // TODO(MXG): Account for the multi-floor use case
  std::string map_name = state.location.level_name;
  if (map_name.empty())
    map_name = "B1";


  std::vector<Eigen::Vector3d> positions;
  positions.push_back({state.location.x, state.location.y, state.location.yaw});
  for (const auto& location : state.path)
    positions.push_back({location.x, location.y, location.yaw});

  const auto start_time = rmf_traffic_ros2::convert(state.location.t);

  auto trajectory = rmf_traffic::agv::Interpolate::positions(
    traits, start_time, positions);

  if (trajectory.size() < 2)
  {
    // If a robot state results in a single-point trajectory, then we should
    // make a temporary sitting trajectory.
    const Eigen::Vector3d p = positions.front();
    rmf_traffic::Trajectory sitting;
    sitting.insert(start_time, p, Eigen::Vector3d::Zero());

    const auto finish_time = start_time + std::chrono::seconds(10);
    sitting.insert(finish_time, p, Eigen::Vector3d::Zero());

    is_sitting = true;
    return sitting;
  }

  is_sitting = false;

  return trajectory;
}

//==============================================================================
namespace {
//==============================================================================
/// Distance along a polyline of the point on it that is closest to p, looking
/// no further than max_s along it. The first closest point wins.
double distance_along(
  const std::vector<Eigen::Vector2d>& line,
  const std::vector<double>& s,
  const Eigen::Vector2d& p,
  double max_s)
{
  double best_dist = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  for (std::size_t i = 0; i + 1 < line.size() && s[i] <= max_s; ++i)
  {
    const Eigen::Vector2d d = line[i+1] - line[i];
    const double length = s[i+1] - s[i];
    double u = 0.0;
    if (length > 1e-8)
      u = std::clamp((p - line[i]).dot(d) / (length * length), 0.0, 1.0);

    u = std::min(u, (max_s - s[i]) / std::max(length, 1e-8));
    const double dist = (p - (line[i] + u * d)).norm();
    if (dist < best_dist - 1e-9)
    {
      best_dist = dist;
      best_s = s[i] + u * length;
    }
  }

  return best_s;
}
} // anonymous namespace

//==============================================================================
std::optional<std::size_t> reached_checkpoint(
  const rmf_traffic::Trajectory& trajectory,
  const rmf_fleet_msgs::msg::RobotState& state)
{
  if (trajectory.size() < 2)
    return std::nullopt;

  // The trajectory as a polyline, and how far along it each waypoint is
  std::vector<Eigen::Vector2d> line;
  std::vector<double> s;
  for (const auto& wp : trajectory)
  {
    const Eigen::Vector2d p = wp.position().head<2>();
    s.push_back(line.empty() ? 0.0 : s.back() + (p - line.back()).norm());
    line.push_back(p);
  }

  // The robot has not reached the next path point that it has to visit
  constexpr double tolerance = 1e-3;
  double max_s = std::numeric_limits<double>::infinity();
  if (!state.path.empty())
  {
    const auto& next = state.path.front();
    max_s = distance_along(line, s, {next.x, next.y}, s.back()) - tolerance;
  }

  const auto& l = state.location;
  const double robot_s =
    distance_along(line, s, {l.x, l.y}, std::min(max_s, s.back()));

  // The last waypoint that the robot has passed
  std::size_t c = 0;
  while (c + 1 < s.size() && s[c+1] <= robot_s + tolerance && s[c+1] < max_s)
    ++c;

  // If the robot has not moved on from that waypoint, it may still be turning
  // there, so go back to where it arrived at it.
  if (robot_s < s[c] + tolerance)
  {
    while (c > 0 && s[c] - s[c-1] < tolerance)
      --c;
  }

  return c;
}

//==============================================================================
rmf_traffic::Trajectory make_trajectory(
  const rmf_traffic::Time start_time,
  const std::vector<rmf_fleet_msgs::msg::Location>& path,
  const rmf_traffic::agv::VehicleTraits& traits)
{
  std::vector<Eigen::Vector3d> positions;
  for (const auto& location : path)
    positions.push_back({location.x, location.y, location.yaw});

  return rmf_traffic::agv::Interpolate::positions(
    traits, start_time, positions);
}

//==============================================================================
rmf_traffic::Trajectory make_timed_trajectory(
  const std::vector<rmf_fleet_msgs::msg::Location>& path,
  const rmf_traffic::agv::VehicleTraits& traits)
{
  rmf_traffic::Trajectory output;
  for (const auto& location : path)
  {
    if (output.size() == 0)
    {
      output.insert(
        rmf_traffic_ros2::convert(location.t),
        Eigen::Vector3d(location.x, location.y, location.yaw),
        Eigen::Vector3d::Zero());
      continue;
    }

    std::vector<Eigen::Vector3d> positions;
    positions.reserve(2);
    positions.push_back(output.back().position());
    positions.push_back({location.x, location.y, location.yaw});

    rmf_traffic::Trajectory extension =
      rmf_traffic::agv::Interpolate::positions(traits,
        output.back().time(), positions);

    for (const auto& wp : extension)
      output.insert(wp);

    const auto wait_time = rmf_traffic_ros2::convert(location.t);
    const auto wait_duration = wait_time - output.back().time();

    if (wait_duration > std::chrono::milliseconds(1))
    {
      output.insert(
        wait_time, output.back().position(), Eigen::Vector3d::Zero());
    }
  }

  return output;
}

//==============================================================================
rmf_traffic::Route make_route(
  const rmf_fleet_msgs::msg::RobotState& state,
  const rmf_traffic::agv::VehicleTraits& traits,
  bool& is_sitting)
{
  return rmf_traffic::Route{
    state.location.level_name,
    make_trajectory(state, traits, is_sitting)
  };
}

//==============================================================================
rmf_traffic::Trajectory make_hold(
  const rmf_fleet_msgs::msg::Location& l,
  const rmf_traffic::Time t,
  rmf_traffic::Duration duration)
{
  rmf_traffic::Trajectory hold;
  const Eigen::Vector3d p{l.x, l.y, l.yaw};
  const Eigen::Vector3d v = Eigen::Vector3d::Zero();

  const auto start = t;
  const auto finish = start + duration;

  hold.insert(start, p, v);
  hold.insert(finish, p, v);

  return hold;
}
