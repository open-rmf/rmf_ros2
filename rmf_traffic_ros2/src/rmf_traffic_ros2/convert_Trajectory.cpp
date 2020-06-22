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

#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/geometry/ConvexShape.hpp>

#include <rmf_traffic/geometry/Circle.hpp>

#include <unordered_map>
#include <vector>

namespace rmf_traffic_ros2 {

//==============================================================================
Eigen::Vector3d to_eigen(const std::array<double, 3>& values)
{
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

//==============================================================================
std::array<double, 3> from_eigen(const Eigen::Vector3d& values)
{
  return {values[0], values[1], values[2]};
}

//==============================================================================
rmf_traffic::Trajectory convert(const rmf_traffic_msgs::msg::Trajectory& from)
{
  rmf_traffic::Trajectory output;

  for (const auto& waypoint : from.waypoints)
  {
    output.insert(
      rmf_traffic::Time(rmf_traffic::Duration(waypoint.time)),
      to_eigen(waypoint.position),
      to_eigen(waypoint.velocity));
  }

  return output;
}

namespace {
//==============================================================================
rmf_traffic_msgs::msg::TrajectoryWaypoint convert_waypoint(
  const rmf_traffic::Trajectory::Waypoint& from)
{
  rmf_traffic_msgs::msg::TrajectoryWaypoint output;
  output.time = from.time().time_since_epoch().count();
  output.position = from_eigen(from.position());
  output.velocity = from_eigen(from.velocity());
  return output;
}

} // anonymous namespace

//==============================================================================
rmf_traffic_msgs::msg::Trajectory convert(const rmf_traffic::Trajectory& from)
{
  rmf_traffic_msgs::msg::Trajectory output;
  for (const auto& waypoint : from)
    output.waypoints.emplace_back(convert_waypoint(waypoint));

  return output;
}

} // namespace rmf_traffic_ros2
