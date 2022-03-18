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

#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_traffic_msgs/msg/traffic_dependency.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::Route convert(const rmf_traffic_msgs::msg::Route& from)
{
  rmf_traffic::Route route{from.map, convert(from.trajectory)};
  std::set<uint64_t> checkpoints(
    from.checkpoints.begin(), from.checkpoints.end());
  route.checkpoints(std::move(checkpoints));
  for (const auto& dep : from.dependencies)
  {
    route.add_dependency(
      dep.dependent_checkpoint,
      {
        dep.on_participant,
        dep.on_plan,
        dep.on_route,
        dep.on_checkpoint
      });
  }

  return route;
}

//==============================================================================
std::vector<rmf_traffic_msgs::msg::TrafficDependency> convert(
  const rmf_traffic::DependsOnParticipant& from)
{
  std::vector<rmf_traffic_msgs::msg::TrafficDependency> deps;
  for (const auto& [on_participant, p_dep] : from)
  {
    if (!p_dep.plan().has_value())
      continue;

    const auto on_plan = *p_dep.plan();
    for (const auto& [on_route, r_dep] : p_dep.routes())
    {
      for (const auto& [dependent_checkpoint, on_checkpoint] : r_dep)
      {
        deps.push_back(
          rmf_traffic_msgs::build<rmf_traffic_msgs::msg::TrafficDependency>()
          .dependent_checkpoint(dependent_checkpoint)
          .on_participant(on_participant)
          .on_plan(on_plan)
          .on_route(on_route)
          .on_checkpoint(on_checkpoint));
      }
    }
  }

  return deps;
}

//==============================================================================
rmf_traffic_msgs::msg::Route convert(const rmf_traffic::Route& from)
{
  std::vector<uint64_t> checkpoints(
    from.checkpoints().begin(),
    from.checkpoints().end());

  return rmf_traffic_msgs::build<rmf_traffic_msgs::msg::Route>()
    .map(from.map())
    .trajectory(convert(from.trajectory()))
    .checkpoints(std::move(checkpoints))
    .dependencies(convert(from.dependencies()));
}

//==============================================================================
std::vector<rmf_traffic::Route> convert(
  const std::vector<rmf_traffic_msgs::msg::Route>& from)
{
  std::vector<rmf_traffic::Route> output;
  for (const auto& msg : from)
    output.emplace_back(convert(msg));

  return output;
}

//==============================================================================
std::vector<rmf_traffic_msgs::msg::Route> convert(
  const std::vector<rmf_traffic::Route>& from)
{
  std::vector<rmf_traffic_msgs::msg::Route> output;
  for (const auto& msg : from)
    output.emplace_back(convert(msg));

  return output;
}

} // namespace rmf_traffic_ros2
