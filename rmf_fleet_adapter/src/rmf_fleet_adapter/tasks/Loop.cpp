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

#include "Loop.hpp"

#include "../phases/GoToPlace.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
std::shared_ptr<Task> make_loop(
    const rmf_task::ConstRequestPtr request,
    const agv::RobotContextPtr& context,
    const rmf_traffic::agv::Plan::Start start,
    const rmf_traffic::Time deployment_time,
    const rmf_task::agv::State finish_state)
{
  std::shared_ptr<const rmf_task::requests::Loop::Description> description =
    std::dynamic_pointer_cast<
      const rmf_task::requests::Loop::Description>(request->description());

  if (description == nullptr)
    return nullptr;

  const auto start_waypoint = description->start_waypoint();
  const auto finish_waypoint = description->finish_waypoint();

  const auto loop_start = [&]() -> rmf_traffic::agv::Planner::Start
    {
      if (start.waypoint() == start_waypoint)
        return start;

      rmf_traffic::agv::Planner::Goal goal{start_waypoint};

      const auto result = context->planner()->plan(start, goal);
      // We assume we can always compute a plan
      const auto& trajectory =
        result->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const double orientation = trajectory.back().position()[2];

      return rmf_traffic::agv::Planner::Start{
        finish_time,
        start_waypoint,
        orientation};
    }(); 

  const auto loop_end = [&]() -> rmf_traffic::agv::Planner::Start
    {
      if (loop_start.waypoint() == finish_waypoint)
        return loop_start;

      rmf_traffic::agv::Planner::Goal goal{finish_waypoint};

      const auto result = context->planner()->plan(loop_start, goal);
      // We assume we can always compute a plan
      const auto& trajectory =
        result->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const double orientation = trajectory.back().position()[2];

      return rmf_traffic::agv::Planner::Start{
        finish_time,
        finish_waypoint,
        orientation};
    }();

  Task::PendingPhases phases;
  phases.push_back(
    phases::GoToPlace::make(
      context, std::move(start), start_waypoint));

  phases.push_back(
    phases::GoToPlace::make(
      context, loop_start, finish_waypoint));

  for (std::size_t i = 1; i < description->num_loops(); ++i)
  {
    phases.push_back(
      phases::GoToPlace::make(
        context, loop_end, start_waypoint));

    phases.push_back(
      phases::GoToPlace::make(
        context, loop_start, finish_waypoint));
  }

  return Task::make(
    request->id(),
    std::move(phases),
    context->worker(),
    deployment_time,
    finish_state,
    request);
}

} // namespace tasks
} // namespace rmf_fleet_adapter
