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

#include "../phases/GoToPlace.hpp"

#include "Clean.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
std::shared_ptr<Task> make_clean(
    const rmf_task::ConstRequestPtr request,
    const agv::RobotContextPtr& context,
    const rmf_traffic::agv::Plan::Start clean_start,
    const rmf_traffic::Time deployment_time,
    const rmf_task::agv::State finish_state)
{
  std::shared_ptr<const rmf_task::requests::Clean::Description> description = 
    std::dynamic_pointer_cast<
      const rmf_task::requests::Clean::Description>(request->description());

  if (description == nullptr)
    return nullptr;

  const auto start_waypoint = description->start_waypoint();
  const auto end_waypoint = description->end_waypoint();

  rmf_traffic::agv::Planner::Goal clean_goal{start_waypoint};
  rmf_traffic::agv::Planner::Goal end_goal{end_waypoint};

  // Determine the Start of the robot once it completes its cleaning job.
  // We assume that the robot will be back at start_waypoint once it finishes
  // cleaning.
  const auto end_start = [&]() -> rmf_traffic::agv::Planner::Start
    {
      auto initial_time = clean_start.time();
      double orientation = clean_start.orientation();
      // If the robot is not at its cleaning start_waypoint, we calculate the
      // time it takes to travel from clean_start to start_waypoint
      if (clean_start.waypoint() != start_waypoint)
      {
        rmf_traffic::agv::Planner::Goal goal{start_waypoint};
        const auto result = context->planner()->plan(clean_start, goal);
        // We assume we can always compute a plan
        const auto& trajectory =
          result->get_itinerary().back().trajectory();
        initial_time = *trajectory.finish_time();
        orientation = trajectory.back().position()[2];
      }      

      // Get the duration of the cleaning process
      const auto request_model = description->make_model(
        clean_start.time(),
        context->task_planner()->configuration().parameters());
      const auto invariant_duration = request_model->invariant_duration();

      return rmf_traffic::agv::Planner::Start{
        initial_time + invariant_duration,
        start_waypoint,
        orientation};
    }();

  Task::PendingPhases phases;
  phases.push_back(
        phases::GoToPlace::make(context, std::move(clean_start), clean_goal));
  phases.push_back(
        phases::GoToPlace::make(context, std::move(end_start), end_goal));

  return Task::make(
    request->id(),
    std::move(phases),
    context->worker(),
    deployment_time,
    finish_state,
    request);
}

} // namespace task
} // namespace rmf_fleet_adapter
