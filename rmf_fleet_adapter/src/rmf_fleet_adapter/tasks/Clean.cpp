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
    const rmf_task::requests::ConstCleanRequestPtr request,
    const agv::RobotContextPtr& context,
    const rmf_traffic::agv::Plan::Start clean_start,
    const rmf_traffic::Time deployment_time,
    const rmf_task::agv::State finish_state)
{
  rmf_traffic::agv::Planner::Goal clean_goal{request->start_waypoint()};
  auto end_start = request->location_after_clean(clean_start);
  rmf_traffic::agv::Planner::Goal end_goal{request->end_waypoint()};
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
