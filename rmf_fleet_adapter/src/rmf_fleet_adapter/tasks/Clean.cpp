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

  // Presently the cleaning process is triggered through a Dock entry event for
  // the lane that leads to start_waypoint. We assume start_waypoint is
  // configured as a docking waypoint.
  // It is the responsibility of the implemented fleet adapter to correctly
  // update the position of the robot at the end of its cleaning process.
  const auto start_waypoint = description->start_waypoint();
  rmf_traffic::agv::Planner::Goal clean_goal{start_waypoint};

  Task::PendingPhases phases;
  phases.push_back(
    phases::GoToPlace::make(context, std::move(clean_start), clean_goal));

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
