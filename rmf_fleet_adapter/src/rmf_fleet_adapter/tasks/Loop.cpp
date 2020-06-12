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
    const rmf_task_msgs::msg::Loop& request,
    const agv::RobotContextPtr& context,
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> init_start,
    rmf_traffic::agv::Plan::Start loop_start,
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> loop_end)
{
  const auto& graph = context->navigation_graph();

  Task::PendingPhases phases;

  const auto start_wp =
      graph.find_waypoint(request.start_name)->index();

  const auto end_wp =
      graph.find_waypoint(request.finish_name)->index();

  if (init_start)
  {
    phases.push_back(
          phases::GoToPlace::make(context, *init_start, start_wp));
  }


  phases.push_back(
        phases::GoToPlace::make(context, loop_start, end_wp));

  for (std::size_t i=1; i < request.num_loops; ++i)
  {
    assert(loop_end);

    phases.push_back(
          phases::GoToPlace::make(context, *loop_end, start_wp));

    phases.push_back(
          phases::GoToPlace::make(context, loop_start, end_wp));
  }

  return Task::make(request.task_id, std::move(phases), context->worker());
}

} // namespace tasks
} // naemspace rmf_fleet_adapter
