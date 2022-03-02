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

#ifndef SRC__RMF_FLEET_ADAPTER__TASKS__CLEAN_HPP
#define SRC__RMF_FLEET_ADAPTER__TASKS__CLEAN_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_task/State.hpp>
#include <rmf_task/requests/Clean.hpp>

#include <rmf_task_sequence/Phase.hpp>
#include <rmf_task_sequence/Event.hpp>

#include "../agv/internal_FleetUpdateHandle.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
std::shared_ptr<LegacyTask> make_clean(
  const rmf_task::ConstRequestPtr request,
  const agv::RobotContextPtr& context,
  const rmf_traffic::agv::Plan::Start clean_start,
  const rmf_traffic::Time deployment_time,
  const rmf_task::State finish_state);

void add_clean(
  const agv::FleetUpdateHandle::Implementation::ConstDockParamsPtr& dock_params,
  const rmf_traffic::agv::VehicleTraits& traits,
  agv::TaskDeserialization& deserialization,
  agv::TaskActivation& activation,
  std::function<rmf_traffic::Time()> clock);

} // namespace tasks
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__TASKS__CLEAN_HPP
