/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__EXECUTEPLAN_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__EXECUTEPLAN_HPP

#include "../agv/RobotContext.hpp"

#include <rmf_task/events/SimpleEventState.hpp>
#include <rmf_task_sequence/Event.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
struct ExecutePlan
{
  static std::optional<ExecutePlan> make(
    agv::RobotContextPtr context,
    rmf_traffic::PlanId plan_id,
    rmf_traffic::agv::Plan plan,
    rmf_traffic::schedule::Itinerary full_itinerary,
    const rmf_task_sequence::Event::AssignIDPtr& event_id,
    rmf_task::events::SimpleEventStatePtr state,
    std::function<void()> update,
    std::function<void()> finished,
    std::optional<rmf_traffic::Duration> tail_period);

  rmf_traffic::agv::Plan plan;
  rmf_traffic::Time finish_time_estimate;
  rmf_task_sequence::Event::ActivePtr sequence;
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__EXECUTEPLAN_HPP
