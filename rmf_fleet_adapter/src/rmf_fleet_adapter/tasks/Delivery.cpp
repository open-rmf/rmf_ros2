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

#include "../phases/DispenseItem.hpp"
#include "../phases/GoToPlace.hpp"

#include "Delivery.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
Task make_delivery(
    const rmf_task_msgs::msg::Delivery& request,
    const agv::RobotContextPtr& context,
    rmf_traffic::agv::Plan::Start pickup_start,
    rmf_traffic::agv::Plan::Start dropoff_start)
{
  const auto pickup_wp =
      context->navigation_graph()
      .find_waypoint(request.pickup_place_name)->index();

  Task::PendingPhases phases;
  phases.push_back(
        phases::GoToPlace::make(context, std::move(pickup_start), pickup_wp));

  phases.push_back(
        std::make_unique<phases::DispenseItem>(
          context->node(),
          request.pickup_dispenser,
          context->itinerary().description().owner(),
          request.items,
          ))
}

} // namespace task
} // namespace rmf_fleet_adapter
