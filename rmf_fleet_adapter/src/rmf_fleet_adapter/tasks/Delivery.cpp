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
#include "../phases/IngestItem.hpp"
#include "../phases/GoToPlace.hpp"

#include "Delivery.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
std::shared_ptr<Task> make_delivery(
    const rmf_task_msgs::msg::Delivery& request,
    const agv::RobotContextPtr& context,
    rmf_traffic::agv::Plan::Start pickup_start,
    rmf_traffic::agv::Plan::Start dropoff_start)
{
  const auto& graph = context->navigation_graph();

  const auto pickup_wp =
      graph.find_waypoint(request.pickup_place_name)->index();

  Task::PendingPhases phases;
  phases.push_back(
        phases::GoToPlace::make(context, std::move(pickup_start), pickup_wp));

  phases.push_back(
        std::make_unique<phases::DispenseItem::PendingPhase>(
          context,
          request.task_id,
          request.pickup_dispenser,
          context->itinerary().description().owner(),
          request.items));

  const auto dropoff_wp =
      graph.find_waypoint(request.dropoff_place_name)->index();

  phases.push_back(
        phases::GoToPlace::make(context, std::move(dropoff_start), dropoff_wp));

  std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> ingestor_items;
  ingestor_items.reserve(request.items.size());
  for(auto& dispenser_item : request.items){
    rmf_ingestor_msgs::msg::IngestorRequestItem item{};
    item.type_guid = dispenser_item.type_guid;
    item.quantity = dispenser_item.quantity;
    item.compartment_name = dispenser_item.compartment_name;
    ingestor_items.push_back(std::move(item));
  }

  phases.push_back(
        std::make_unique<phases::IngestItem::PendingPhase>(
          context,
          request.task_id,
          request.dropoff_ingestor,
          context->itinerary().description().owner(),
          ingestor_items));

  return Task::make(request.task_id, std::move(phases), context->worker());
}

} // namespace task
} // namespace rmf_fleet_adapter
