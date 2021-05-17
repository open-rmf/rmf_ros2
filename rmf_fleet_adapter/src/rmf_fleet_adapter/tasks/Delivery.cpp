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

#include <rmf_ingestor_msgs/msg/ingestor_request_item.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
std::shared_ptr<Task> make_delivery(
  const rmf_task::ConstRequestPtr request,
  const agv::RobotContextPtr& context,
  const rmf_traffic::agv::Plan::Start pickup_start,
  const rmf_traffic::Time deployment_time,
  const rmf_task::agv::State finish_state)
{

  std::shared_ptr<const rmf_task::requests::Delivery::Description> description =
    std::dynamic_pointer_cast<
    const rmf_task::requests::Delivery::Description>(request->description());

  if (description == nullptr)
    return nullptr;

  const auto pickup_waypoint = description->pickup_waypoint();
  const auto dropoff_waypoint = description->dropoff_waypoint();

  Task::PendingPhases phases;
  phases.push_back(
    phases::GoToPlace::make(
      context,
      std::move(pickup_start),
      pickup_waypoint));

  phases.push_back(
    std::make_unique<phases::DispenseItem::PendingPhase>(
      context,
      request->id(),
      description->pickup_dispenser(),
      context->itinerary().description().owner(),
      description->items()));

  const auto dropoff_start = [&]() -> rmf_traffic::agv::Planner::Start
    {

      if (pickup_start.waypoint() == pickup_waypoint)
        return pickup_start;

      rmf_traffic::agv::Planner::Goal goal{pickup_waypoint};

      const auto result = context->planner()->plan(pickup_start, goal);
      // We assume we can always compute a plan
      const auto& trajectory =
        result->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const double orientation = trajectory.back().position()[2];

      return rmf_traffic::agv::Planner::Start{
      finish_time,
      pickup_waypoint,
      orientation};
    } ();

  phases.push_back(
    phases::GoToPlace::make(
      context,
      std::move(dropoff_start),
      dropoff_waypoint));

  std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> ingestor_items;
  ingestor_items.reserve(description->items().size());
  for (const auto& dispenser_item : description->items())
  {
    rmf_ingestor_msgs::msg::IngestorRequestItem item{};
    item.type_guid = dispenser_item.type_guid;
    item.quantity = dispenser_item.quantity;
    item.compartment_name = dispenser_item.compartment_name;
    ingestor_items.push_back(std::move(item));
  }

  phases.push_back(
    std::make_unique<phases::IngestItem::PendingPhase>(
      context,
      request->id(),
      description->dropoff_ingestor(),
      context->itinerary().description().owner(),
      ingestor_items));

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
