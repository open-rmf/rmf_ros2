/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "project_itinerary.hpp"

namespace rmf_fleet_adapter {

//==============================================================================
rmf_traffic::schedule::Itinerary project_itinerary(
  const rmf_traffic::agv::Plan& starting_from,
  const std::vector<rmf_traffic::agv::Plan::Goal>& through_destinations,
  const rmf_traffic::agv::Planner& with_planner)
{
  auto itinerary = starting_from.get_itinerary();
  auto last_plan = starting_from;
  for (const auto& destination : through_destinations)
  {
    if (last_plan.get_waypoints().empty())
      break;

    const auto& wp = last_plan.get_waypoints().back();
    if (!wp.graph_index().has_value())
      break;

    rmf_traffic::agv::Plan::Start start(
      wp.time(), wp.graph_index().value(), wp.position()[2]);

    auto options = with_planner.get_default_options();
    options.validator(nullptr);
    auto result = with_planner.plan(start, destination, options);
    if (!result)
      break;

    last_plan = *std::move(result);
    const auto& new_itinerary = last_plan.get_itinerary();
    if (new_itinerary.front().map() == itinerary.back().map())
    {
      // We only look at the first route because we're not going to include
      // any map switches for now.
      for (const auto& wp : new_itinerary.front().trajectory())
        itinerary.back().trajectory().insert(wp);
    }
    else
    {
      // If the map has switched, let's break out of this loop.
      break;
    }
  }

  return itinerary;
}

} // namespace rmf_fleet_adapter
