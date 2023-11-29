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

#ifndef SRC__RMF_FLEET_ADAPTER__PROJECT_ITINERARY_HPP
#define SRC__RMF_FLEET_ADAPTER__PROJECT_ITINERARY_HPP

#include <rmf_traffic/schedule/Itinerary.hpp>
#include <rmf_traffic/agv/Planner.hpp>

namespace rmf_fleet_adapter {

//==============================================================================
rmf_traffic::schedule::Itinerary project_itinerary(
  const rmf_traffic::agv::Plan& starting_from,
  const std::vector<rmf_traffic::agv::Plan::Goal>& through_destinations,
  const rmf_traffic::agv::Planner& with_planner);

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PROJECT_ITINERARY_HPP
