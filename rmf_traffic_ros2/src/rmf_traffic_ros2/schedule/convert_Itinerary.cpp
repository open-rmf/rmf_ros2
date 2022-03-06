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

#include <rmf_traffic_ros2/schedule/Itinerary.hpp>
#include <rmf_traffic_ros2/Route.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
std::vector<rmf_traffic::schedule::Itinerary> convert(
  const std::vector<rmf_traffic_msgs::msg::Itinerary>& from)
{
  std::vector<rmf_traffic::schedule::Itinerary> output;
  output.reserve(from.size());
  for (const auto& from_itinerary : from)
  {
    rmf_traffic::schedule::Itinerary to_itinerary;
    to_itinerary.reserve(from_itinerary.routes.size());
    for (const auto& from_route : from_itinerary.routes)
      to_itinerary.emplace_back(convert(from_route));

    output.emplace_back(std::move(to_itinerary));
  }

  return output;
}

//==============================================================================
std::vector<rmf_traffic_msgs::msg::Itinerary> convert(
  const std::vector<rmf_traffic::schedule::Itinerary>& from)
{
  std::vector<rmf_traffic_msgs::msg::Itinerary> output;
  output.reserve(from.size());
  for (const auto& from_itinerary : from)
  {
    rmf_traffic_msgs::msg::Itinerary to_itinerary;
    to_itinerary.routes.reserve(from_itinerary.size());
    for (const auto& from_route : from_itinerary)
      to_itinerary.routes.emplace_back(convert(from_route));

    output.emplace_back(std::move(to_itinerary));
  }

  return output;
}

} // namespace rmf_traffic_ros2
