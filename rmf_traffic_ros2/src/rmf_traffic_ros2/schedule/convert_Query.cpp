/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic_ros2/schedule/Query.hpp>

#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

namespace {
//==============================================================================
rmf_traffic::schedule::Query::Spacetime parse_regions(
    const rmf_traffic_msgs::msg::ScheduleQuerySpacetime& from)
{
  std::vector<rmf_traffic::geometry::Box> boxes;
  std::vector<rmf_traffic::geometry::Circle> circles;
}

//==============================================================================
rmf_traffic::schedule::Query::Spacetime parse_timespan(
    const rmf_traffic_msgs::msg::ScheduleQuerySpacetime& from)
{

}
} // anonymous namespace

//==============================================================================
rmf_traffic::schedule::Query::Spacetime convert(
    const rmf_traffic_msgs::msg::ScheduleQuerySpacetime& from)
{
  if(rmf_traffic_msgs::msg::ScheduleQuerySpacetime::ALL == from.type)
    return rmf_traffic::schedule::Query::Spacetime();
  else if(rmf_traffic_msgs::msg::ScheduleQuerySpacetime::REGIONS == from.type)
    return parse_regions(from);
  else if(rmf_traffic_msgs::msg::ScheduleQuerySpacetime::TIMESPAN == from.type)
    return parse_timespan(from);

  throw std::runtime_error(
        "Invalid rmf_traffic_msgs/ScheduleQuerySpacetime type ["
        + std::to_string(from.type) + "]");
}

} // namespace schedule
} // namespace rmf_traffic_ros2
