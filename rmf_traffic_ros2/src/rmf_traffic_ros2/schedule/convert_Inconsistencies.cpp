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

#include <rmf_traffic_ros2/schedule/Inconsistencies.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic_msgs::msg::ScheduleInconsistency convert(
  const rmf_traffic::schedule::Inconsistencies::Element& from)
{
  rmf_traffic_msgs::msg::ScheduleInconsistency msg;
  msg.participant = from.participant;
  msg.last_known_version = from.ranges.last_known_version();

  for (const auto& r : from.ranges)
  {
    rmf_traffic_msgs::msg::ScheduleInconsistencyRange range;
    range.lower = r.lower;
    range.upper = r.upper;
    msg.ranges.push_back(range);
  }

  return msg;
}

} // namespace rmf_traffic_ros2
