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
  const rmf_traffic::schedule::Inconsistencies::Element& from,
  const rmf_traffic::schedule::ProgressVersion progress_version)
{
  std::vector<rmf_traffic_msgs::msg::ScheduleInconsistencyRange> ranges;
  for (const auto& r : from.ranges)
  {
    rmf_traffic_msgs::msg::ScheduleInconsistencyRange range;
    range.lower = r.lower;
    range.upper = r.upper;
    ranges.push_back(range);
  }

  return rmf_traffic_msgs::build<rmf_traffic_msgs::msg::ScheduleInconsistency>()
    .participant(from.participant)
    .ranges(std::move(ranges))
    .last_known_itinerary(from.ranges.last_known_version())
    .last_known_progress(progress_version);
}

} // namespace rmf_traffic_ros2
