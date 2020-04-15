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

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/Route.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::Writer::Input convert(
  const std::vector<rmf_traffic_msgs::msg::ScheduleWriterItem>& from)
{
  rmf_traffic::schedule::Writer::Input output;
  for (const auto& item : from)
  {
    output.emplace_back(
      rmf_traffic::schedule::Writer::Item{
        item.id,
        std::make_shared<rmf_traffic::Route>(convert(item.route))
      });
  }

  return output;
}

//==============================================================================
std::vector<rmf_traffic_msgs::msg::ScheduleWriterItem> convert(
  const rmf_traffic::schedule::Writer::Input& from)
{
  std::vector<rmf_traffic_msgs::msg::ScheduleWriterItem> output;
  for (const auto& item : from)
  {
    rmf_traffic_msgs::msg::ScheduleWriterItem msg;
    msg.id = item.id;
    assert(item.route);
    msg.route = convert(*item.route);
    output.push_back(std::move(msg));
  }

  return output;
}

} // namespace rmf_traffic_ros2
