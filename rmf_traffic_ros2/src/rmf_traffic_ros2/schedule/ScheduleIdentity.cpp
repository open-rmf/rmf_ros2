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

#include <rmf_traffic_ros2/schedule/ScheduleIdentity.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
bool reconnect_schedule(
  rmf_traffic_msgs::msg::ScheduleIdentity& previous,
  const rmf_traffic_msgs::msg::ScheduleIdentity& incoming)
{
  const bool need_reconnect = need_reconnection(previous, incoming);
  if (need_reconnect || previous.node_uuid == incoming.node_uuid)
  {
    // Reconnection is only needed if the incoming UUID is different from the
    // one we already had. If it isn't different then we just update the
    // timestamp that's been saved.
    previous = incoming;
  }

  return need_reconnect;
}

//==============================================================================
bool reconnect_schedule(
  std::optional<rmf_traffic_msgs::msg::ScheduleIdentity>& previous,
  const rmf_traffic_msgs::msg::ScheduleIdentity& incoming)
{
  if (!previous.has_value())
  {
    previous = incoming;
    return true;
  }

  return reconnect_schedule(*previous, incoming);
}

//==============================================================================
bool need_reconnection(
  const rmf_traffic_msgs::msg::ScheduleIdentity& previous,
  const rmf_traffic_msgs::msg::ScheduleIdentity& incoming)
{
  bool incoming_is_newer = false;
  if (previous.timestamp.sec < incoming.timestamp.sec)
  {
    incoming_is_newer = true;
  }
  else if (previous.timestamp.sec == incoming.timestamp.sec)
  {
    if (previous.timestamp.nanosec < incoming.timestamp.nanosec)
      incoming_is_newer = true;
  }

  if (incoming_is_newer)
  {
    return previous.node_uuid != incoming.node_uuid;
  }

  return false;
}


} // namespace schedule
} // namespace rmf_traffic_ros2
