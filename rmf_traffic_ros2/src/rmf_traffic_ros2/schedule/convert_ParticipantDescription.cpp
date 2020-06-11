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

#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/Profile.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::ParticipantDescription convert(
  const rmf_traffic_msgs::msg::ParticipantDescription& from)
{
  return rmf_traffic::schedule::ParticipantDescription{
    from.name,
    from.owner,
    static_cast<rmf_traffic::schedule::ParticipantDescription::Rx>(
      from.responsiveness),
    convert(from.profile)
  };
}

//==============================================================================
rmf_traffic_msgs::msg::ParticipantDescription convert(
  const rmf_traffic::schedule::ParticipantDescription& from)
{
  rmf_traffic_msgs::msg::ParticipantDescription output;
  output.name = from.name();
  output.owner = from.owner();
  output.responsiveness = static_cast<uint8_t>(from.responsiveness());
  output.profile = convert(from.profile());

  return output;
}

} // namespace rmf_traffic_ros2
