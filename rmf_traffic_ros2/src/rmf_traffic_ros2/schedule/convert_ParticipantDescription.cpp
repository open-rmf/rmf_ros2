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

//==============================================================================
rmf_traffic::schedule::ParticipantDescriptionsMap convert(
  const rmf_traffic_msgs::msg::Participants& from)
{
  rmf_traffic::schedule::ParticipantDescriptionsMap output;
  for (const auto& participant: from.participants)
  {
    std::pair<
      rmf_traffic::schedule::ParticipantId,
      rmf_traffic::schedule::ParticipantDescription> to_insert{
      participant.id,
      convert(participant.description)};
    output.insert(to_insert);
  }
  return output;
}

//==============================================================================
rmf_traffic_msgs::msg::Participants convert(
  const rmf_traffic::schedule::ParticipantDescriptionsMap& from)
{
  rmf_traffic_msgs::msg::Participants output;
  for (const auto& [id, description]: from)
  {
    rmf_traffic_msgs::msg::Participant participant;
    participant.id = id;
    participant.description = convert(description);
    output.participants.push_back(participant);
  }
  return output;
}

} // namespace rmf_traffic_ros2
