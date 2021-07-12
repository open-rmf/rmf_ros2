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

#include <rmf_traffic_ros2/schedule/Patch.hpp>
#include <rmf_traffic_ros2/schedule/Change.hpp>

#include <rmf_traffic_msgs/msg/schedule_participant_patch.hpp>
#include <rmf_traffic_msgs/msg/schedule_change_cull.hpp>
#include <rmf_traffic_msgs/msg/schedule_change_add.hpp>
#include <rmf_traffic_msgs/msg/schedule_change_delay.hpp>

using Time = rmf_traffic::Time;
using Duration = rmf_traffic::Duration;

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::Patch::Participant convert(
  const rmf_traffic_msgs::msg::ScheduleParticipantPatch& from);

//==============================================================================

//==============================================================================
template<typename T_out, typename T_in>
void convert_vector(
  std::vector<T_out>& output,
  const std::vector<T_in>& input)
{
  output.reserve(input.size());
  for (const auto& i : input)
    output.emplace_back(convert(i));
}

//==============================================================================
template<typename T_out, typename T_in>
std::vector<T_out> convert_vector(
  const std::vector<T_in>& input)
{
  std::vector<T_out> output;
  convert_vector(output, input);
  return output;
}

//==============================================================================
rmf_traffic_msgs::msg::ScheduleParticipantPatch convert(
  const rmf_traffic::schedule::Patch::Participant& from)
{
  return
    rmf_traffic_msgs::build<rmf_traffic_msgs::msg::ScheduleParticipantPatch>()
    .participant_id(from.participant_id())
    .itinerary_version(from.itinerary_version())
    .erasures(from.erasures().ids())
    .delays(convert_vector<rmf_traffic_msgs::msg::ScheduleChangeDelay>(
        from.delays()))
    .additions(convert_vector<rmf_traffic_msgs::msg::ScheduleChangeAdd>(
        from.additions().items()));
}

//==============================================================================
rmf_traffic::schedule::Patch::Participant convert(
  const rmf_traffic_msgs::msg::ScheduleParticipantPatch& from)
{
  return rmf_traffic::schedule::Patch::Participant{
    from.participant_id,
    from.itinerary_version,
    rmf_traffic::schedule::Change::Erase{from.erasures},
    convert_vector<rmf_traffic::schedule::Change::Delay>(from.delays),
    rmf_traffic::schedule::Change::Add{
      convert_vector<rmf_traffic::schedule::Change::Add::Item>(from.additions)
    }
  };
}

//==============================================================================
rmf_traffic_msgs::msg::SchedulePatch convert(
  const rmf_traffic::schedule::Patch& from)
{
  rmf_traffic_msgs::msg::SchedulePatch output;

  output.participants.reserve(from.size());
  for (const auto& p : from)
    output.participants.emplace_back(convert(p));

  if (const auto& cull = from.cull())
    output.cull.emplace_back(convert(*cull));

  output.has_base_version = from.base_version().has_value();
  if (from.base_version().has_value())
    output.base_version = *from.base_version();

  output.latest_version = from.latest_version();

  return output;
}

//==============================================================================
rmf_traffic::schedule::Patch convert(
  const rmf_traffic_msgs::msg::SchedulePatch& from)
{
  std::optional<rmf_traffic::schedule::Change::Cull> cull;
  if (!from.cull.empty())
    cull = convert(from.cull.front());

  std::optional<rmf_traffic::schedule::Version> base_version;
  if (from.has_base_version)
    base_version = from.base_version;

  return rmf_traffic::schedule::Patch{
    convert_vector<rmf_traffic::schedule::Patch::Participant>(
      from.participants),
    std::move(cull),
    base_version,
    from.latest_version
  };
}

} // namespace rmf_traffic_ros2
