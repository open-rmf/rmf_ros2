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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__PATCH_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__PATCH_HPP

#include <rmf_traffic/schedule/Patch.hpp>

#include <rmf_traffic_msgs/msg/schedule_patch.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic_msgs::msg::ScheduleParticipantPatch convert(
  const rmf_traffic::schedule::Patch::Participant& from);

//==============================================================================
rmf_traffic::schedule::Patch::Participant convert(
  const rmf_traffic_msgs::msg::ScheduleParticipantPatch& from);

//==============================================================================
rmf_traffic_msgs::msg::SchedulePatch convert(
  const rmf_traffic::schedule::Patch& from);

//==============================================================================
rmf_traffic::schedule::Patch convert(
  const rmf_traffic_msgs::msg::SchedulePatch& from);

} // nmaespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__PATCH_HPP
