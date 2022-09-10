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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__SCHEDULEIDENTITY_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__SCHEDULEIDENTITY_HPP

#include <rmf_traffic_msgs/msg/schedule_identity.hpp>
#include <optional>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
/// Compare a previous schedule startup message to an incoming one. If a
/// reconnection is appropriate, then the previous argument will be filled with
/// the data of the new incoming message and the function will return true. If
/// no reconnection should happen then the previous argument will not be
/// modified and the function will return false.
bool reconnect_schedule(
  rmf_traffic_msgs::msg::ScheduleIdentity& previous,
  const rmf_traffic_msgs::msg::ScheduleIdentity& incoming);

/// Same as its overload, but it accepts an optional for previous. When previous
/// is nullopt, this will save incoming and return true.
bool reconnect_schedule(
  std::optional<rmf_traffic_msgs::msg::ScheduleIdentity>& previous,
  const rmf_traffic_msgs::msg::ScheduleIdentity& incoming);

//==============================================================================
/// Equivalent to reconnect_schedule, but it does not modify the previous
/// argument.
bool need_reconnection(
  const rmf_traffic_msgs::msg::ScheduleIdentity& previous,
  const rmf_traffic_msgs::msg::ScheduleIdentity& incoming);

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__SCHEDULESTARTUP_HPP
