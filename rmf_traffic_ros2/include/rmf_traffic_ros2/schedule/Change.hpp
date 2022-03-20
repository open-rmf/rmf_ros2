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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__CHANGE_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__CHANGE_HPP

#include <rmf_traffic/schedule/Change.hpp>

#include <rmf_traffic_msgs/msg/schedule_change_add.hpp>
#include <rmf_traffic_msgs/msg/schedule_change_delay.hpp>
#include <rmf_traffic_msgs/msg/schedule_change_cull.hpp>
#include <rmf_traffic_msgs/msg/schedule_change_add_item.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::Change::Add::Item convert(
  const rmf_traffic_msgs::msg::ScheduleChangeAddItem& from);

//==============================================================================
rmf_traffic_msgs::msg::ScheduleChangeAddItem convert(
  const rmf_traffic::schedule::Change::Add::Item& from);

//==============================================================================
rmf_traffic::schedule::Change::Add convert(
  const rmf_traffic_msgs::msg::ScheduleChangeAdd& from);

//==============================================================================
rmf_traffic_msgs::msg::ScheduleChangeAdd convert(
  const rmf_traffic::schedule::Change::Add& from);

//==============================================================================
rmf_traffic::schedule::Change::Delay convert(
  const rmf_traffic_msgs::msg::ScheduleChangeDelay& from);

//==============================================================================
rmf_traffic_msgs::msg::ScheduleChangeDelay convert(
  const rmf_traffic::schedule::Change::Delay& from);

//==============================================================================
rmf_traffic::schedule::ParticipantId convert(
  const rmf_traffic::schedule::Change::UnregisterParticipant& from);

//==============================================================================
rmf_traffic::schedule::Change::Cull convert(
  const rmf_traffic_msgs::msg::ScheduleChangeCull& from);

//==============================================================================
rmf_traffic_msgs::msg::ScheduleChangeCull convert(
  const rmf_traffic::schedule::Change::Cull& from);

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__CHANGE_HPP
