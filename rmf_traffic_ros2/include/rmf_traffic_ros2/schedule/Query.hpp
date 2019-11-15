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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__QUERY_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__QUERY_HPP

#include <rmf_traffic_msgs/msg/schedule_query_spacetime.hpp>

#include <rmf_traffic/schedule/Query.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::Query::Spacetime convert(
    const rmf_traffic_msgs::msg::ScheduleQuerySpacetime& from);

//==============================================================================
rmf_traffic_msgs::msg::ScheduleQuerySpacetime convert(
    const rmf_traffic::schedule::Query::Spacetime& from);

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__QUERY_HPP
