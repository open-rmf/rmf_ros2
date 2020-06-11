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

#ifndef RMF_TRAFFIC_ROS2__PROFILE_HPP
#define RMF_TRAFFIC_ROS2__PROFILE_HPP

#include <rmf_traffic_msgs/msg/profile.hpp>

#include <rmf_traffic/Profile.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::Profile convert(const rmf_traffic_msgs::msg::Profile& from);

//==============================================================================
rmf_traffic_msgs::msg::Profile convert(const rmf_traffic::Profile& from);

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__PROFILE_HPP
