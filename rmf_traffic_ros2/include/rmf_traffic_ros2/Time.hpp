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

#ifndef RMF_TRAFFIC_ROS2__TIME_HPP
#define RMF_TRAFFIC_ROS2__TIME_HPP

#include <builtin_interfaces/msg/time.hpp>

#include <rmf_traffic/Time.hpp>
#include <rclcpp/time.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
builtin_interfaces::msg::Time convert(rmf_traffic::Time time);

//==============================================================================
rmf_traffic::Time convert(builtin_interfaces::msg::Time time);

//==============================================================================
rclcpp::Time to_ros2(rmf_traffic::Time time);

//==============================================================================
rmf_traffic::Time convert(rclcpp::Time time);

//==============================================================================
rclcpp::Duration convert(rmf_traffic::Duration duration);

//==============================================================================
rmf_traffic::Duration convert(rclcpp::Duration duration);

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__TIME_HPP
