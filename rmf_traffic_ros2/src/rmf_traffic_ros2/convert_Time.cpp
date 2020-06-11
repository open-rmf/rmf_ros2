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

#include <rmf_traffic_ros2/Time.hpp>

#include <cassert>

namespace rmf_traffic_ros2 {

//==============================================================================
builtin_interfaces::msg::Time convert(rmf_traffic::Time time)
{
  return rclcpp::Time(time.time_since_epoch().count(), RCL_ROS_TIME);
}

//==============================================================================
rmf_traffic::Time convert(builtin_interfaces::msg::Time time)
{
  return std::chrono::steady_clock::time_point(
    std::chrono::seconds(time.sec)
    + std::chrono::nanoseconds(time.nanosec));
}

//==============================================================================
rclcpp::Time to_ros2(rmf_traffic::Time from)
{
  return rclcpp::Time(from.time_since_epoch().count(), RCL_ROS_TIME);
}

//==============================================================================
rmf_traffic::Time convert(rclcpp::Time from)
{
  return std::chrono::steady_clock::time_point(
    std::chrono::nanoseconds(from.nanoseconds()));
}

//==============================================================================
rclcpp::Duration convert(rmf_traffic::Duration duration)
{
  return rclcpp::Duration{duration};
}

//==============================================================================
rmf_traffic::Duration convert(rclcpp::Duration duration)
{
  return duration.to_chrono<rmf_traffic::Duration>();
}

} // namespace rmf_traffic_ros2
