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

#ifndef RMF_TRAFFIC_ROS2__TRAJECTORY_HPP
#define RMF_TRAFFIC_ROS2__TRAJECTORY_HPP

#include <rmf_traffic_msgs/msg/trajectory.hpp>

#include <rmf_traffic/Trajectory.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
/// Convert from a Trajectory message to a Trajectory instance.
///
/// If the Trajectory is malformed, this will throw a std::runtime_error
/// describing the issue.
// TODO(MXG): Consider making conversion functions that do not require any
// allocation.
rmf_traffic::Trajectory convert(const rmf_traffic_msgs::msg::Trajectory& from);

//==============================================================================
/// Convert from a Trajectory instance to a Trajectory message.
rmf_traffic_msgs::msg::Trajectory convert(const rmf_traffic::Trajectory& from);

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__TRAJECTORY_HPP
