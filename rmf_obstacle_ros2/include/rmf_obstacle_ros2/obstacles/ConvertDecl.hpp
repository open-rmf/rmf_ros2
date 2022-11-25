// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMF_OBSTACLE_ROS2__OBSTACLES__CONVERTDECL_HPP
#define RMF_OBSTACLE_ROS2__OBSTACLES__CONVERTDECL_HPP

namespace rmf_obstacle_ros2 {
using Obstacle = rmf_obstacle_msgs::msg::Obstacle;

//==============================================================================
/// Serialize a SensorMsg into RMF Obstacle msg
template<typename SensorMsg>
void fill_obstacle_data(const SensorMsg& msg, Obstacle& obstacle);

//==============================================================================
/// Convert an RMF Obstacle msg into a SensorMsg type
template<typename SensorMsg>
SensorMsg convert(const Obstacle& msg);
}  // namespace ros_gz_bridge

#endif // #indef RMF_OBSTACLE_ROS2__OBSTACLES__CONVERTDECL_HPP
