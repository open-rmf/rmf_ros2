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

#ifndef RMF_OBSTACLE_ROS2__OBSTACLES__CONVERT__CONVERTPOINTCLOUDOBSTACLES_HPP
#define RMF_OBSTACLE_ROS2__OBSTACLES__CONVERT__CONVERTPOINTCLOUDOBSTACLES_HPP

#include <rmf_obstacle_msgs/msg/obstacle.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rmf_obstacle_ros2/obstacles/ConvertDecl.hpp"

namespace rmf_obstacle_ros2 {

using PointCloud = sensor_msgs::msg::PointCloud2;

//==============================================================================
/// Serialize a PointCloud2 msg into RMF Obstacle msg
template<>
void fill_obstacle_data(const PointCloud& msg, Obstacle& obstacle);

//==============================================================================
/// Convert an RMF Obstacle msg into a PointCloud2 msg
template<>
PointCloud convert(const Obstacle& msg);
} // namespace rmf_obstacle_ros2

#endif // #indef RMF_OBSTACLE_ROS2__OBSTACLES__CONVERT__CONVERTPOINTCLOUDOBSTACLES_HPP
