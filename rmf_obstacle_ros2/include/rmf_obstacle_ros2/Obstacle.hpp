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

#ifndef RMF_OBSTACLE_ROS2_OBSTACLE_HPP
#define RMF_OBSTACLE_ROS2_OBSTACLE_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_obstacle_msgs/msg/obstacle_data.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <optional>

namespace rmf_obstacle_ros2 {

using Header = std_msgs::msg::Header;
using PointCloud = sensor_msgs::msg::PointCloud2;
using ObstacleData = rmf_obstacle_msgs::msg::ObstacleData;

// TODO(YV): Consider defining a pure abstract class to perform
// serialization/deserialization. The abstract class could also have a function
// to generate MarkerArrays for rviz visualization.
// Provide a default implementation.
//==============================================================================
/// Serialize a PointCloud2 msg into RMF obstacle octree
static ObstacleData convert(const PointCloud& msg);

//==============================================================================
/// Deserialize an RMF obstacle octree into a PointCloud2 msg
static PointCloud convert(
  const Header& header,
  const ObstacleData& msg);


} // namespace rmf_obstacle_ros2

#endif // #indef RMF_OBSTACLE_ROS2_OBSTACLE_HPP
