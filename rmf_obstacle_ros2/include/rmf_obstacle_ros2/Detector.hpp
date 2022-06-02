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

#ifndef RMF_OBSTACLE_ROS2__DETECTOR_HPP
#define RMF_OBSTACLE_ROS2__DETECTOR_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>

#include <optional>

namespace rmf_obstacle_ros2 {

//==============================================================================
/// Pure abstract class for detecting and reporting obstacles
/// This class should be implemented as a plugin using pluginlib
class Detector
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;
  using DetectorCallback = std::function<void(const Obstacles& obstacles)>;


  /// param[in] node
  ///   A reference to rclcpp::Node
  ///
  /// param[in] cb
  ///   The callback that should be triggered when this detector detects any
  ///   obstacles. If nullptr, the detector will not do anything.
  virtual void initialize(
    const rclcpp::Node& node,
    DetectorCallback cb) = 0;

  /// Get the name of this detector
  virtual std::string name() const = 0;

  virtual ~Detector() = default;

};

using DetectorPtr = std::shared_ptr<Detector>;
using ConstDetectorPtr = std::shared_ptr<const Detector>;

} // namespace rmf_obstacle_ros2

#endif // RMF_OBSTACLE_ROS2__DETECTOR_HPP
