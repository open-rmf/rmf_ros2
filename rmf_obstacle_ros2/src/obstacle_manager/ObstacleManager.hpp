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

#ifndef SRC__RMF_OBSTACLE_ROS2__OBSTACLEMANAGER_HPP
#define SRC__RMF_OBSTACLE_ROS2__OBSTACLEMANAGER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_obstacle_ros2/Detector.hpp>
#include <rmf_obstacle_ros2/Responder.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>

#include <pluginlib/class_loader.hpp>

namespace rmf_obstacle_ros2 {

//==============================================================================
/// Node to detect and publish obstacles to RMF
class ObstacleManager : public rclcpp::Node
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;
  /// The implementation expects users to pass the fully qualified plugin paths
  /// for the detector and responder via ROS 2 parameters. The parameter names
  /// are "detector_plugin" and "responder_plugin" respectively.
  ObstacleManager(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  class Implementation;

private:
  pluginlib::ClassLoader<Detector> _detector_loader;
  pluginlib::ClassLoader<Responder> _responder_loader;
  DetectorPtr _detector;
  ResponderPtr _responder;
  rclcpp::Publisher<Obstacles>::SharedPtr _detection_pub;
};

} // namespace rmf_obstacle_ros2

#endif // SRC__RMF_OBSTACLE_ROS2__OBSTACLEMANAGER_HPP
