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

#ifndef RMF_OBSTACLE_ROS2__OBSTACLEMANAGER_HPP
#define RMF_OBSTACLE_ROS2__OBSTACLEMANAGER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_obstacle_ros2/Responder.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_obstacle_ros2 {

//==============================================================================
class ObstacleManager
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;

  /// Make an ObstacleManager instance.
  ///
  /// \note You must initialize rclcpp before calling this, either by using
  /// rclcpp::init(~) or rclcpp::Context::init(~).
  ///
  /// \param[in] node_name
  ///   The name of the rclcpp::Node for this ObstacleManager.
  ///
  /// \param[in] responder
  ///   An optional responder implementation to directly respond to detections
  ///
  /// \param[in] node_options
  ///   The options that the rclcpp::Node will be constructed with.
  static std::shared_ptr<ObstacleManager> make(
    const std::string& name,
    ConstResponderPtr responder = nullptr,
    const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  /// Call this function when you have some detections.
  /// This function will publish the detection and optionally respond to them.
  void process(const Obstacles& detections);

  /// Get the rclcpp::Node that this adapter will be using for communication.
  std::shared_ptr<rclcpp::Node> node();

  /// const-qualified node()
  std::shared_ptr<const rclcpp::Node> node() const;

  /// Begin running the event loop for this manager. The event loop will run
  /// in another thread, so this function is non-blocking.
  ObstacleManager& start();

  /// Wait until the adapter is done spinning.
  ObstacleManager& wait();

  ~ObstacleManager();

  class Implementation;

private:
  ObstacleManager();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_obstacle_ros2

#endif // RMF_OBSTACLE_ROS2__OBSTACLEMANAGER_HPP
