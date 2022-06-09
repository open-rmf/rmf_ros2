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


#include "ObstacleManager.hpp"
#include <rmf_obstacle_ros2/StandardNames.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>

#include <rclcpp_components/register_node_macro.hpp>


namespace rmf_obstacle_ros2 {

//==============================================================================
ObstacleManager::ObstacleManager(
  const rclcpp::NodeOptions& options)
: Node("obstacle_manager", options),
  _detector_loader("rmf_obstacle_ros2", "rmf_obstacle_ros2::Detector"),
  _responder_loader("rmf_obstacle_ros2", "rmf_obstacle_ros2::Responder")
{

  _responder = nullptr;

  RCLCPP_INFO(
    this->get_logger(),
    "Setting up ObstacleManager...");

  // Parameters to receive the fully qualified plugin strings
  const std::string detector_plugin = this->declare_parameter(
    "detector_plugin", "");

  const std::string responder_plugin = this->declare_parameter(
    "responder_plugin", "");

  _detection_pub = this->create_publisher<Obstacles>(
    ObstacleTopicName,
    rclcpp::QoS(10));

  // Initialize the responder
  try
  {
    _responder = _responder_loader.createSharedInstance(responder_plugin);
  }
  catch (pluginlib::PluginlibException& e)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to load responder plugin provided via the responder_plugin "
      "ROS 2  parameter. Please ensure the fully qualified name of the "
      "plugin is provided. The ObstacleManager will not respond to any "
      "obstacles detected. Detailed error: %s",
      e.what());
  }
  if (_responder != nullptr)
    _responder->initialize(*this);

  // Initialize the detector
  try
  {
    _detector = _detector_loader.createSharedInstance(detector_plugin);
  }
  catch (pluginlib::PluginlibException& e)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load detector plugin provided via the detector_plugin ROS 2 "
      "parameter. Please ensure the fully qualified name of the plugin is "
      "provided. Detailed error: %s",
      e.what());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Successfully loaded detector_plugin: %s",
    detector_plugin.c_str());


  _detector->initialize(*this,
    [pub=_detection_pub, responder = _responder](
      const Responder::Obstacles& obstacles)
    {
      pub->publish(obstacles);
      if (responder)
      {
        responder->respond(obstacles);
      }
    });

}

} // namespace rmf_obstacle_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(rmf_obstacle_ros2::ObstacleManager)
