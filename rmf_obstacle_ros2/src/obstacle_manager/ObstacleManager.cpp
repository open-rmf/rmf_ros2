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
#include <rmf_obstacle_ros2/Detector.hpp>
#include <rmf_obstacle_ros2/Responder.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <pluginlib/class_loader.hpp>

namespace rmf_obstacle_ros2 {

//==============================================================================
class ObstacleManager::Implementation
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;
  Implementation(
    std::shared_ptr<rclcpp::Node> node_)
  {
    responder = nullptr;

    RCLCPP_INFO(
      node_->get_logger(),
      "Setting up ObstacleManager...");
    // First argument is the package name of the tempalted base class.
    // Second argument is the fully qualified base class type
    pluginlib::ClassLoader<Detector> detector_loader(
      "rmf_obstacle_ros2", "rmf_obstacle_ros2::Detector");
    pluginlib::ClassLoader<Responder> responder_loader(
      "rmf_obstacle_ros2", "rmf_obstacle_ros2::Responder");

    // Parameters to receive the fully qualified plugin strings
    const std::string detector_plugin = node_->declare_parameter(
      "detector_plugin", "");

    const std::string responder_plugin = node_->declare_parameter(
      "responder_plugin", "");

    // Initialize the detector
    try
    {
      detector = detector_loader.createSharedInstance(detector_plugin);
    }
    catch(pluginlib::PluginlibException& e)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to load detector plugin provided via the detector_plugin ROS 2 "
        "parameter. Please ensure the fully qualified name of the plugin is "
        "provided. Detailed error: %s",
        e.what());
      return;
    }
    detector->initialize(*node_);

    // Initialize the detector
    try
    {
      responder = responder_loader.createSharedInstance(responder_plugin);
    }
    catch(pluginlib::PluginlibException& e)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Failed to load responder plugin provided via the responder_plugin "
        "ROS 2  parameter. Please ensure the fully qualified name of the "
        "plugin is provided. The ObstacleManager will not respond to any "
        "obstacles detected. Detailed error: %s",
        e.what());
    }
    if (responder)
      responder->initialize(*node_);

    double rate = node_->declare_parameter("rate", 1.0);
    const auto timer_rate =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(rate));

    detection_pub = node_->create_publisher<Obstacles>(
      ObstaclesTopicName,
      rclcpp::QoS(10));

    // TODO(YV): Bundle capture args into a data struct shared_ptr
    detection_timer = node_->create_wall_timer(
      timer_rate,
      [detector = detector, responder = responder, node = node, pub = detection_pub]()
      {
        const auto obstacles_opt = detector->obstacles();
        if (!obstacles_opt.has_value())
        {
          if (auto n = node.lock())
          {
            RCLCPP_INFO(
              n->get_logger(),
              "No obstacles detected by detector %s",
              detector->name().c_str());
          }
        }

        const auto& obstacles = obstacles_opt.value();
        if (auto n = node.lock())
        {
            RCLCPP_INFO(
              n->get_logger(),
              "Detector %s detected %ld obstacles",
              detector->name().c_str(), obstacles.obstacles.size());
        }
        // Publish obstacles
        pub->publish(obstacles);
        if (responder)
        {
          responder->respond(obstacles);
          if (auto n = node.lock())
          {
            RCLCPP_INFO(
              n->get_logger(),
              "Responder %s has responded to obstacles",
              responder->name().c_str());
          }
        }

      });

    node = node_;

  }

  DetectorPtr detector;
  ResponderPtr responder;
  std::weak_ptr<rclcpp::Node> node;
  rclcpp::TimerBase::SharedPtr detection_timer;
  rclcpp::Publisher<Obstacles>::SharedPtr detection_pub;
};

//==============================================================================
ObstacleManager::ObstacleManager(
  const rclcpp::NodeOptions& options)
: Node("obstacle_manager", options)
{
  _pimpl = rmf_utils::make_unique_impl<Implementation>(
      this->ObstacleManager::shared_from_this());
}

} // namespace rmf_obstacle_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(rmf_obstacle_ros2::ObstacleManager)
