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

  struct Data
  {
    DetectorPtr detector;
    ResponderPtr responder;
    std::weak_ptr<rclcpp::Node> node;
    rclcpp::TimerBase::SharedPtr detection_timer;
    rclcpp::Publisher<Obstacles>::SharedPtr detection_pub;
  };

  Implementation(
    std::shared_ptr<Data> data_)
  {
    data = std::move(data_);
  }

  std::shared_ptr<Data> data;
};

//==============================================================================
ObstacleManager::ObstacleManager(
  const rclcpp::NodeOptions& options)
: Node("obstacle_manager", options)
{

  auto data = std::make_shared<Implementation::Data>();
  data->responder = nullptr;

  RCLCPP_INFO(
    this->get_logger(),
    "Setting up ObstacleManager...");

  // First argument is the package name of the tempalted base class.
  // Second argument is the fully qualified base class type
  pluginlib::ClassLoader<Detector> detector_loader(
    "rmf_obstacle_ros2", "rmf_obstacle_ros2::Detector");
  pluginlib::ClassLoader<Responder> responder_loader(
    "rmf_obstacle_ros2", "rmf_obstacle_ros2::Responder");

  // Parameters to receive the fully qualified plugin strings
  const std::string detector_plugin = this->declare_parameter(
    "detector_plugin", "dummy_detector");

  const std::string responder_plugin = this->declare_parameter(
    "responder_plugin", "dummy_responder");


  data->node = this->ObstacleManager::shared_from_this();

  data->detection_pub = this->create_publisher<Implementation::Obstacles>(
    ObstaclesTopicName,
    rclcpp::QoS(10));

  // Initialize the responder
  try
  {
    data->responder = responder_loader.createSharedInstance(responder_plugin);
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
  if (data->responder)
    data->responder->initialize(*this);

  // Initialize the detector
  try
  {
    data->detector = detector_loader.createSharedInstance(detector_plugin);
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

  auto detection_cb =
    [data = data](const Responder::Obstacles& obstacles)
    {
      if (auto n = data->node.lock())
      {
        RCLCPP_INFO(
          n->get_logger(),
          "Detector %s detected %ld obstacles",
          data->detector->name().c_str(), obstacles.obstacles.size());
      }

      // Publish obstacles
      data->detection_pub->publish(obstacles);
      if (data->responder)
      {
        data->responder->respond(obstacles);
        if (auto n = data->node.lock())
        {
          RCLCPP_INFO(
            n->get_logger(),
            "Responder %s has responded to obstacles",
            data->responder->name().c_str());
        }
      }
    };


  data->detector->initialize(*this, detection_cb);


  _pimpl = rmf_utils::make_unique_impl<Implementation>(data);
}

} // namespace rmf_obstacle_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(rmf_obstacle_ros2::ObstacleManager)
