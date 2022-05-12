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


#include <rmf_obstacle_ros2/ObstacleManager.hpp>
#include <rmf_obstacle_ros2/StandardNames.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>

namespace rmf_obstacle_ros2 {

//==============================================================================
class ObstacleManager::Implementation
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;
  Implementation(
    DetectorPtr detector_,
    ResponderPtr responder_,
    std::weak_ptr<rclcpp::Node> node_)
  {
    detector = detector_;
    responder = responder_;
    node = node_;


    auto n = node.lock();
    if (!detector)
      return;

    detector->initialize(*n);
    if (responder)
      responder->initialize(*n);

    double rate = n->declare_parameter("rate", 1.0);
    const auto timer_rate =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(rate));


    detection_pub = n->create_publisher<Obstacles>(
      ObstaclesTopicName,
      rclcpp::QoS(10));

    detection_timer = n->create_wall_timer(
      timer_rate,
      [detector = detector, responder = responder, node = node, pub = detection_pub]()
      {
        const auto obstacles_opt = detector->obstacles();
        if (!obstacles_opt.has_value())
        {
          if (auto n = node.lock())
          {
            RCLCPP_ERROR(
              n->get_logger(),
              "No obstacles detected by detector %s",
              detector->name().c_str());
          }
        }

        const auto& obstacles = obstacles_opt.value();
        if (auto n = node.lock())
        {
            RCLCPP_ERROR(
              n->get_logger(),
              "Detector %s detected %ld obstacles",
              detector->name().c_str(), obstacles.obstacles.size());
        }
        // Publish obstacles
        pub->publish(obstacles);
        if (responder)
          responder->respond(obstacles);
      });
  }

  DetectorPtr detector;
  ResponderPtr responder;
  std::weak_ptr<rclcpp::Node> node;
  rclcpp::TimerBase::SharedPtr detection_timer;
  rclcpp::Publisher<Obstacles>::SharedPtr detection_pub;
};

//==============================================================================
ObstacleManager::ObstacleManager(
  DetectorPtr detector,
  ResponderPtr responder,
  const rclcpp::NodeOptions& options)
: Node("obstacle_manager", options),
  _pimpl(rmf_utils::make_unique_impl<Implementation>(
      std::move(detector),
      std::move(responder),
      this->ObstacleManager::weak_from_this()))
{
  // Do nothing
}

} // namespace rmf_obstacle_ros2
