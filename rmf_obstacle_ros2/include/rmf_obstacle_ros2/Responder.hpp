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

#ifndef RMF_OBSTACLE_ROS2__RESPONDER_HPP
#define RMF_OBSTACLE_ROS2__RESPONDER_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>

namespace rmf_obstacle_ros2 {

//==============================================================================
/// Pure abstract class that reacts to obstacles detected. This should be
/// implemented as a plugin using pluginlib.
class Responder
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;

  virtual void initialize(const rclcpp::Node& node) = 0;

  virtual std::string name() const = 0;

  virtual void respond(const Obstacles& obstacles) const = 0;

  virtual ~Responder() = default;

};

using ResponderPtr = std::shared_ptr<Responder>;
using ConstResponderPtr = std::shared_ptr<const Responder>;

} // namespace rmf_obstacle_ros2

#endif // RMF_OBSTACLE_ROS2__RESPONDER_HPP
