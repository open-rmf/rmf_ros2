/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__LOAD_PARAM_HPP
#define SRC__RMF_FLEET_ADAPTER__LOAD_PARAM_HPP

#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rclcpp/node.hpp>

#include <chrono>
#include <string>

namespace rmf_fleet_adapter {

//==============================================================================
template<typename T>
T get_parameter_or_default(
  rclcpp::Node& node,
  const std::string& param_name,
  const T& default_value)
{
  // TODO(MXG): Consider a way to automatically make a callback that will update
  // a target variable when the parameter value gets changed.

  const T value = node.declare_parameter(param_name, default_value);
  RCLCPP_INFO(
    node.get_logger(),
    "Parameter [" + param_name + "] set to: " + std::to_string(value));
  return value;
}

//==============================================================================
std::string get_fleet_name_parameter(rclcpp::Node& node);

//==============================================================================
std::chrono::nanoseconds get_parameter_or_default_time(
  rclcpp::Node& node,
  const std::string& param_name,
  const double default_value);

//==============================================================================
rmf_traffic::agv::VehicleTraits get_traits_or_default(
  rclcpp::Node& node,
  const double default_v_nom, const double default_w_nom,
  const double default_a_nom, const double default_alpha_nom,
  const double default_radius);

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__LOAD_PARAM_HPP
