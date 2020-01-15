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

#include "load_param.hpp"

#include <rmf_traffic/geometry/Circle.hpp>

namespace rmf_fleet_adapter {

//==============================================================================
std::chrono::nanoseconds get_parameter_or_default_time(
    rclcpp::Node& node,
    const std::string& param_name,
    const double default_value)
{
  const double value =
      get_parameter_or_default(node, param_name, default_value);

  return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::ratio<1>>(value));
}

//==============================================================================
std::string get_fleet_name_parameter(rclcpp::Node& node)
{
  std::string fleet_name = node.declare_parameter("fleet_name", std::string());
  if (fleet_name.empty())
  {
    RCLCPP_ERROR(
          node.get_logger(),
          "The fleet_name parameter must be specified!");
    throw std::runtime_error("fleet_name parameter is missing");
  }

  return fleet_name;
}

//==============================================================================
rmf_traffic::agv::VehicleTraits get_traits_or_default(
    rclcpp::Node& node,
    const double default_v_nom, const double default_w_nom,
    const double default_a_nom, const double default_alpha_nom,
    const double default_radius)
{
  const double v_nom =
      get_parameter_or_default(node, "linear_velocity", default_v_nom);
  const double w_nom =
      get_parameter_or_default(node, "angular_velocity", default_w_nom);
  const double a_nom =
      get_parameter_or_default(node, "linear_acceleration", default_a_nom);
  const double b_nom =
      get_parameter_or_default(node, "angular_acceleration", default_alpha_nom);
  const double r =
      get_parameter_or_default(node, "profile_radius", default_radius);
  const bool reversible =
      get_parameter_or_default(node, "reversible", true);

  if (!reversible)
    std::cout << " ===== We have an irreversible robot" << std::endl;

  auto traits = rmf_traffic::agv::VehicleTraits{
    {v_nom, a_nom},
    {w_nom, b_nom},
    rmf_traffic::Trajectory::Profile::make_guided(
          rmf_traffic::geometry::make_final_convex<
            rmf_traffic::geometry::Circle>(r))
  };

  traits.get_differential()->set_reversible(reversible);
  return traits;
}

} // namespace rmf_fleet_adapter
