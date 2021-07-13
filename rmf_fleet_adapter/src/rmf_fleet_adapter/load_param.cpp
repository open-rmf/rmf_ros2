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
rmf_traffic::agv::VehicleTraits get_traits_or_default(rclcpp::Node& node,
  const double default_v_nom, const double default_w_nom,
  const double default_a_nom, const double default_alpha_nom,
  const double default_r_f, const double default_r_v)
{
  const double v_nom =
    get_parameter_or_default(node, "linear_velocity", default_v_nom);
  const double w_nom =
    get_parameter_or_default(node, "angular_velocity", default_w_nom);
  const double a_nom =
    get_parameter_or_default(node, "linear_acceleration", default_a_nom);
  const double b_nom =
    get_parameter_or_default(node, "angular_acceleration", default_alpha_nom);
  const double r_f =
    get_parameter_or_default(node, "footprint_radius", default_r_f);
  const double r_v =
    get_parameter_or_default(node, "vicinity_radius", default_r_v);
  const bool reversible =
    get_parameter_or_default(node, "reversible", true);

  if (!reversible)
    std::cout << " ===== We have an irreversible robot" << std::endl;

  auto traits = rmf_traffic::agv::VehicleTraits{
    {v_nom, a_nom},
    {w_nom, b_nom},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(r_f),
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(r_v)
    }
  };

  traits.get_differential()->set_reversible(reversible);
  return traits;
}


//==============================================================================
std::optional<rmf_battery::agv::BatterySystem> get_battery_system(
  rclcpp::Node& node,
  const double default_voltage,
  const double default_capacity,
  const double default_charging_current)
{
  const double voltage =
    get_parameter_or_default(node, "battery_voltage", default_voltage);
  const double capacity =
    get_parameter_or_default(node, "battery_capacity", default_capacity);
  const double charging_current =
    get_parameter_or_default(
    node, "battery_charging_current", default_charging_current);

  auto battery_system = rmf_battery::agv::BatterySystem::make(
    voltage, capacity, charging_current);

  return battery_system;
}

//==============================================================================
std::optional<rmf_battery::agv::MechanicalSystem> get_mechanical_system(
  rclcpp::Node& node,
  const double default_mass,
  const double default_moment,
  const double default_friction)
{
  const double mass =
    get_parameter_or_default(node, "mass", default_mass);
  const double moment_of_inertia =
    get_parameter_or_default(node, "inertia", default_moment);
  const double friction =
    get_parameter_or_default(node, "friction_coefficient", default_friction);

  auto mechanical_system = rmf_battery::agv::MechanicalSystem::make(
    mass, moment_of_inertia, friction);

  return mechanical_system;
}

} // namespace rmf_fleet_adapter
