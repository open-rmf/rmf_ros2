/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
// Required, not optional: the task/action consideration callbacks below take an
// nlohmann::json argument, and without this caster pybind11 falls back to
// resolving its internal `concat` against nlohmann's, which fails to compile
// with a wall of template errors rather than anything readable.
#include "pybind11_json/pybind11_json.hpp"

#include <rmf_fleet_adapter/agv/PathGuide.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>

#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ParkRobotFactory.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace py = pybind11;
namespace agv = rmf_fleet_adapter::agv;
namespace battery = rmf_battery::agv;

namespace {

/// Same shape as the helper in adapter.cpp: replaces the by-reference `confirm`
/// argument with a return value, which is much friendlier from Python.
using Confirmation = agv::FleetUpdateHandle::Confirmation;
using ConsiderRequest = agv::FleetUpdateHandle::ConsiderRequest;
using ModifiedConsiderRequest =
  std::function<Confirmation(const nlohmann::json& description)>;

std::unordered_map<std::string, ConsiderRequest> convert_consideration(
  const std::unordered_map<std::string, ModifiedConsiderRequest>& consideration)
{
  std::unordered_map<std::string, ConsiderRequest> output;
  for (const auto& element : consideration)
  {
    output[element.first] = [consider = element.second](
      const nlohmann::json& description, Confirmation& confirm)
      {
        confirm = consider(description);
      };
  }
  return output;
}

rmf_task::ConstRequestFactoryPtr parse_finishing_request(
  const std::string& finishing_request_string)
{
  if (finishing_request_string == "charge")
  {
    return std::make_shared<rmf_task::requests::ChargeBatteryFactory>();
  }

  if (finishing_request_string == "park")
  {
    return std::make_shared<rmf_task::requests::ParkRobotFactory>();
  }

  return nullptr;
}

} // anonymous namespace

void bind_path_guide(py::module& m)
{
  py::class_<agv::PathGuide, std::shared_ptr<agv::PathGuide>>(
    m, "PathGuide")
  .def("add_robot", &agv::PathGuide::add_robot)
  .def("more", [](agv::PathGuide& self)
    {
      return self.more();
    });

  auto m_path_guide = m.def_submodule("path_guide");

  // PathRobotUpdateHandle ====================================================
  py::class_<
    agv::PathGuide::PathRobotUpdateHandle,
    std::shared_ptr<agv::PathGuide::PathRobotUpdateHandle>
  >(m_path_guide, "PathRobotUpdateHandle")
  .def("update", &agv::PathGuide::PathRobotUpdateHandle::update)
  .def("max_merge_waypoint_distance",
    &agv::PathGuide::PathRobotUpdateHandle::max_merge_waypoint_distance)
  .def("set_max_merge_waypoint_distance",
    &agv::PathGuide::PathRobotUpdateHandle::set_max_merge_waypoint_distance)
  .def("max_merge_lane_distance",
    &agv::PathGuide::PathRobotUpdateHandle::max_merge_lane_distance)
  .def("set_max_merge_lane_distance",
    &agv::PathGuide::PathRobotUpdateHandle::set_max_merge_lane_distance)
  .def("min_lane_length",
    &agv::PathGuide::PathRobotUpdateHandle::min_lane_length)
  .def("set_min_lane_length",
    &agv::PathGuide::PathRobotUpdateHandle::set_min_lane_length)
  .def("more", [](agv::PathGuide::PathRobotUpdateHandle& self)
    {
      return self.more();
    });

  // RobotState ===============================================================
  py::class_<agv::PathGuide::RobotState>(m_path_guide, "RobotState")
  .def(py::init<
      const std::string&,
      Eigen::Vector3d,
      double>(),
    py::arg("map"),
    py::arg("position"),
    py::arg("battery_soc"))
  .def_property(
    "map",
    &agv::PathGuide::RobotState::map,
    &agv::PathGuide::RobotState::set_map)
  .def_property(
    "position",
    &agv::PathGuide::RobotState::position,
    &agv::PathGuide::RobotState::set_position)
  .def_property(
    "battery_state_of_charge",
    &agv::PathGuide::RobotState::battery_state_of_charge,
    &agv::PathGuide::RobotState::set_battery_state_of_charge);

  // RobotConfiguration =======================================================
  py::class_<agv::PathGuide::RobotConfiguration>(
    m_path_guide, "RobotConfiguration")
  .def(py::init<std::vector<std::string>>(),
    py::arg("compatible_chargers"))
  .def_property(
    "compatible_chargers",
    &agv::PathGuide::RobotConfiguration::compatible_chargers,
    &agv::PathGuide::RobotConfiguration::set_compatible_chargers)
  .def_property(
    "finishing_request",
    &agv::PathGuide::RobotConfiguration::finishing_request,
    &agv::PathGuide::RobotConfiguration::set_finishing_request);

  // RobotCallbacks ===========================================================
  py::class_<agv::PathGuide::RobotCallbacks>(m_path_guide, "RobotCallbacks")
  .def(py::init<
      agv::PathGuide::PathRequest,
      agv::PathGuide::StopRequest,
      agv::PathGuide::ActionExecutor>(),
    py::arg("follow_path"),
    py::arg("stop"),
    py::arg("action_executor"))
  .def_property_readonly(
    "follow_path",
    &agv::PathGuide::RobotCallbacks::follow_path)
  .def_property_readonly(
    "stop",
    &agv::PathGuide::RobotCallbacks::stop)
  .def_property_readonly(
    "action_executor",
    &agv::PathGuide::RobotCallbacks::action_executor)
  .def_property(
    "localize",
    &agv::PathGuide::RobotCallbacks::localize,
    &agv::PathGuide::RobotCallbacks::with_localization
  );

  // CommandExecution =========================================================
  py::class_<agv::PathGuide::CommandExecution>(
    m_path_guide, "CommandExecution")
  .def("finished", &agv::PathGuide::CommandExecution::finished)
  .def("okay", &agv::PathGuide::CommandExecution::okay)
  .def(
    "override_schedule",
    [](
      agv::PathGuide::CommandExecution& self,
      std::string map,
      std::vector<Eigen::Vector3d> path,
      double hold)
    {
      return self.override_schedule(
        std::move(map),
        std::move(path),
        rmf_traffic::time::from_seconds(hold));
    },
    py::arg("map"),
    py::arg("path"),
    py::arg("hold") = 0.0)
  .def_property_readonly("identifier",
    &agv::PathGuide::CommandExecution::identifier);

  // Destination ==============================================================
  py::class_<agv::PathGuide::Destination>(m_path_guide, "Destination")
  .def_property_readonly("map", &agv::PathGuide::Destination::map)
  .def_property_readonly("position", &agv::PathGuide::Destination::position)
  .def_property_readonly("xy", &agv::PathGuide::Destination::xy)
  .def_property_readonly("yaw", &agv::PathGuide::Destination::yaw)
  .def_property_readonly("graph_index",
    &agv::PathGuide::Destination::graph_index)
  .def_property_readonly("name", &agv::PathGuide::Destination::name)
  .def_property_readonly("dock", &agv::PathGuide::Destination::dock)
  .def_property_readonly("speed_limit",
    &agv::PathGuide::Destination::speed_limit)
  .def_property_readonly("inside_lift",
    &agv::PathGuide::Destination::inside_lift);

  // PathWaypoint =============================================================
  py::class_<agv::PathGuide::PathWaypoint>(m_path_guide, "PathWaypoint")
  .def_property_readonly("destination",
    &agv::PathGuide::PathWaypoint::destination)
  .def_property_readonly("merge_radius",
    &agv::PathGuide::PathWaypoint::merge_radius)
  .def_property_readonly("index", &agv::PathGuide::PathWaypoint::index)
  .def_property_readonly("execution",
    &agv::PathGuide::PathWaypoint::execution);

  // Path =====================================================================
  py::class_<agv::PathGuide::Path>(m_path_guide, "Path")
  .def_property_readonly("waypoints", &agv::PathGuide::Path::waypoints)
  .def_property_readonly("plan_id", &agv::PathGuide::Path::plan_id)
  .def_property_readonly("map", &agv::PathGuide::Path::map)
  .def("__len__", [](const agv::PathGuide::Path& self)
    {
      return self.waypoints().size();
    })
  .def("__getitem__",
    [](const agv::PathGuide::Path& self, std::size_t index)
    {
      if (index >= self.waypoints().size())
      {
        throw py::index_error();
      }
      return self.waypoints()[index];
    });

  // FleetConfiguration =======================================================
  // Mirrors the EasyFullControl binding, minus skip_rotation_commands, which
  // PathGuide does not have: it never issues a rotate-in-place command.
  py::class_<agv::PathGuide::FleetConfiguration>(
    m_path_guide, "FleetConfiguration")
  .def(py::init([]( // Lambda function to convert reference to shared ptr
        std::string& fleet_name,
        std::optional<std::unordered_map<std::string,
        agv::Transformation>> transformations_to_robot_coordinates,
        std::unordered_map<std::string,
        agv::PathGuide::RobotConfiguration> known_robot_configurations,
        rmf_traffic::agv::VehicleTraits& traits,
        rmf_traffic::agv::Graph& graph,
        battery::BatterySystem& battery_system,
        battery::SimpleMotionPowerSink& motion_sink,
        battery::SimpleDevicePowerSink& ambient_sink,
        battery::SimpleDevicePowerSink& tool_sink,
        double recharge_threshold,
        double recharge_soc,
        bool account_for_battery_drain,
        std::unordered_map<std::string,
        ModifiedConsiderRequest> task_consideration,
        std::unordered_map<std::string,
        ModifiedConsiderRequest> action_consideration,
        std::string& finishing_request_string,
        std::optional<std::string> server_uri,
        rmf_traffic::Duration max_delay,
        rmf_traffic::Duration update_interval,
        bool default_responsive_wait,
        double default_max_merge_waypoint_distance,
        double default_max_merge_lane_distance,
        double default_min_lane_length)
        {
          return agv::PathGuide::FleetConfiguration(
            fleet_name,
            std::move(transformations_to_robot_coordinates),
            std::move(known_robot_configurations),
            std::make_shared<rmf_traffic::agv::VehicleTraits>(traits),
            std::make_shared<rmf_traffic::agv::Graph>(graph),
            std::make_shared<battery::BatterySystem>(battery_system),
            std::make_shared<battery::SimpleMotionPowerSink>(motion_sink),
            std::make_shared<battery::SimpleDevicePowerSink>(ambient_sink),
            std::make_shared<battery::SimpleDevicePowerSink>(tool_sink),
            recharge_threshold,
            recharge_soc,
            account_for_battery_drain,
            convert_consideration(task_consideration),
            convert_consideration(action_consideration),
            parse_finishing_request(finishing_request_string),
            server_uri,
            max_delay,
            update_interval,
            default_responsive_wait,
            default_max_merge_waypoint_distance,
            default_max_merge_lane_distance,
            default_min_lane_length);
        }
        ),
    py::arg("fleet_name"),
    py::arg("transformations_to_robot_coordinates"),
    py::arg("known_robot_configurations"),
    py::arg("traits"),
    py::arg("graph"),
    py::arg("battery_system"),
    py::arg("motion_sink"),
    py::arg("ambient_sink"),
    py::arg("tool_sink"),
    py::arg("recharge_threshold"),
    py::arg("recharge_soc"),
    py::arg("account_for_battery_drain"),
    py::arg("task_categories"),
    py::arg("action_categories"),
    py::arg("finishing_request") = "nothing",
    py::arg("server_uri") = std::nullopt,
    py::arg("max_delay") = rmf_traffic::time::from_seconds(10.0),
    py::arg("update_interval") = rmf_traffic::time::from_seconds(0.5),
    py::arg("default_responsive_wait") = false,
    py::arg("default_max_merge_waypoint_distance") = 1e-3,
    py::arg("default_max_merge_lane_distance") = 0.3,
    py::arg("default_min_lane_length") = 1e-8)
  .def_static("from_config_files",
    &agv::PathGuide::FleetConfiguration::from_config_files,
    py::arg("config_file"),
    py::arg("nav_graph_path"),
    py::arg("server_uri") = std::nullopt)
  .def_property(
    "fleet_name",
    &agv::PathGuide::FleetConfiguration::fleet_name,
    &agv::PathGuide::FleetConfiguration::set_fleet_name)
  .def_property(
    "vehicle_traits",
    &agv::PathGuide::FleetConfiguration::vehicle_traits,
    &agv::PathGuide::FleetConfiguration::set_vehicle_traits)
  .def_property_readonly(
    "transformations_to_robot_coordinates",
    &agv::PathGuide::FleetConfiguration::
    transformations_to_robot_coordinates)
  .def(
    "add_robot_coordinates_transformation",
    &agv::PathGuide::FleetConfiguration::
    add_robot_coordinate_transformation)
  .def_property_readonly(
    "known_robot_configurations",
    &agv::PathGuide::FleetConfiguration::known_robot_configurations)
  .def_property_readonly(
    "known_robots",
    &agv::PathGuide::FleetConfiguration::known_robots)
  .def(
    "add_known_robot_configuration",
    &agv::PathGuide::FleetConfiguration::add_known_robot_configuration)
  .def(
    "get_known_robot_configuration",
    &agv::PathGuide::FleetConfiguration::get_known_robot_configuration)
  .def_property(
    "retreat_to_charger_interval",
    &agv::PathGuide::FleetConfiguration::retreat_to_charger_interval,
    &agv::PathGuide::FleetConfiguration::set_retreat_to_charger_interval)
  .def_property(
    "graph",
    &agv::PathGuide::FleetConfiguration::graph,
    &agv::PathGuide::FleetConfiguration::set_graph)
  .def_property(
    "battery_system",
    &agv::PathGuide::FleetConfiguration::battery_system,
    &agv::PathGuide::FleetConfiguration::set_battery_system)
  .def_property(
    "motion_sink",
    &agv::PathGuide::FleetConfiguration::motion_sink,
    &agv::PathGuide::FleetConfiguration::set_motion_sink)
  .def_property(
    "ambient_sink",
    &agv::PathGuide::FleetConfiguration::ambient_sink,
    &agv::PathGuide::FleetConfiguration::set_ambient_sink)
  .def_property(
    "tool_sink",
    &agv::PathGuide::FleetConfiguration::tool_sink,
    &agv::PathGuide::FleetConfiguration::set_tool_sink)
  .def_property(
    "recharge_threshold",
    &agv::PathGuide::FleetConfiguration::recharge_threshold,
    &agv::PathGuide::FleetConfiguration::set_recharge_threshold)
  .def_property(
    "recharge_soc",
    &agv::PathGuide::FleetConfiguration::recharge_soc,
    &agv::PathGuide::FleetConfiguration::set_recharge_soc)
  .def_property(
    "account_for_battery_drain",
    &agv::PathGuide::FleetConfiguration::account_for_battery_drain,
    &agv::PathGuide::FleetConfiguration::set_account_for_battery_drain)
  .def_property(
    "finishing_request",
    &agv::PathGuide::FleetConfiguration::finishing_request,
    &agv::PathGuide::FleetConfiguration::set_finishing_request)
  .def_property(
    "server_uri",
    &agv::PathGuide::FleetConfiguration::server_uri,
    &agv::PathGuide::FleetConfiguration::set_server_uri)
  .def_property(
    "max_delay",
    &agv::PathGuide::FleetConfiguration::max_delay,
    &agv::PathGuide::FleetConfiguration::set_max_delay)
  .def_property(
    "update_interval",
    &agv::PathGuide::FleetConfiguration::update_interval,
    &agv::PathGuide::FleetConfiguration::set_update_interval)
  .def_property(
    "default_responsive_wait",
    &agv::PathGuide::FleetConfiguration::default_responsive_wait,
    &agv::PathGuide::FleetConfiguration::set_default_responsive_wait)
  .def_property(
    "default_max_merge_waypoint_distance",
    &agv::PathGuide::FleetConfiguration::
    default_max_merge_waypoint_distance,
    &agv::PathGuide::FleetConfiguration::
    set_default_max_merge_waypoint_distance)
  .def_property(
    "default_max_merge_lane_distance",
    &agv::PathGuide::FleetConfiguration::default_max_merge_lane_distance,
    &agv::PathGuide::FleetConfiguration::
    set_default_max_merge_lane_distance)
  .def_property(
    "default_min_lane_length",
    &agv::PathGuide::FleetConfiguration::default_min_lane_length,
    &agv::PathGuide::FleetConfiguration::set_default_min_lane_length)

  .def_property_readonly(
    "lift_emergency_lanes",
    &agv::PathGuide::FleetConfiguration::lift_emergency_levels)
  .def(
    "set_lift_emergency_level",
    &agv::PathGuide::FleetConfiguration::set_lift_emergency_level);
}
