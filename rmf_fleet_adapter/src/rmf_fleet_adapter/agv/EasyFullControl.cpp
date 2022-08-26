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

#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

#include "internal_EasyFullControl.hpp"

#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic_ros2/Time.hpp>

// Public rmf_task API headers
#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ParkRobotFactory.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/MechanicalSystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <iostream>

namespace rmf_fleet_adapter {
namespace agv {


//==============================================================================
class EasyFullControl::Configuration::Implementation
{
public:

  const std::string node_name;
  const std::string fleet_name;
  const std::string config_file;
  const std::string nav_graph_path;
  std::optional<std::string> server_uri;

};

//==============================================================================
EasyFullControl::Configuration::Configuration(
  const std::string& node_name,
  const std::string& fleet_name,
  const std::string& config_file,
  const std::string& nav_graph_path,
  std::optional<std::string> server_uri)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(node_name),
        std::move(fleet_name),
        std::move(config_file),
        std::move(nav_graph_path),
        std::move(server_uri)
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& EasyFullControl::Configuration::node_name() const
{
  return _pimpl->node_name;
}

//==============================================================================
const std::string& EasyFullControl::Configuration::fleet_name() const
{
  return _pimpl->fleet_name;
}

//==============================================================================
const YAML::Node EasyFullControl::Configuration::fleet_config() const
{
  auto config_str = _pimpl->config_file;
  const auto fleet_config = YAML::LoadFile(_pimpl->config_file);
  return fleet_config;
}

//==============================================================================
Graph EasyFullControl::Configuration::graph() const
{
  auto graph = parse_graph(_pimpl->nav_graph_path, vehicle_traits());
  return graph;
}

//==============================================================================
VehicleTraits EasyFullControl::Configuration::vehicle_traits() const
{
  const auto config = fleet_config();
  const YAML::Node rmf_fleet = config["rmf_fleet"];

  // Profile and traits
  const YAML::Node profile = rmf_fleet["profile"];
  const double footprint_rad = profile["footprint"].as<double>();
  const double vicinity_rad = profile["vicinity"].as<double>();
  const YAML::Node limits = rmf_fleet["limits"];
  const YAML::Node linear = limits["linear"];
  const double v_nom = linear[0].as<double>();
  const double a_nom = linear[1].as<double>();
  const YAML::Node angular = limits["angular"];
  const double w_nom = angular[0].as<double>();
  const double b_nom = angular[1].as<double>();
  const bool reversible = rmf_fleet["reversible"].as<bool>();

  if (!reversible)
    std::cout << " ===== We have an irreversible robot" << std::endl;

  auto traits = VehicleTraits{
    {v_nom, a_nom},
    {w_nom, b_nom},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(footprint_rad),
      rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(vicinity_rad)
    }
  };
  traits.get_differential()->set_reversible(reversible);

  return traits;
}

//==============================================================================
std::optional<std::string> EasyFullControl::Configuration::server_uri() const
{
  return _pimpl->server_uri;
}

//==============================================================================
EasyFullControl::EasyFullControl()
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<EasyFullControl> EasyFullControl::make(Configuration config)
{
  // Make an Adapter instance
  const auto adapter = Adapter::make(config.node_name());
  if (!adapter)
    return nullptr;

  auto easy_handle = std::shared_ptr<EasyFullControl>(new EasyFullControl);
  easy_handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
    config, adapter);

  return easy_handle;
}

//==============================================================================
bool EasyFullControl::add_robot(
  const std::string& robot_name,
  Start pose,
  GetRobotState get_state,
  std::function<ProcessCompleted(const EasyFullControl::Navigate command)> navigate,
  std::function<ProcessCompleted()> stop,
  RobotUpdateHandle::ActionExecutor action_executor)
{
  // Create an EasyCommandHandle for this fella somehow somewhere
  // const auto command = std::make_shared<EasyCommandHandle>(
  //   *_pimpl->adapter->node(), _pimpl->fleet_name, robot_name,cout _pimpl->graph, _pimpl->traits);

  // // TODO(XY): obtain stuff NOT from config yaml. temporary solution
  // const YAML::Node robot_config = _pimpl->fleet_config["robots"][robot_name];
  // const double max_delay = robot_config["robot_config"]["max_delay"].as<double>();
  // const std::string map_name = robot_config["rmf_config"]["map_name"].as<std::string>();

  // Planner::StartSet starts;

  // // Use Start or compute plan starts
  // if (std::holds_alternative<Planner::Start>(pose))
  // {
  //   const auto p = std::get<Planner::Start>(pose);
  //   // Planner::StartSet starts;
  //   starts.push_back(p);
  // }
  // else if (std::holds_alternative<Eigen::Vector3d>(pose))
  // {
  //   const auto p = std::get<Eigen::Vector3d>(pose);

  //   // Use compute plan starts to estimate the start
  //   starts = rmf_traffic::agv::compute_plan_starts(
  //     *_pimpl->graph, map_name, {p.x(), p.y(), p.z()},
  //     rmf_traffic_ros2::convert(_pimpl->adapter->node()->now()));
  // }

  // if (starts.empty())
  // {
  //   RCLCPP_ERROR(_pimpl->adapter->node()->get_logger(),
  //     "Unable to determine StartSet for %s", robot_name.c_str());

  //   return false;
  // }

  // // Add robot to fleet
  // _pimpl->fleet_handle->add_robot(
  //   command, robot_name, _pimpl->traits->profile(), starts, 
  //   [w = this->weak_from_this(), command, robot_name = std::move(robot_name)](
  //     const RobotUpdateHandlePtr& updater)
  //   {
  //     const auto self = w.lock();
  //     if (!self)
  //       return;
  //     // auto lock = self->lock();

  //     command->set_updater(updater);
  //     self->_pimpl->robots[robot_name] = command;
  //   });

  return true;

}

//==============================================================================
bool EasyFullControl::Implementation::initialize_fleet()
{
  const auto& node = adapter->node();
  fleet_name = config.fleet_name();
  fleet_config = config.fleet_config();
  traits = std::make_shared<VehicleTraits>(config.vehicle_traits());

  graph = std::make_shared<Graph>(config.graph());
  std::cout << "The fleet [" << fleet_name
            << "] has the following named waypoints:\n";
  for (const auto& key : graph->keys())
    std::cout << " -- " << key.first << std::endl;

  auto server_uri = config.server_uri();

  // Add fleet to adapter
  fleet_handle = adapter->add_fleet(
    fleet_name, *traits, *graph, server_uri);

  // Set the fleet state topic publish period
  // TODO(XY): make this configurable
  fleet_handle->fleet_state_topic_publish_period(std::nullopt);

  closed_lanes_pub = adapter->node()->create_publisher<rmf_fleet_msgs::msg::ClosedLanes>(
    rmf_fleet_adapter::ClosedLaneTopicName,
    rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local());

  // Create subscription to lane closure requests
  lane_closure_request_sub =
    adapter->node()->create_subscription<rmf_fleet_msgs::msg::LaneRequest>(
    rmf_fleet_adapter::LaneClosureRequestTopicName,
    rclcpp::SystemDefaultsQoS(),
    [this](
      rmf_fleet_msgs::msg::LaneRequest::UniquePtr request_msg)
    {

      if (request_msg->fleet_name != this->fleet_name &&
      !request_msg->fleet_name.empty())
        return;

      this->fleet_handle->open_lanes(request_msg->open_lanes);
      this->fleet_handle->close_lanes(request_msg->close_lanes);

      std::unordered_set<std::size_t> newly_closed_lanes;
      for (const auto& l : request_msg->close_lanes)
      {
        if (closed_lanes.count(l) == 0)
          newly_closed_lanes.insert(l);

        closed_lanes.insert(l);
      }

      for (const auto& l : request_msg->open_lanes)
        closed_lanes.erase(l);

      for (auto& [_, robot] : robots)
        robot->newly_closed_lanes(newly_closed_lanes);

      rmf_fleet_msgs::msg::ClosedLanes state_msg;
      state_msg.fleet_name = this->fleet_name;
      state_msg.closed_lanes.insert(
        state_msg.closed_lanes.begin(),
        closed_lanes.begin(),
        closed_lanes.end());

      closed_lanes_pub->publish(state_msg);
    });

  // Set up parameters required for task planner
  const YAML::Node rmf_fleet = fleet_config["rmf_fleet"];

  // Battery system
  const YAML::Node battery = rmf_fleet["battery_system"];
  const double voltage = battery["voltage"].as<double>();
  const double capacity = battery["capacity"].as<double>();
  const double charging_current = battery["charging_current"].as<double>();

  auto battery_system_optional = rmf_battery::agv::BatterySystem::make(
    voltage, capacity, charging_current);
  auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
    *battery_system_optional);

  // Mechanical system
  const YAML::Node mechanical = rmf_fleet["mechanical_system"];
  const double mass = mechanical["mass"].as<double>();
  const double moment_of_inertia = mechanical["moment_of_inertia"].as<double>();
  const double friction = mechanical["friction_coefficient"].as<double>();

  auto mechanical_system_optional = rmf_battery::agv::MechanicalSystem::make(
    mass, moment_of_inertia, friction);
  rmf_battery::agv::MechanicalSystem& mechanical_system =
    *mechanical_system_optional;

  std::shared_ptr<rmf_battery::agv::SimpleMotionPowerSink> motion_sink =
    std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
    *battery_system, mechanical_system);

  // Ambient power system
  const YAML::Node ambient_system = rmf_fleet["ambient_system"];
  const double ambient_power_drain = ambient_system["power"].as<double>();
  auto ambient_power_system = rmf_battery::agv::PowerSystem::make(
    ambient_power_drain);
  if (!ambient_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for ambient power system");

    return false;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> ambient_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *ambient_power_system);

  // Tool power system
  const YAML::Node tool_system = rmf_fleet["tool_system"];
  const double tool_power_drain = ambient_system["power"].as<double>();
  auto tool_power_system = rmf_battery::agv::PowerSystem::make(
    tool_power_drain);
  if (!tool_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for tool power system");

    return false;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> tool_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  // Drain battery
  const bool drain_battery = rmf_fleet["account_for_battery_drain"].as<bool>();
  // Recharge threshold
  const double recharge_threshold = rmf_fleet["recharge_threshold"].as<double>();
  // Recharge state of charge
  const double recharge_soc = rmf_fleet["recharge_soc"].as<double>();

  // Finishing tasks
  const YAML::Node task_capabilities = rmf_fleet["task_capabilities"];
  const std::string finishing_request_string = task_capabilities["finishing_request"].as<std::string>();
  rmf_task::ConstRequestFactoryPtr finishing_request = nullptr;
  if (finishing_request_string == "charge")
  {
    finishing_request =
      std::make_shared<rmf_task::requests::ChargeBatteryFactory>();
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is configured to perform ChargeBattery as finishing request");
  }
  else if (finishing_request_string == "park")
  {
    finishing_request =
      std::make_shared<rmf_task::requests::ParkRobotFactory>();
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is configured to perform ParkRobot as finishing request");
  }
  else if (finishing_request_string == "nothing")
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is not configured to perform any finishing request");
  }
  else
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Provided finishing request [%s] is unsupported. The valid "
      "finishing requests are [charge, park, nothing]. The task planner will "
      " default to [nothing].",
      finishing_request_string.c_str());
  }

  // Set task planner params
  if (!fleet_handle->set_task_planner_params(
      battery_system,
      motion_sink,
      ambient_sink,
      tool_sink,
      recharge_threshold,
      recharge_soc,
      drain_battery,
      finishing_request))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to initialize task planner parameters");

    return false;
  }

  // Accept task types

  // TODO: Currently accepting all by default, check if additional stuff is needed
  const auto consider =
  [](const nlohmann::json& description,
    rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
  {
    confirm.accept();
  };

  const bool perform_loop = task_capabilities["loop"].as<bool>();
  if (perform_loop)
  {
    fleet_handle->consider_patrol_requests(consider);
  }

  const bool perform_delivery = task_capabilities["delivery"].as<bool>();
  if (perform_delivery)
  {
    fleet_handle->consider_delivery_requests(consider,
                                             consider);
  }

  const bool perform_cleaning = task_capabilities["clean"].as<bool>();
  if (perform_cleaning)
  {
    fleet_handle->consider_cleaning_requests(consider);
  }

  // Configure this fleet to perform any kind of teleop action
  fleet_handle->add_performable_action("teleop", consider);

  return true;
}

} // namespace agv
} // namespace rmf_fleet_adapter