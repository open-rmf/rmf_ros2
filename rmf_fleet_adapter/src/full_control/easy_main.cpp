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

// Public rmf_fleet_adapter API headers
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

// Standard topic names for communicating with fleet drivers
#include <rmf_fleet_adapter/StandardNames.hpp>

// Example API: RobotState and PathRequest
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/dock_summary.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <yaml-cpp/yaml.h>


using EasyFullControl = rmf_fleet_adapter::agv::EasyFullControl;
using RobotUpdateHandle = rmf_fleet_adapter::agv::RobotUpdateHandle;

//==============================================================================
struct State
{
  rmf_fleet_msgs::msg::RobotState state;
  std::optional<Eigen::Vector3d> destination;
};

//==============================================================================
struct FleetManager : public std::enable_shared_from_this<FleetManager>
{
public:

  void initialize_manager(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name,
    std::vector<std::string> robot_names)
  {
    _node = node;
    _fleet_name = fleet_name;

    // Set up topics
    _robot_state_sub = node->create_subscription<rmf_fleet_msgs::msg::RobotState>(
      "/robot_state",
      rclcpp::QoS(10),
      std::bind(&FleetManager::robot_state_cb, this, std::placeholders::_1));

    _path_request_pub = node->create_publisher<rmf_fleet_msgs::msg::PathRequest>(
      rmf_fleet_adapter::PathRequestTopicName,
      rclcpp::SystemDefaultsQoS());

    _dock_summary_sub =
      _node->create_subscription<rmf_fleet_msgs::msg::DockSummary>(
      "dock_summary",
      rclcpp::QoS(10).reliable().keep_last(1).transient_local(),
      [this](rmf_fleet_msgs::msg::DockSummary::UniquePtr msg)
      {
        for (const auto fleet : msg->docks)
        {
          if (fleet.fleet_name == _fleet_name)
          {
            for (const auto dock : fleet.params)
            {
              _docks[dock.start] = dock.path;
            }
          }
        }
      });

    // Set up robots
    for (const auto& name : robot_names)
      _robots[name] = State();
  }

  EasyFullControl::Position get_position(
    const std::string& robot_name)
  {
    // if (_robots.find(robot_name) == _robots.end())
    //   return;

    EasyFullControl::Position position;
    auto location = _robots[robot_name].state.location;
    Eigen::Vector3d pose(location.x, location.y, location.yaw);
    position.position = pose;
    position.map_name = location.level_name;
    position.battery_percent = _robots[robot_name].state.battery_percent;

    std::optional<double> dist_to_target;
    if (_robots[robot_name].destination.has_value())
    {
      auto destination = _robots[robot_name].destination.value();
      dist_to_target = dist(pose, destination);
    }
    else
    {
      dist_to_target = std::nullopt;
    }
    position.dist_to_target = dist_to_target;

    return position;
  }

  EasyFullControl::ProcessCompleted navigate(
    const std::string& robot_name,
    const EasyFullControl::Target target)
  {
    // if (_robots.find(robot_name) == _robots.end())
    //   return;

    rmf_fleet_msgs::msg::PathRequest path_request;
    path_request.fleet_name = _fleet_name;
    path_request.robot_name = robot_name;
    path_request.task_id = std::to_string(++_task_id);
  
    // Append current pose to path request
    rmf_fleet_msgs::msg::Location cur_loc;
    auto location = _robots[robot_name].state.location;
    cur_loc.x = location.x;
    cur_loc.y = location.y;
    cur_loc.yaw = location.yaw;
    cur_loc.level_name = location.level_name;
    path_request.path.push_back(std::move(cur_loc));

    // Append target pose to path request
    rmf_fleet_msgs::msg::Location target_loc;
    target_loc.x = target.pose.x();
    target_loc.y = target.pose.y();
    target_loc.yaw = target.pose.z();
    target_loc.level_name = target.map_name;

    if (target.speed_limit.has_value())
    {
      target_loc.obey_approach_speed_limit = true;
      target_loc.approach_speed_limit = target.speed_limit.value();
    }
    else
    {
      target_loc.obey_approach_speed_limit = false;
    }
    path_request.path.push_back(std::move(target_loc));

    RCLCPP_INFO(_node->get_logger(),
      "Fleet [%s] received navigation for robot [%s], publishing "
      "PathRequest to target [%.2f, %.2f, %.2f]",
      _fleet_name.c_str(), robot_name.c_str(),
      target.pose.x(), target.pose.y(), target.pose.z());
    _path_request_pub->publish(path_request);

    _robots[robot_name].destination = target.pose;

    auto navigation_completed =
    [this, robot_name]()
    {
      if (!this->_robots[robot_name].destination.has_value())
        return true;
      else
        return false;
    };

    return navigation_completed;
  }

  bool stop(
    const std::string& robot_name)
  {
    rmf_fleet_msgs::msg::PathRequest stop_request;
    stop_request.fleet_name = _fleet_name;
    stop_request.robot_name = robot_name;
    stop_request.task_id = std::to_string(++_task_id);
  
    // Append current pose to path request
    rmf_fleet_msgs::msg::Location cur_loc;
    auto location = _robots[robot_name].state.location;
    cur_loc.x = location.x;
    cur_loc.y = location.y;
    cur_loc.yaw = location.yaw;
    cur_loc.level_name = location.level_name;
    stop_request.path.push_back(std::move(cur_loc));

    // Send PathRequest to its current location
    rmf_fleet_msgs::msg::Location target_loc;
    target_loc.x = location.x;
    target_loc.y = location.y;
    target_loc.yaw = location.yaw;
    target_loc.level_name = location.level_name;
    stop_request.path.push_back(target_loc);

    RCLCPP_INFO(_node->get_logger(),
      "Fleet [%s] received stop request for robot [%s], "
      "asking robot to stop...",
      _fleet_name.c_str(), robot_name.c_str());
    _path_request_pub->publish(stop_request);

    return true;
  }

  EasyFullControl::ProcessCompleted dock(
    const std::string& robot_name,
    const std::string& dock_name)
  {
    // TODO: if robot name not in .....
    // if (_robots.find(robot_name) == _robots.end() ||
    //     _docks.find(dock_name) == _docks.end())
    //   return;

    rmf_fleet_msgs::msg::PathRequest path_request;
    path_request.fleet_name = _fleet_name;
    path_request.robot_name = robot_name;
    path_request.task_id = std::to_string(++_task_id);
  
    // Append current pose to path request
    rmf_fleet_msgs::msg::Location cur_loc;
    auto location = _robots[robot_name].state.location;
    cur_loc.x = location.x;
    cur_loc.y = location.y;
    cur_loc.yaw = location.yaw;
    cur_loc.level_name = location.level_name;
    path_request.path.push_back(std::move(cur_loc));

    // Append target pose to path request
    rmf_fleet_msgs::msg::Location target_loc;
    for (const auto& wp : _docks[dock_name])
    {
      target_loc = wp;
      path_request.path.push_back(std::move(wp));
    }

    RCLCPP_INFO(_node->get_logger(),
      "Fleet [%s] received docking request for robot [%s]",
      _fleet_name.c_str(), robot_name.c_str());
    _path_request_pub->publish(path_request);

    _robots[robot_name].destination = Eigen::Vector3d(target_loc.x, target_loc.y, target_loc.yaw);

    auto docking_completed =
    [this, robot_name]()
    {
      if (!this->_robots[robot_name].destination.has_value())
        return true;
      else
        return false;
    };

    return docking_completed;
  }

  void action_executor(
    const std::string& robot_name,
    const std::string& category,
    const nlohmann::json& description,
    RobotUpdateHandle::ActionExecution execution)
  {
    // TODO
  }

  void robot_state_cb(rmf_fleet_msgs::msg::RobotState msg)
  {
    {
      if (_robots.find(msg.name) == _robots.end())
        return;

        const auto me = this->lock();
        if (!me)
          return;

        std::lock_guard<std::recursive_mutex> lock(_mutex);

        _robots[msg.name].state = msg;

        // Check if reached destination
        if ((msg.mode.mode == 0 || msg.mode.mode == 1) &&
            msg.path.size() == 0)
        {
          _robots[msg.name].destination = std::nullopt;
        }
    }
  }

  double dist(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
  {
    return (a.block<2, 1>(0, 0) - b.block<2, 1>(0, 0)).norm();
  }


// private:

  std::shared_ptr<rclcpp::Node> _node;
  std::string _fleet_name;
  std::unordered_map<std::string, State> _robots;
  std::unordered_map<std::string, std::vector<rmf_fleet_msgs::msg::Location>> _docks;
  int _task_id = -1;

  rclcpp::Subscription<rmf_fleet_msgs::msg::RobotState>::SharedPtr _robot_state_sub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr _path_request_pub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::DockSummary>::SharedPtr _dock_summary_sub;

  std::recursive_mutex _mutex;
  std::unique_lock<std::recursive_mutex> lock()
  {
    std::unique_lock<std::recursive_mutex> l(_mutex, std::defer_lock);
    while (!l.try_lock())
    {
      // Intentionally busy wait
    }

    return l;
  }

};


//==============================================================================
Eigen::Vector2d get_waypoint(
  const std::string& waypoint_name,
  const std::string& nav_graph_file,
  const YAML::Node config_yaml)
{
  // Get vehicle traits
  const YAML::Node rmf_fleet = config_yaml["rmf_fleet"];

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

  auto traits = rmf_traffic::agv::VehicleTraits{
    {v_nom, a_nom},
    {w_nom, b_nom},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(footprint_rad),
      rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(vicinity_rad)
    }
  };
  traits.get_differential()->set_reversible(reversible);

  auto graph = rmf_fleet_adapter::agv::parse_graph(nav_graph_file, traits);

  return graph.find_waypoint(waypoint_name)->get_location();
}


//==============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  //----------------------------------------------------------------------------
  // Initialize Adapter
  //----------------------------------------------------------------------------

  // Make an Adapter instance
  const auto adapter =
    rmf_fleet_adapter::agv::Adapter::make("fleet_command_handle");
  if (!adapter)
    return 1;

  //----------------------------------------------------------------------------
  // Set up Configuration and make EasyFullControl fleet
  //----------------------------------------------------------------------------

  const std::string nav_graph_param_name = "nav_graph_file";
  const std::string nav_graph_path =
    adapter->node()->declare_parameter(nav_graph_param_name, std::string());
  if (nav_graph_path.empty())
  {
    std::cout << "Missing [" << nav_graph_param_name.c_str() << "] parameter" << std::endl;
    return 1;
  }

  const std::string config_param_name = "config_file";
  const std::string config_file =
    adapter->node()->declare_parameter(config_param_name, std::string());
  if (config_file.empty())
  {
    std::cout << "Missing [" << config_param_name.c_str() << "] parameter" << std::endl;
    return 1;
  }

  std::optional<std::string> server_uri = std::nullopt;
  const std::string uri = adapter->node()->declare_parameter("server_uri", std::string());
  if (!uri.empty())
  {
    RCLCPP_INFO(
      adapter->node()->get_logger(),
      "API server URI: [%s]", uri.c_str());

    server_uri = uri;
  }

  // Set up Configuration to easily parse parameters to Adapter
  auto adapter_config = EasyFullControl::Configuration(config_file,
                                                       nav_graph_path,
                                                       server_uri);

  const auto easy_adapter = EasyFullControl::make(adapter_config, adapter);
  if (!easy_adapter)
    return 1;

  //----------------------------------------------------------------------------
  // Initialize FleetManager
  //----------------------------------------------------------------------------

  // Load robot names from config
  const auto config_yaml = YAML::LoadFile(config_file);
  std::vector<std::string> robot_names;
  YAML::Node robot_config = config_yaml["robots"];
  for (YAML::const_iterator it=robot_config.begin(); it != robot_config.end(); ++it)
  {
    std::string robot = it->first.as<std::string>();
    robot_names.push_back(robot);
    RCLCPP_INFO(adapter->node()->get_logger(),
    "Robot detected: %s",
    robot.c_str());
  }

  const std::string fleet_name = config_yaml["rmf_fleet"]["name"].as<std::string>();
  auto fleet_manager = std::make_shared<FleetManager>();
  fleet_manager->initialize_manager(adapter->node(),
                                    fleet_name,
                                    robot_names);

  //----------------------------------------------------------------------------
  // Add robots to fleet adapter
  //----------------------------------------------------------------------------

  // Add robots to adapter
  for (auto robot : robot_names)
  {
    const auto get_position =
      [fleet_manager, robot]()
      {
        return fleet_manager->get_position(robot);
      };

    const auto navigate =
      [fleet_manager, robot](
      const EasyFullControl::Target target)
      {
        return fleet_manager->navigate(robot, target);
      };

    const auto stop =
      [fleet_manager, robot]()
      {
        return fleet_manager->stop(robot);
      };

    const auto action_executor =
      [fleet_manager, robot](
      const std::string& category,
      const nlohmann::json& description,
      RobotUpdateHandle::ActionExecution execution)
      {
        return fleet_manager->action_executor(robot, category, description, execution);
      };

    const auto dock =
      [fleet_manager, robot](
      const std::string& dock_name)
      {
        return fleet_manager->dock(robot, dock_name);
      };

    const YAML::Node robot_conf = robot_config[robot]["rmf_config"]["start"];
    std::string start_wp = robot_conf["waypoint"].as<std::string>();
    RCLCPP_INFO(adapter->node()->get_logger(),
    start_wp.c_str());

    // auto start_pose = get_position().position;
    auto wp = get_waypoint(start_wp, nav_graph_path, config_yaml);
    Eigen::Vector3d start_pose(wp.x(), wp.y(), 0.0);
    RCLCPP_INFO(adapter->node()->get_logger(),
      "Set start pose for [%s] to [%.2f, %.2f, %.2f]",
      robot.c_str(), start_pose.x(), start_pose.y(), start_pose.z());

    auto success = easy_adapter->add_robot(
      robot, start_pose, get_position, navigate,
      dock, stop, action_executor);
  }


  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Fleet Adapter");

  // Start running the adapter and wait until it gets stopped by SIGINT
  adapter->start().wait();

  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();

}