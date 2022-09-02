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

// Standard topic names for communicating with fleet drivers
#include <rmf_fleet_adapter/StandardNames.hpp>

// Example API: RobotState and PathRequest
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

#include "rclcpp/rclcpp.hpp"
#include <chrono>


using EasyFullControl = rmf_fleet_adapter::agv::EasyFullControl;
using RobotUpdateHandle = rmf_fleet_adapter::agv::RobotUpdateHandle;

//==============================================================================
struct RobotManager : public std::enable_shared_from_this<RobotManager>
{
  EasyFullControl::Position get_position()
  {
    std::optional<double> dist_to_target;
    if (_destination.has_value())
    {
      dist_to_target = dist(_position.position, _destination.value());
    }
    else
    {
      dist_to_target = std::nullopt;
    }
    _position.dist_to_target = dist_to_target;

    return _position;
  }

  EasyFullControl::ProcessCompleted navigate(
    const EasyFullControl::Target target)
  {
    rmf_fleet_msgs::msg::PathRequest path_request;
    path_request.fleet_name = _fleet_name;
    path_request.robot_name = _robot_name;
    path_request.task_id = std::to_string(++_current_task_id);
  
    // Append current pose to path request
    rmf_fleet_msgs::msg::Location cur_loc;
    cur_loc.x = _position.position.x();
    cur_loc.y = _position.position.y();
    cur_loc.yaw = _position.position.z();
    cur_loc.level_name = _position.map_name;
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
      _fleet_name.c_str(), _robot_name.c_str(),
      target.pose.x(), target.pose.y(), target.pose.z());
    _path_request_pub->publish(path_request);

    _destination = target.pose;

    auto navigation_completed =
    [this]()
    {
      if (this->_destination == std::nullopt)
        return true;
      else
        return false;
    };

    return navigation_completed;
  }

  bool stop()
  {
    rmf_fleet_msgs::msg::PathRequest stop_request;
    stop_request.fleet_name = _fleet_name;
    stop_request.robot_name = _robot_name;
    stop_request.task_id = std::to_string(++_current_task_id);
  
    // Append current pose to path request
    rmf_fleet_msgs::msg::Location cur_loc;
    cur_loc.x = _position.position.x();
    cur_loc.y = _position.position.y();
    cur_loc.yaw = _position.position.z();
    cur_loc.level_name = _position.map_name;
    stop_request.path.push_back(cur_loc);

    // Send PathRequest to its current location
    rmf_fleet_msgs::msg::Location target_loc;
    target_loc.x = _position.position.x();
    target_loc.y = _position.position.y();
    target_loc.yaw = _position.position.z();
    target_loc.level_name = _position.map_name;
    stop_request.path.push_back(target_loc);

    RCLCPP_INFO(_node->get_logger(),
      "Fleet [%s] received stop request for robot [%s], "
      "asking robot to stop...",
      _fleet_name.c_str(), _robot_name.c_str());
    _path_request_pub->publish(stop_request);

    return true;
  }

  void action_executor(
    const std::string& category,
    const nlohmann::json& description,
    RobotUpdateHandle::ActionExecution execution)
  {
    //
  }

// private:
  double dist(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
  {
    return (a.block<2, 1>(0, 0) - b.block<2, 1>(0, 0)).norm();
  }

  std::shared_ptr<rclcpp::Node> _node;
  std::string _robot_name;
  std::string _fleet_name;
  uint32_t _current_task_id = 0;
  EasyFullControl::Position _position;
  std::optional<Eigen::Vector3d> _destination = std::nullopt;
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr _path_request_pub;
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

    // Set up robots
    for (const auto& name : robot_names)
    {
      _robots[name] = std::make_shared<RobotManager>();
      _robots[name]->_node = _node;
      _robots[name]->_fleet_name = _fleet_name;
      _robots[name]->_robot_name = name;
      _robots[name]->_path_request_pub = _path_request_pub;
    }
  }

  void robot_state_cb(rmf_fleet_msgs::msg::RobotState msg)
  {
    if (_robots.find(msg.name) == _robots.end())
      return;

    const auto me = this->lock();
    if (!me)
      return;

    std::lock_guard<std::recursive_mutex> lock(_mutex);

    _robots[msg.name]->_position = EasyFullControl::Position{
      .position = Eigen::Vector3d(msg.location.x, msg.location.y, msg.location.yaw),
      .map_name = msg.location.level_name,
      .battery_percent = msg.battery_percent
    };

    // Check if reached destination
    if ((msg.mode.mode == 0 || msg.mode.mode == 1) &&
        msg.path.size() == 0)
    {
      _robots[msg.name]->_destination = std::nullopt;
    }
  }

// private:

  std::shared_ptr<rclcpp::Node> _node;
  std::string _fleet_name;
  std::unordered_map<std::string, std::shared_ptr<RobotManager>> _robots;
  rclcpp::Subscription<rmf_fleet_msgs::msg::RobotState>::SharedPtr _robot_state_sub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr _path_request_pub;

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

// Temp: manually provide file path to config and nav graph
bool use_sim_time = true;
std::string node_name = "tinyRobot_command_handle";
std::string fleet_name = "tinyRobot";
std::string config_file = "/home/xiyu/rmf_ws/install/rmf_demos/share/rmf_demos/config/office/tinyRobot_config.yaml";
std::string nav_graph_path = "/home/xiyu/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml";


//==============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Make an Adapter instance
  const auto adapter =
    rmf_fleet_adapter::agv::Adapter::make(node_name);
  if (!adapter)
    return 1;

  // const std::string nav_graph_param_name = "nav_graph_file";
  // const std::string nav_graph_path =
  //   adapter->node()->declare_parameter(nav_graph_param_name, std::string());
  // if (nav_graph_path.empty())
  // {
  //   RCLCPP_ERROR(
  //     adapter->node()->get_logger(),
  //     "Missing [%s] parameter", nav_graph_param_name.c_str());

  //   return 1;
  // }

  // const std::string config_param_name = "config_file";
  // const std::string config_file =
  //   adapter->node()->declare_parameter(nav_graph_param_name, std::string());
  // if (config_file.empty())
  // {
  //   RCLCPP_ERROR(
  //     adapter->node()->get_logger(),
  //     "Missing [%s] parameter", nav_graph_param_name.c_str());

  //   return 1;
  // }

  // const bool use_sim_time = adapter->node()->declare_parameter("use_sim_time", bool());
  if (use_sim_time)
  {
    rclcpp::Parameter param("use_sim_time", true);
    adapter->node()->set_parameter(param);
  }

  // Set up Configuration to easily parse parameters to Adapter
  auto adapter_config = EasyFullControl::Configuration(fleet_name,
                                                       config_file,
                                                       nav_graph_path);

  const auto easy_adapter = EasyFullControl::make(adapter_config, adapter);
  if (!easy_adapter)
    return 1;

  //----------------------------------------------------------------------------
  // Initialize FleetManager
  //----------------------------------------------------------------------------

  std::vector<std::string> robot_names = {"tinyRobot1", "tinyRobot2"};
  auto fleet_manager = std::make_shared<FleetManager>();
  fleet_manager->initialize_manager(adapter->node(),
                                    fleet_name,
                                    robot_names);

  //----------------------------------------------------------------------------
  // Add tinyRobot1
  //----------------------------------------------------------------------------

  // Prep robot manager functions
  // (TODO) get pose directly from robot_state instead of manually
  // (TODO) make loop instead of manually for each robot
  Eigen::Vector3d start_pose_1 = Eigen::Vector3d(10.4330, -5.5750, 1.3280);
  RCLCPP_INFO(adapter->node()->get_logger(),
    "Set start pose for [tinyRobot1] to [%.2f, %.2f, %.2f]",
    start_pose_1.x(), start_pose_1.y(), start_pose_1.z());

  const auto add_get_position_1 =
    [fleet_manager]()
    {
      return fleet_manager->_robots["tinyRobot1"]->get_position();
    };

  const auto add_navigate_1 =
    [fleet_manager](
    const EasyFullControl::Target target)
    {
      return fleet_manager->_robots["tinyRobot1"]->navigate(target);
    };

  const auto add_stop_1 =
    [fleet_manager]()
    {
      return fleet_manager->_robots["tinyRobot1"]->stop();
    };

  const auto add_action_executor_1 =
    [fleet_manager](
    const std::string& category,
    const nlohmann::json& description,
    RobotUpdateHandle::ActionExecution execution)
    {
      return fleet_manager->_robots["tinyRobot1"]->action_executor(category, description, execution);
    };

  // Add robot here
  auto robot_1_success = easy_adapter->add_robot(
    "tinyRobot1", start_pose_1, add_get_position_1,
    add_navigate_1, add_stop_1, add_action_executor_1);

  //----------------------------------------------------------------------------
  // Add tinyRobot2
  //----------------------------------------------------------------------------

  Eigen::Vector3d start_pose_2 = Eigen::Vector3d(20.4237, -5.3121, -0.6966);

  RCLCPP_INFO(adapter->node()->get_logger(),
    "Set start pose for [tinyRobot2] to [%.2f, %.2f, %.2f]",
    start_pose_2.x(), start_pose_2.y(), start_pose_2.z());

  const auto add_get_position_2 =
    [fleet_manager]()
    {
      return fleet_manager->_robots["tinyRobot2"]->get_position();
    };

  const auto add_navigate_2 =
    [fleet_manager](
    const EasyFullControl::Target target)
    {
      return fleet_manager->_robots["tinyRobot2"]->navigate(target);
    };

  const auto add_stop_2 =
    [fleet_manager]()
    {
      return fleet_manager->_robots["tinyRobot2"]->stop();
    };

  const auto add_action_executor_2 =
    [fleet_manager](
    const std::string& category,
    const nlohmann::json& description,
    RobotUpdateHandle::ActionExecution execution)
    {
      return fleet_manager->_robots["tinyRobot2"]->action_executor(category, description, execution);
    };

  // Add robot here
  auto robot_2_success = easy_adapter->add_robot(
    "tinyRobot2", start_pose_2, add_get_position_2,
    add_navigate_2, add_stop_2, add_action_executor_2);

  //----------------------------------------------------------------------------
  //----------------------------------------------------------------------------

  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Fleet Adapter");

  // Start running the adapter and wait until it gets stopped by SIGINT
  adapter->start().wait();

  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();


  // TODO: add use_sim_time somewhere
}