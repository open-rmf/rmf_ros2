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

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/closed_lanes.hpp>
#include <rmf_fleet_msgs/msg/interrupt_request.hpp>
#include <rmf_fleet_msgs/msg/dock_summary.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic_ros2/Time.hpp>

// Public rmf_task API headers
#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ParkRobotFactory.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/MechanicalSystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include "rclcpp/rclcpp.hpp"
#include "Node.hpp"
#include <thread>
#include <yaml-cpp/yaml.h>

#include <iostream>

namespace rmf_fleet_adapter {
namespace agv {


//==============================================================================
class EasyFullControl::Configuration::Implementation
{
public:

  const std::string config_file;
  const std::string nav_graph_path;
  std::optional<std::string> server_uri;

};

//==============================================================================
EasyFullControl::Configuration::Configuration(
  const std::string& config_file,
  const std::string& nav_graph_path,
  std::optional<std::string> server_uri)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(config_file),
        std::move(nav_graph_path),
        std::move(server_uri)
      }))
{
  // Do nothing
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
      rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(
        footprint_rad),
      rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(
        vicinity_rad)
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
EasyFullControl::EasyCommandHandle::EasyCommandHandle(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& fleet_name,
  const std::string& robot_name,
  std::shared_ptr<Graph> graph,
  std::shared_ptr<VehicleTraits> traits,
  Transformer rmf_to_robot_transformer,
  const std::string& map_name,
  std::optional<rmf_traffic::Duration> max_delay,
  const Planner::Start& start,
  const Eigen::Vector3d& initial_position,
  double initial_battery_soc,
  std::size_t charger_waypoint,
  GetPosition get_position,
  std::function<ProcessCompleted(const Target target)> navigate,
  std::function<ProcessCompleted(const std::string& dock_name)> dock,
  ProcessCompleted stop,
  RobotUpdateHandle::ActionExecutor action_executor)
: _node(node),
  _fleet_name(fleet_name),
  _robot_name(std::move(robot_name)),
  _graph(graph),
  _traits(traits),
  _rmf_to_robot_transformer(rmf_to_robot_transformer),
  _map_name(std::move(map_name)),
  _max_delay(max_delay),
  _position(initial_position),
  _battery_soc(initial_battery_soc),
  _charger_waypoint(charger_waypoint),
  _get_position(std::move(get_position)),
  _navigate(std::move(navigate)),
  _dock(std::move(dock)),
  _stop(std::move(stop)),
  _action_executor(std::move(action_executor))
{
  _updater = nullptr;
  _is_charger_set = false;
  _stop_follow_thread = false;

  if (start.lane().has_value())
    _on_lane = start.lane().value();
  else
  {
    _on_waypoint = start.waypoint();
    _last_known_waypoint = start.waypoint();
  }

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

  _action_execution_sub =
    _node->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
    "action_execution_notice",
    rclcpp::SystemDefaultsQoS(),
    [this](rmf_fleet_msgs::msg::ModeRequest::UniquePtr msg)
    {
      if (msg->fleet_name.empty() ||
      msg->fleet_name != _fleet_name ||
      msg->robot_name.empty())
        return;

      if (msg->mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_IDLE)
      {
        complete_robot_action();
      }
    });

  _initialized = true;
}

//==============================================================================
EasyFullControl::EasyCommandHandle::~EasyCommandHandle()
{
  if (_follow_thread.joinable())
  {
    _stop_follow_thread = true;
    _follow_thread.join();
  }

  if (_update_thread.joinable())
    _update_thread.join();
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::start_update_thread()
{
  _update_thread = std::thread(
    [w = weak_from_this()]()
    {
      auto me = w.lock();
      if (me)
      {
        me->update_state();
      }
    });
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::follow_new_path(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  ArrivalEstimator next_arrival_estimator,
  RequestCompleted path_finished_callback)
{
  if (waypoints.empty() ||
    next_arrival_estimator == nullptr ||
    path_finished_callback == nullptr)
  {
    RCLCPP_WARN(
      _node->get_logger(),
      "Robot [%s] received follow_new_path request with invalid parameters. "
      " Ignoring...",
      _robot_name.c_str());
    return;
  }

  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] received a new path to follow...",
    _robot_name.c_str());
  for (const auto& wp : waypoints)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "\t[%.2f,%.2f, %.2f]: %d",
      wp.position()[0], wp.position()[1], wp.position()[1],
      wp.time().time_since_epoch().count());
  }

  // stop();
  if (_follow_thread.joinable())
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "[stop] _follow_thread present. Calling join");
    _stop_follow_thread = true;
    _follow_thread.join();
  }

  _target_waypoint = std::nullopt;
  _remaining_waypoints.clear();
  _state = RobotState::IDLE;

  parse_waypoints(waypoints);
  RCLCPP_DEBUG(
    _node->get_logger(),
    "_remaining_waypoints: [%d]. _target_waypoint: [%d]",
    _remaining_waypoints.size(), _target_waypoint.has_value());
  for (const auto& wp : _remaining_waypoints)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "[%.2f,%.2f, %.2f]: %d",
      wp.position[0], wp.position[1], wp.position[1],
      wp.time.time_since_epoch().count());
  }

  _next_arrival_estimator = std::move(next_arrival_estimator);
  _path_finished_callback = std::move(path_finished_callback);

  // Robot needs to wait
  if (_target_waypoint.has_value())
  {
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[follow_new_path] Target waypoint has a value. Robot [%s] will wait...",
      _robot_name.c_str());
    _state = RobotState::WAITING;
    _on_waypoint = _target_waypoint.value().graph_index;
  }

  _stop_follow_thread = false;
  _follow_thread = std::thread(
    [w = weak_from_this()]()
    {
      auto me = w.lock();
      if (!me)
        return;
      me->start_follow();
    });

}

//==============================================================================
void EasyFullControl::EasyCommandHandle::start_follow()
{
  while ((!_remaining_waypoints.empty() ||
    _state == RobotState::MOVING ||
    _state == RobotState::WAITING) && !_stop_follow_thread)
  {
    if (_state == RobotState::IDLE)
    {
      _target_waypoint = _remaining_waypoints[0];
      const auto& target_pose = _target_waypoint.value().position;
      const auto& transformed_pose = _rmf_to_robot_transformer(target_pose);
      RCLCPP_INFO(
        _node->get_logger(),
        "Requesting robot [%s] to navigate to RMF coordinates: [%.2f, %.2f, %.2f] "
        "Robot coordinates: [%.2f, %.2f, %.2f]",
        _robot_name.c_str(),
        target_pose[0], target_pose[1], target_pose[2],
        transformed_pose[0], transformed_pose[1], transformed_pose[2]);

      // The speed limit is set as the minimum of all the approach lanes' limits
      std::optional<double> speed_limit = std::nullopt;
      for (const auto& lane_idx : _target_waypoint.value().approach_lanes)
      {
        const auto& lane = _graph->get_lane(lane_idx);
        const auto& lane_limit = lane.properties().speed_limit();
        if (lane_limit.has_value())
        {
          if (speed_limit.has_value())
            speed_limit = std::min(speed_limit.value(), lane_limit.value());
          else
            speed_limit = lane_limit.value();
        }
      }

      // Send target pose to robot
      Target target;
      target.pose = transformed_pose;
      target.map_name = _map_name;
      target.speed_limit = 0.0;
      _navigation_cb = _navigate(target);

      if (_navigation_cb)
      {
        _remaining_waypoints.erase(_remaining_waypoints.begin());
        _state = RobotState::MOVING;
      }
      else
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "Robot [%s] failed to navigate. Retrying...",
          _robot_name.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }

    else if (_state == RobotState::MOVING)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (_navigation_cb())
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "Robot [%s] has reached its target waypoint",
          _robot_name.c_str());
        _state = RobotState::WAITING;
        if (_target_waypoint.has_value() &&
          _target_waypoint.value().graph_index.has_value())
        {
          _on_waypoint = _target_waypoint.value().graph_index;
          _last_known_waypoint = _on_waypoint;
        }
      }
      else
      {
        _on_lane = get_current_lane();
        if (_on_lane.has_value())
        {
          std::lock_guard<std::mutex> lock(_mutex);
          _on_waypoint = std::nullopt;
        }
        else
        {
          // The robot may either be on the previous or target waypoint
          const auto& last_location =
            _graph->get_waypoint(_last_known_waypoint.value()).get_location();
          const Eigen::Vector3d last_pose =
          {last_location[0], last_location[1], 0.0};
          if (_target_waypoint.has_value() &&
            _target_waypoint.value().graph_index.has_value() &&
            dist(_position, _target_waypoint.value().position) < 0.5)
          {
            _on_waypoint = _target_waypoint.value().graph_index.value();
          }
          else if (_last_known_waypoint.has_value() &&
            dist(_position, last_pose) < 0.5)
          {
            _on_waypoint = _last_known_waypoint.value();
          }
          else
          {
            // The robot is probably off-grid
            std::lock_guard<std::mutex> lock(_mutex);
            _on_waypoint = std::nullopt;
            _on_lane = std::nullopt;
          }
        }

        // Update arrival estimate
        if (_target_waypoint.has_value())
        {
          const auto& target_position = _target_waypoint.value().position;
          const auto& now = std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(_node->get_clock()->now().nanoseconds()));
          const auto& trajectory = rmf_traffic::agv::Interpolate::positions(
            *_traits,
            now,
            {_position, target_position});
          const auto finish_time = trajectory.finish_time();
          if (finish_time)
          {
            _next_arrival_estimator(
              _target_waypoint.value().index,
              *finish_time - now);
          }
        }
      }
    }

    else if (_state == RobotState::WAITING)
    {
      RCLCPP_DEBUG(
        _node->get_logger(),
        "State: Waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      const rmf_traffic::Time& now =
        std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(_node->get_clock()->now().nanoseconds()));

      if (_target_waypoint.has_value())
      {
        const auto& wait_until = _target_waypoint.value().time;
        if (wait_until < now)
        {
          // Robot can move to next waypoint
          _state = RobotState::IDLE;
          RCLCPP_INFO(
            _node->get_logger(),
            "Robot [%s] is done waiting. Remaining waypoints: %d",
            _robot_name.c_str(),
            _remaining_waypoints.size());
        }
        else
        {
          const auto& waiting_duration = wait_until - now;
          RCLCPP_DEBUG(
            _node->get_logger(),
            "Robot [%s] waiting at target for [%.1f]seconds",
            _robot_name.c_str(),
            waiting_duration.count() / 1e9);

          _next_arrival_estimator(
            _target_waypoint.value().index,
            waiting_duration);
        }
      }
      else
      {
        RCLCPP_ERROR(
          _node->get_logger(),
          "State: Waiting but _target_waypoint is nullopt");
      }
    }

    else
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "[%s][start_follow]. Invalid state [%d]. Report this bug.",
        _robot_name.c_str(),
        _state);
      _state = RobotState::IDLE;
    }

  }

  // The robot is done navigating through all the waypoints
  assert(_path_finished_callback);
  _path_finished_callback();
  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] has successfully completed navigating along requested path.",
    _robot_name.c_str());
  _target_waypoint = std::nullopt;
  _path_finished_callback = nullptr;
  _next_arrival_estimator = nullptr;
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::stop()
{
  // Lock mutex
  std::lock_guard<std::mutex> lock(_mutex);

  if (_stop())
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] has stopped.",
      _robot_name.c_str());
  }
  else
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Stop command sent to robot [%s] but it failed to stop.",
      _robot_name.c_str());
  }
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::dock(
  const std::string& dock_name,
  RequestCompleted docking_finished_callback)
{
  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] received a docking request to [%s]...",
    _robot_name.c_str(),
    dock_name.c_str());

  // stop();
  if (_dock_thread.joinable())
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "[stop] _dock_thread present. Calling join");
    _stop_dock_thread = true;
    _dock_thread.join();
  }

  _dock_name = dock_name;
  assert(docking_finished_callback != nullptr);
  _docking_finished_callback = std::move(docking_finished_callback);

  // Get the waypoint that the robot is trying to dock into
  auto dock_waypoint = _graph->find_waypoint(_dock_name);
  assert(dock_waypoint);
  _dock_waypoint_index = dock_waypoint->index();

  std::lock_guard<std::mutex> lock(_mutex);
  _on_waypoint = std::nullopt;
  _on_lane = std::nullopt;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  _stop_dock_thread = false;
  _dock_thread = std::thread(
    [w = weak_from_this()]()
    {
      auto me = w.lock();
      if (!me)
        return;
      me->start_dock();
    });
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::start_dock()
{
  _docking_cb = _dock(_dock_name);

  // Request the robot to start the relevant process
  while (!_docking_cb)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Requesting robot [%s] to dock at [%s]",
      _robot_name.c_str(), _dock_name.c_str());
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Check if requested dock name is valid
  if (_docks.find(_dock_name) == _docks.end())
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Requested dock not found, aborting docking");
    return;
  }

  std::vector<Eigen::Vector3d> positions;
  for (const auto& loc : _docks[_dock_name])
  {
    Eigen::Vector3d wp(loc.x, loc.y, loc.yaw);
    positions.push_back(wp);
  }
  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] is docking...",
    _robot_name.c_str());

  while (!_docking_cb())
  {
    if (positions.empty())
    {
      continue;
    }

    const auto& now = std::chrono::steady_clock::time_point(
      std::chrono::nanoseconds(_node->get_clock()->now().nanoseconds()));
    const auto trajectory = rmf_traffic::agv::Interpolate::positions(
      *_traits,
      now,
      positions);
    if (trajectory.size() < 2)
      return;

    if (auto participant =
      _updater->unstable().get_participant())
    {
      participant->set(
        participant->assign_plan_id(),
        {rmf_traffic::Route{_map_name, trajectory}});
    }

    // Check if we need to abort
    if (_stop_dock_thread)
    {
      RCLCPP_INFO(
        _node->get_logger(),
        "Aborting docking");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  _on_waypoint = _dock_waypoint_index;
  _dock_waypoint_index = std::nullopt;
  _docking_finished_callback();
  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] has completed docking",
    _robot_name.c_str());
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::set_updater(
  RobotUpdateHandlePtr updater)
{
  _updater = std::move(updater);

  // Set the action_executor for the robot
  const auto action_executor =
    [w = weak_from_this()](
    const std::string& category,
    const nlohmann::json& description,
    RobotUpdateHandle::ActionExecution execution)
    {
      // Store the completed callback which will be called when we receive
      // a RobotModeRequest
      const auto self = w.lock();
      if (!self)
        return;

      std::lock_guard<std::mutex> lock(self->_mutex);
      self->_action_waypoint_index = self->_last_known_waypoint;
      self->_on_waypoint = std::nullopt;
      self->_on_lane = std::nullopt;
      self->_action_execution = execution;

      self->set_action_execution(execution);
      self->_action_executor(category, description, execution);
    };

  _updater->set_action_executor(action_executor);
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::update_state()
{
  while (rclcpp::ok())
  {
    // Update position and battery soc
    const auto state = _get_position();
    update_position(state.position, state.map_name);
    update_battery_soc(state.battery_percent/100.0);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::update_position(
  const Eigen::Vector3d& position,
  const std::string& map_name)
{
  if (!_updater)
    return;
  std::lock_guard<std::mutex> lock(_mutex);
  _position = position;
  _map_name = map_name;

  // If robot is on a waypoint
  if (_on_waypoint.has_value())
  {
    const std::size_t& wp = _on_waypoint.value();
    const double& ori = _position[2];
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with waypoint[%d] and orientation [%.2f]",
      _robot_name.c_str(), wp, ori);
    _updater->update_position(wp, ori);
  }
  // If robot is on a lane
  else if (_on_lane.has_value())
  {
    std::vector<std::size_t> lanes = {_on_lane.value()};
    const auto& forward_lane = _graph->get_lane(_on_lane.value());
    const auto& entry_index = forward_lane.entry().waypoint_index();
    const auto& exit_index = forward_lane.exit().waypoint_index();
    const auto reverse_lane = _graph->lane_from(exit_index, entry_index);
    if (reverse_lane)
      lanes.push_back(reverse_lane->index());
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with position [%.2f, %.2f, %.2f] and lane count [%d]",
      _robot_name.c_str(), _position[0], _position[1], _position[2],
      lanes.size());
    _updater->update_position(_position, lanes);
  }
  // If robot is merging into a waypoint
  else if (_target_waypoint.has_value() &&
    _target_waypoint.value().graph_index.has_value())
  {
    const auto& graph_index = _target_waypoint.value().graph_index.value();
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with position [%.2f, %.2f, %.2f] and target waypoint [%d]",
      _robot_name.c_str(), _position[0], _position[1], _position[2],
      graph_index);
    _updater->update_position(_position, graph_index);
  }
  // If robot is docking
  else if (_dock_waypoint_index.has_value())
  {
    const auto& graph_index = _dock_waypoint_index.value();
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with position [%.2f, %.2f, %.2f] and dock waypoint [%d]",
      _robot_name.c_str(), _position[0], _position[1], _position[2],
      graph_index);
    _updater->update_position(_position, graph_index);
  }
  // If robot is performing an action
  else if (_action_waypoint_index.has_value())
  {
    const auto& graph_index = _action_waypoint_index.value();
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with position [%.2f, %.2f, %.2f] and action waypoint [%d]",
      _robot_name.c_str(), _position[0], _position[1], _position[2],
      graph_index);
    _updater->update_position(_position, graph_index);
  }
  // If robot is lost
  else
  {
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with map_name [%s] and position [%.2f, %.2f, %.2f]",
      _robot_name.c_str(),
      _map_name.c_str(), _position[0], _position[1], _position[2]);
    _updater->update_position(_map_name, _position);
  }
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::update_battery_soc(
  double soc)
{
  if (!_updater)
    return;
  std::lock_guard<std::mutex> lock(_mutex);
  _battery_soc = soc;
  if (!_is_charger_set)
  {
    _updater->set_charger_waypoint(_charger_waypoint);
    _updater->maximum_delay(_max_delay);
    _is_charger_set = true;
  }
  RCLCPP_DEBUG(
    _node->get_logger(),
    "[%s] Calling update_battery_soc with [%.2f]",
    _robot_name.c_str(), _battery_soc);
  _updater->update_battery_soc(_battery_soc);
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::newly_closed_lanes(
  const std::unordered_set<std::size_t>& closed_lanes)
{
  bool need_to_replan = false;
  const auto& current_lane = get_current_lane();

  // get_current_lane() already checks that _target_waypoint and approach_lanes are not empty
  if (current_lane.has_value())
  {
    const auto& current_lanes = _target_waypoint.value().approach_lanes;
    for (const auto& l : current_lanes)
    {
      if (closed_lanes.count(l))
      {
        need_to_replan = true;
        // The robot is currently on a lane that has been closed.
        // We take this to mean that the robot needs to reverse.
        if (l == current_lane)
        {
          const auto& lane = _graph->get_lane(l);
          const auto return_waypoint = lane.entry().waypoint_index();
          const auto* reverse_lane =
            _graph->lane_from(lane.entry().waypoint_index(),
              lane.exit().waypoint_index());

          // Lock?
          if (reverse_lane)
          {
            // Update current lane to reverse back to start of the lane
            _on_lane = reverse_lane->index();
          }
          else
          {
            // Update current position and waypoint index to return to
            _updater->update_position(_position, return_waypoint);
          }
        }
      }
    }
  }

  if (!need_to_replan &&
    _target_waypoint.has_value() &&
    _target_waypoint.value().graph_index.has_value())
  {
    // Check if the remainder of the current plan has been invalidated by the
    // lane closure.
    const auto next_index = _target_waypoint.value().graph_index.value();
    for (std::size_t i = next_index; i < _remaining_waypoints.size(); ++i)
    {
      for (const auto& lane : _remaining_waypoints[i].approach_lanes)
      {
        if (closed_lanes.count(lane))
        {
          need_to_replan = true;
          break;
        }
      }

      if (need_to_replan)
        break;
    }
  }

  if (need_to_replan)
    _updater->replan();
}

//==============================================================================
bool EasyFullControl::EasyCommandHandle::initialized()
{
  return _initialized;
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::parse_waypoints(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints)
{
  if (waypoints.empty())
    return;

  std::vector<PlanWaypoint> wps;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    wps.push_back(PlanWaypoint(i, waypoints[i]));

  // We assume the first waypoint is safe for pruning if it is
  // within a threshold of the robot's current position
  double filter_threshold = 0.5;
  auto last_position = _position;
  const auto&  first_position = wps[0].position;
  if (waypoints.size() > 2 &&
    dist(first_position, last_position) < filter_threshold)
  {
    wps.erase(wps.begin());
  }

  std::lock_guard<std::mutex> lock(_mutex);
  _target_waypoint = std::nullopt;
  _remaining_waypoints = std::move(wps);
  return;

}

//==============================================================================
std::optional<std::size_t> EasyFullControl::EasyCommandHandle::get_current_lane()
{
  const auto projection = [](
    const Eigen::Vector2d& current_position,
    const Eigen::Vector2d& target_position,
    const Eigen::Vector2d& lane_entry,
    const Eigen::Vector2d& lane_exit) -> double
    {
      return (current_position - target_position).dot(lane_exit - lane_entry);
    };

  if (!_target_waypoint.has_value())
    return std::nullopt;
  const auto& approach_lanes = _target_waypoint.value().approach_lanes;
  // Empty approach lanes signifies the robot will rotate at the waypoint.
  // Here we rather update that the robot is at the waypoint rather than
  // approaching it.
  if (approach_lanes.empty())
    return std::nullopt;

  for (const auto& lane_index : approach_lanes)
  {
    const auto& lane = _graph->get_lane(lane_index);
    const auto& p0 =
      _graph->get_waypoint(lane.entry().waypoint_index()).get_location();
    const auto& p1 =
      _graph->get_waypoint(lane.exit().waypoint_index()).get_location();
    const auto& p = _position.block<2, 1>(0, 0);
    const bool before_lane = projection(p, p0, p0, p1) < 0.0;
    const bool after_lane = projection(p, p1, p0, p1) >= 0.0;
    if (!before_lane && !after_lane) // the robot is on this lane
      return lane_index;
  }

  return std::nullopt;
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::set_action_execution(
  RobotUpdateHandle::ActionExecution execution)
{
  _action_execution = execution;
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::complete_robot_action()
{
  std::lock_guard<std::mutex> lock(_mutex);

  if (!_action_execution.has_value())
    return;
  _action_execution->finished();
  _action_execution = std::nullopt;
  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] has completed the action it was performing.",
    _robot_name.c_str());
}

//==============================================================================
double EasyFullControl::EasyCommandHandle::dist(
  const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return (a.block<2, 1>(0, 0) - b.block<2, 1>(0, 0)).norm();
}

//==============================================================================
EasyFullControl::EasyFullControl()
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<EasyFullControl> EasyFullControl::make(
  Configuration config,
  const AdapterPtr& adapter)
{
  auto easy_handle = std::shared_ptr<EasyFullControl>(new EasyFullControl);
  easy_handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
    config);

  auto success = easy_handle->_pimpl->initialize_fleet(adapter);
  if (!success)
    return nullptr;

  return easy_handle;
}

//==============================================================================
bool EasyFullControl::add_robot(
  const std::string& robot_name,
  Start pose,
  GetPosition get_position,
  std::function<ProcessCompleted(
    const EasyFullControl::Target target)> navigate,
  std::function<ProcessCompleted(const std::string& dock_name)> dock,
  ProcessCompleted stop,
  RobotUpdateHandle::ActionExecutor action_executor)
{
  // Obtain additional robot config from config
  const YAML::Node robot_config = _pimpl->_fleet_config["robots"][robot_name];
  const double _max_delay =
    robot_config["robot_config"]["max_delay"].as<double>();
  std::optional<rmf_traffic::Duration> max_delay =
    rmf_traffic::time::from_seconds(_max_delay);
  const std::string charger_waypoint =
    robot_config["rmf_config"]["charger"]["waypoint"].as<std::string>();
  const std::size_t charger_waypoint_index = _pimpl->_graph->find_waypoint(
    charger_waypoint)->index();
  const std::string map_name =
    robot_config["rmf_config"]["start"]["map_name"].as<std::string>();

  Planner::StartSet starts;

  // Use Start or compute plan starts
  if (std::holds_alternative<Planner::Start>(pose))
  {
    RCLCPP_INFO(_pimpl->_adapter->node()->get_logger(),
      "Using provided Planner::Start to initialize starts "
      " for robot %s.",
      robot_name.c_str());
    const auto p = std::get<Planner::Start>(pose);
    starts.push_back(p);
  }
  else if (std::holds_alternative<Eigen::Vector3d>(pose))
  {
    RCLCPP_INFO(_pimpl->_adapter->node()->get_logger(),
      "Running compute_plan_starts for robot %s.",
      robot_name.c_str());
    const auto p = std::get<Eigen::Vector3d>(pose);

    RCLCPP_INFO(_pimpl->_adapter->node()->get_logger(),
      "Robot %s pose is [%.2f, %.2f, %.2f]",
      robot_name.c_str(),
      p.x(), p.y(), p.z());

    // Use compute plan starts to estimate the start
    starts = rmf_traffic::agv::compute_plan_starts(
      *_pimpl->_graph, map_name, {p.x(), p.y(), p.z()},
      rmf_traffic_ros2::convert(_pimpl->_adapter->node()->now()));
  }

  if (starts.empty())
  {
    RCLCPP_ERROR(_pimpl->_adapter->node()->get_logger(),
      "Unable to determine StartSet for %s", robot_name.c_str());

    return false;
  }

  // TODO(XY) Set up robot-rmf transformation here
  EasyCommandHandle::Transformer rmf_to_robot_transformer =
    [](Eigen::Vector3d a)
    {
      return a;
    };

  auto initial_position = get_position().position;
  double initial_battery_soc = get_position().battery_percent;

  // Create an EasyCommandHandle for this fella somehow somewhere
  const auto command = std::make_shared<EasyCommandHandle>(
    _pimpl->_adapter->node(),
    _pimpl->_fleet_name,
    robot_name,
    _pimpl->_graph,
    _pimpl->_traits,
    rmf_to_robot_transformer,
    map_name,
    max_delay,
    starts[0],
    initial_position,
    initial_battery_soc,
    charger_waypoint_index,
    get_position,
    navigate,
    dock,
    stop,
    action_executor);

  if (command->initialized())
  {
    // Robot successfully initialized
    // Add robot to fleet
    _pimpl->_fleet_handle->add_robot(
      command, robot_name, _pimpl->_traits->profile(), starts,
      [w = this->weak_from_this(), command, robot_name = std::move(robot_name)](
        const RobotUpdateHandlePtr& updater)
      {
        const auto self = w.lock();
        if (!self)
          return;
        command->set_updater(updater);
        self->_pimpl->_robots[robot_name] = command;
      });

    RCLCPP_INFO(_pimpl->_adapter->node()->get_logger(),
      "Successfully added new robot to %s fleet: %s",
      _pimpl->_fleet_name.c_str(), robot_name.c_str());

    // Start update thread for this robot
    command->start_update_thread();
  }
  else
  {
    RCLCPP_INFO(_pimpl->_adapter->node()->get_logger(),
      "Failed to initialize robot: %s",
      robot_name.c_str());
  }

  return true;

}

//==============================================================================
bool EasyFullControl::Implementation::initialize_fleet(
  const AdapterPtr& adapter)
{
  _adapter = adapter;
  const auto& node = adapter->node();
  _fleet_config = _config.fleet_config();

  const YAML::Node rmf_fleet = _fleet_config["rmf_fleet"];
  _fleet_name = _fleet_config["rmf_fleet"]["name"].as<std::string>();

  _traits = std::make_shared<VehicleTraits>(_config.vehicle_traits());

  _graph = std::make_shared<Graph>(_config.graph());
  std::cout << "The fleet [" << _fleet_name
            << "] has the following named waypoints:\n";
  for (const auto& key : _graph->keys())
    std::cout << " -- " << key.first << std::endl;

  auto _server_uri = _config.server_uri();

  // Add fleet to adapter
  _fleet_handle = adapter->add_fleet(
    _fleet_name, *_traits, *_graph, _server_uri);

  _closed_lanes_pub =
    node->create_publisher<rmf_fleet_msgs::msg::ClosedLanes>(
    rmf_fleet_adapter::ClosedLaneTopicName,
    rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local());

  // Create subscription to lane closure requests
  _lane_closure_request_sub =
    node->create_subscription<rmf_fleet_msgs::msg::LaneRequest>(
    rmf_fleet_adapter::LaneClosureRequestTopicName,
    rclcpp::SystemDefaultsQoS(),
    [this](
      rmf_fleet_msgs::msg::LaneRequest::UniquePtr request_msg)
    {

      if (request_msg->fleet_name != this->_fleet_name &&
      !request_msg->fleet_name.empty())
        return;

      this->_fleet_handle->open_lanes(request_msg->open_lanes);
      this->_fleet_handle->close_lanes(request_msg->close_lanes);

      std::unordered_set<std::size_t> newly_closed_lanes;
      for (const auto& l : request_msg->close_lanes)
      {
        if (_closed_lanes.count(l) == 0)
          newly_closed_lanes.insert(l);

        _closed_lanes.insert(l);
      }

      for (const auto& l : request_msg->open_lanes)
        _closed_lanes.erase(l);

      for (auto& [_, robot] : this->_robots)
      {
        robot->newly_closed_lanes(newly_closed_lanes);
      }

      rmf_fleet_msgs::msg::ClosedLanes state_msg;
      state_msg.fleet_name = this->_fleet_name;
      state_msg.closed_lanes.insert(
        state_msg.closed_lanes.begin(),
        _closed_lanes.begin(),
        _closed_lanes.end());

      _closed_lanes_pub->publish(state_msg);
    });

  // Set the fleet state topic publish period
  const double fleet_state_frequency =
    rmf_fleet["publish_fleet_state"].as<double>();
  _fleet_handle->fleet_state_topic_publish_period(
    rmf_traffic::time::from_seconds(1.0/fleet_state_frequency));

  // Set up parameters required for task planner
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
  const double recharge_threshold =
    rmf_fleet["recharge_threshold"].as<double>();
  // Recharge state of charge
  const double recharge_soc = rmf_fleet["recharge_soc"].as<double>();

  // Finishing tasks
  const YAML::Node task_capabilities = rmf_fleet["task_capabilities"];
  const std::string finishing_request_string =
    task_capabilities["finishing_request"].as<std::string>();
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
  if (!_fleet_handle->set_task_planner_params(
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

  // Currently accepting any tasks as long as they are in the config
  const auto consider =
    [](const nlohmann::json& description,
      rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      confirm.accept();
    };

  const bool perform_loop = task_capabilities["loop"].as<bool>();
  if (perform_loop)
  {
    _fleet_handle->consider_patrol_requests(consider);
  }

  const bool perform_delivery = task_capabilities["delivery"].as<bool>();
  if (perform_delivery)
  {
    _fleet_handle->consider_delivery_requests(
      consider, consider);
  }

  const bool perform_cleaning = task_capabilities["clean"].as<bool>();
  if (perform_cleaning)
  {
    _fleet_handle->consider_cleaning_requests(consider);
  }

  // Currently accepting any actions as long as they are in the config
  if (task_capabilities["action"])
  {
    std::vector<std::string> action_strings =
      task_capabilities["action"].as<std::vector<std::string>>();
    for (const auto& action : action_strings)
    {
      _fleet_handle->add_performable_action(action, consider);
    }
  }

  return true;
}


} // namespace agv
} // namespace rmf_fleet_adapter