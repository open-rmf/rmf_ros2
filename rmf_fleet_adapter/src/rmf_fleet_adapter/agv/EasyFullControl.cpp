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
#include <rmf_fleet_adapter/agv/Adapter.hpp>

#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/closed_lanes.hpp>
#include <rmf_fleet_msgs/msg/interrupt_request.hpp>
#include <rmf_fleet_msgs/msg/dock_summary.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <thread>

namespace rmf_fleet_adapter {
namespace agv {


//==============================================================================
class EasyFullControl::Configuration::Implementation
{
public:
  const std::string& fleet_name;
  rmf_traffic::agv::VehicleTraits traits;
  rmf_traffic::agv::Graph graph;
  std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system;
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink;
  double recharge_threshold;
  double recharge_soc;
  bool account_for_battery_drain;
  std::vector<std::string> action_categories;
  rmf_task::ConstRequestFactoryPtr finishing_request;
  std::optional<std::string> server_uri;
  rmf_traffic::Duration max_delay;
};

//==============================================================================
EasyFullControl::Configuration::Configuration(
  const std::string& fleet_name,
  rmf_traffic::agv::VehicleTraits traits,
  rmf_traffic::agv::Graph graph,
  std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink,
  double recharge_threshold,
  double recharge_soc,
  bool account_for_battery_drain,
  std::vector<std::string> action_categories,
  rmf_task::ConstRequestFactoryPtr finishing_request,
  std::optional<std::string> server_uri,
  rmf_traffic::Duration max_delay)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(fleet_name),
        std::move(traits),
        std::move(graph),
        std::move(battery_system),
        std::move(motion_sink),
        std::move(ambient_sink),
        std::move(tool_sink),
        std::move(recharge_threshold),
        std::move(recharge_soc),
        std::move(account_for_battery_drain),
        std::move(action_categories),
        std::move(finishing_request),
        std::move(server_uri),
        std::move(max_delay)
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& EasyFullControl::Configuration::fleet_name() const
{
  return _pimpl->fleet_name;
}

//==============================================================================
auto EasyFullControl::Configuration::graph() const -> const Graph&
{
  return _pimpl->graph;
}

//==============================================================================
auto EasyFullControl::Configuration::vehicle_traits() const
-> const VehicleTraits&
{
  return _pimpl->traits;
}

//==============================================================================
std::optional<std::string> EasyFullControl::Configuration::server_uri() const
{
  return _pimpl->server_uri;
}

std::shared_ptr<rmf_battery::agv::BatterySystem>
EasyFullControl::Configuration::battery_system() const
{
  return _pimpl->battery_system;
}

std::shared_ptr<rmf_battery::MotionPowerSink>
EasyFullControl::Configuration::motion_sink() const
{
  return _pimpl->motion_sink;
}

//==============================================================================
std::shared_ptr<rmf_battery::DevicePowerSink>
EasyFullControl::Configuration::ambient_sink() const
{
  return _pimpl->ambient_sink;
}

//==============================================================================
std::shared_ptr<rmf_battery::DevicePowerSink>
EasyFullControl::Configuration::tool_sink() const
{
  return _pimpl->tool_sink;
}

//==============================================================================
double EasyFullControl::Configuration::recharge_threshold() const
{
  return _pimpl->recharge_threshold;
}

//==============================================================================
double EasyFullControl::Configuration::recharge_soc() const
{
  return _pimpl->recharge_soc;
}

//==============================================================================
bool EasyFullControl::Configuration::account_for_battery_drain() const
{
  return _pimpl->account_for_battery_drain;
}

//==============================================================================
rmf_task::ConstRequestFactoryPtr
EasyFullControl::Configuration::finishing_request() const
{
  return _pimpl->finishing_request;
}

//==============================================================================
rmf_traffic::Duration EasyFullControl::Configuration::max_delay() const
{
  return _pimpl->max_delay;
}

//==============================================================================
std::vector<std::string>
EasyFullControl::Configuration::action_categories() const
{
  return _pimpl->action_categories();
}

//==============================================================================
class EasyFullControl::RobotState::Implementation
{
public:
  std::string name;
  std::string charger_name;
  std::string map_name;
  Eigen::Vector3d location;
  battery_soc;
}

//==============================================================================
EasyFullControl::RobotState::RobotState(
  const std::string& name,
  const std::string& charger_name,
  const std::string& map_name,
  Eigen::Vector3d location,
  double battery_soc)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(name),
        std::move(charger_name),
        std::move(map_name),
        std::move(location),
        std::move(battery_soc)
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& EasyFullControl::RobotState::name() const
{
  return _pimpl->name;
}

//==============================================================================
const std::string& EasyFullControl::RobotState::charger_name() const
{
  return _pimpl->charger_name;
}

//==============================================================================
const std::string& EasyFullControl::RobotState::map_name() const
{
  return _pimpl->map_name;
}

//==============================================================================
const Eigen::Vector3d& EasyFullControl::RobotState::location() const
{
  return _pimpl->location;
}

//==============================================================================
const double EasyFullControl::RobotState::battery_soc() const
{
  return _pimpl->battery_soc;
}

//==============================================================================
namespace {
class EasyCommandHandle : public RobotCommandHandle
{
public:
  using Planner = rmf_traffic::agv::Planner;
  using RobotState = EasyCommandHandle::RobotState;
  using Graph = rmf_traffic::agv::Graph;
  using VehicleTraits = rmf_traffic::agv::VehicleTraits;
  using ActionExecutor = RobotUpdateHandle::ActionExecution;
  using GetStateCallback = EasyFullControl::GetStateCallback;
  using GoalCompletedCallback = EasyFullControl::GoalCompletedCallback;
  using NavigationRequest = EasyFullControl::NavigationRequest;
  using StopRequest = EasyFullControl::StopRequest;
  using DockRequest = EasyFullControl::DockRequest;

  EasyCommandHandle(
    rclcpp::Node::SharedPtr node,
    const std::string& robot_name,
    RobotState start_state,
    GetStateCallback get_state,
    NavigationRequest handle_nav_request,
    StopRequest handle_stop,
    DockRequest handle_dock,
    ActionExecutor action_executor
  );

  void stop() final;

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) final;

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final;

  void set_updater(rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater);

  // Variables
  rclcpp::Node::SharedPtr node;
  const std::string& robot_name;
  GetStateCallback get_state;
  NavigationRequest handle_nav_request;
  StopRequest handle_stop;
  DockRequest handle_dock;
  ActionExecutor action_executor;

  bool _filter_waypoints;
  double _filter_threshold;
  rmf_fleet_adapter::agv::RobotUpdateHandlePtr _updater;
  bool _is_charger_set;
  RobotState _state;

  std::optional<std::size_t> _on_waypoint = std::nullopt;
  std::optional<std::size_t> _last_known_waypoint = std::nullopt;
  std::optional<std::size_t> _on_lane = std::nullopt;

  std::mutex _mutex;

  std::optional<PlanWaypoint> _target_waypoint;
  std::vector<PlanWaypoint> _remaining_waypoints;

  std::thread _follow_thread;
  std::atomic_bool _stop_follow_thread;
  std::atomic_bool _navigation_completed;
  GoalHandlePtr _goal_handle; // Result of std::future<GoalHandlePtr>::get();
  RequestCompleted _path_finished_callback;
  ArrivalEstimator _next_arrival_estimator;
};




} // anonymous namespace

//==============================================================================
using EasyCommandHandlePtr = std::shared_ptr<EasyCommandHandle>;
class EasyFullControl::Implementation
{
public:
  std::string fleet_name;
  std::shared_ptr<VehicleTraits> traits;
  std::shared_ptr<Graph> graph;
  std::shared_ptr<Adapter> adapter;
  std::shared_ptr<FleetUpdateHandle> fleet_handle;
  // Map robot name to its EasyCommandHandle
  std::unordered_map<std::string, EasyCommandHandlePtr> cmd_handles;
};

//==============================================================================
EasyFullControl::EasyFullControl()
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<EasyFullControl> EasyFullControl::make(
  Configuration config,
  const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
  std::optional<rmf_traffic::Duration> discovery_timeout = std::nullopt)
{
  auto easy_adapter = std::shared_ptr<EasyFullControl>(new EasyFullControl);
  easy_adapter->_pimpl = rmf_utils::make_unique_impl<Implementation>();

  _pimpl->fleet_name = config.fleet_name();
  _pimpl->traits = std::make_shared<VehicleTraits>(config.traits());
  _pimpl->graph = std::make_shared<Graph>(config.graph());
  _pimpl->adapter = Adapter::make(
    fleet_name + "_fleet_adapter",
    options,
    std::move(discovery_timeout)
  );

  if (!_pimpl->adapter)
  {
    return nullptr;
  }

  const auto node = _pimpl->adapter->node();
  // Create a FleetUpdateHandle
  _pimpl->fleet_handle = _pimpl->adapter->add_fleet(
    _pimpl->fleet_name,
    _pimpl->config.traits(),
    _pimpl->config.graph(),
    _pimpl->config.server_uri()
  );

  bool ok = _pimpl->fleet_handle->set_task_planner_params(
    config.battery_system(),
    config.motion_sink(),
    config.ambient_sink(),
    config.tool_sink(),
    config.recharge_threshold(),
    config.recharge_soc(),
    config.account_for_battery_drain(),
    config.finishing_request(),
  );
  if (ok)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Initialized task planner parameters."
    );
  }
  else
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Failed to initialize task planner parameters. This fleet will not "
      "respond to bid requests for tasks"
    );
  }

  // TODO(YV): Make an API available to specify what tasks this fleet can perform
  auto consider_all =
    [](const nlohmann::json& description, Confirmation& confirm)
    {
      confirm.accept();
    }
  _pimpl->fleet_handle->consider_delivery_requests(
    consider_all, consider_all);
  _pimpl->fleet_handle->consider_cleaning_requests(consider_all);
  _pimpl->fleet_handle->consider_patrol_requests(consider_all);
  _pimpl->fleet_handle->consider_composed_requests(consider_all);
  for (const std::string& action : config.action_categories())
  {
    _pimpl->fleet_handle->add_performable_action(action, consider_all);
  }

  _pimpl->fleet_handle->default_maximum_delay(config.max_delay());

  RCLCPP_INFO(
    node->get_logger(),
    "Successfully initialized Full Control adapter for fleet [%s]",
    fleet_name.c_str()
  );

  return easy_handle;
}

//==============================================================================
std::shared_ptr<rclcpp::Node> EasyFullControl::node()
{
  return _pimpl->adapter->node();
}

//==============================================================================
std::shared_ptr<FleetUpdateHandle> EasyFullControl::fleet_handle()
{
  return _pimpl->adapter->node();
}

//==============================================================================
EasyFullControl& EasyFullControl::start()
{
  _pimpl->adapter->start();
}

//==============================================================================
EasyFullControl& EasyFullControl::stop()
{
  _pimpl->adapter->stop();
}

//==============================================================================
EasyFullControl& EasyFullControl::wait()
{
  _pimpl->adapter->wait();
}

//==============================================================================
auto EasyFullControl::add_robot(
  RobotState start_state,
  GetStateCallback get_state,
  NavigationRequest handle_nav_request,
  StopRequest handle_stop,
  DockRequest handle_dock,
  ActionExecutor action_executor) -> bool
{
  const auto& robot_name = start_state.name();
  const auto node = _pimpl->adapter->node();
  RCLCPP_INFO(
    this->get_logger(),
    "Adding robot [%s] to the fleet.", robot_name.c_str()
  );
  auto insertion = _pimpl->cmd_handles.insert({robot_name, nullptr});
  if (!insertion.second)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Robot [%s] was previously added to the fleet. Ignoring request...",
      robot_name.c_str()
    );
    return false;
  }

  rmf_traffic::Time now = std::chrono::steady_clock::time_point(
    std::chrono::nanoseconds(node->now().nanoseconds()));

  // TODO(YV): Get these constants from EasyCommandHandle::Configuration
  const double max_merge_waypoint_distance = 0.3;
  const double max_merge_lane_distance = 1.0;
  const double min_lane_length = 1e-8;
  auto starts = rmf_traffic::agv::compute_plan_starts(
    *(_pimpl->graph),
    start_state.map_name(),
    start_state.location(),
    std::move(now),
    max_merge_waypoint_distance,
    max_merge_lane_distance,
    min_lane_length
  );

  if (starts.empty())
  {
    const auto& loc = start_state.location();
    RCLCPP_ERROR(
      node->get_logger(),
      "Unable to compute a StartSet for robot [%s] using level_name [%s] and "
      "location [%.3f, %.3f, %.3f] specified in the RobotState param. This can "
      "happen if the level_name in RobotState does not match any "
      "of the map names in the navigation graph supplied or if the location "
      "reported in the RobotState is far way from the navigation "
      "graph. This robot will not be added to the fleet.",
      robot_name.c_str(),
      map_name.c_str(),
      loc[0], loc[1], loc[2]
    );
    return false;
  }

  insertion.first->second = std::make_shared<EasyCommandHandle>(
    node,
    robot_name,
    std::move(start_state),
    std::move(get_state),
    std::move(handle_nav_request),
    std::move(handle_stop),
    std::move(handle_dock),
    std::move(action_executor)
  );

  _pimpl->fleet_handle->add_robot(
    insertion.first->second,
    robot_name,
    _pimpl->traits->profile(),
    std::move(starts),
    [cmd_handle = insertion.first->second](
      const const RobotUpdateHandlePtr& updater)
    {
      cmd_handle->set_updater(updater);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Successfully added robot [%s] to the fleet.", robot_name.c_str()
  );
}

class EasyFullControl::Implementation
{
public:

  using Transformer = std::function<Eigen::Vector3d(Eigen::Vector3d)>;

  Implementation(
    Configuration config)
  : _config{std::move(config)}
  {
    // Do nothing
  }

  bool initialize_fleet(const AdapterPtr& adapter);

  const Configuration _config;
  AdapterPtr _adapter;
  std::string _fleet_name;
  YAML::Node _fleet_config;
  FleetUpdateHandlePtr _fleet_handle;
  std::shared_ptr<Graph> _graph;
  std::shared_ptr<VehicleTraits> _traits;

  double _max_delay;
  std::string _charger_waypoint;
  std::string _map_name;
  Transformer _rmf_to_robot_transformer;

  rclcpp::Publisher<rmf_fleet_msgs::msg::ClosedLanes>::SharedPtr
    _closed_lanes_pub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::LaneRequest>::SharedPtr
    _lane_closure_request_sub;
  std::unordered_set<std::size_t> _closed_lanes;
  std::unordered_map<std::string, EasyCommandHandlePtr> _robots;
};

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
  double lane_merge_distance,
  const Planner::Start& start,
  const Eigen::Vector3d& initial_position,
  double initial_battery_soc,
  std::size_t charger_waypoint,
  GetPosition get_position,
  std::function<ProcessCompleted(const Target target)> navigate,
  std::function<ProcessCompleted(
    const std::string& dock_name, std::size_t cmd_id)> dock,
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
  _lane_merge_distance(lane_merge_distance),
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
  _quit_follow_thread = false;

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
    _quit_follow_thread = true;
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
  if (_debug)
  {
    auto plan_id = _updater->unstable().current_plan_id();
    RCLCPP_INFO(
      _node->get_logger(),
      "follow_new_path for %s with PlanId %d",
      _robot_name.c_str(), plan_id);
  }

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

  interrupt();
  if (_follow_thread.joinable())
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "[stop] _follow_thread present. Calling join");
    _quit_follow_thread = true;
    _follow_thread.join();
  }
  _quit_follow_thread = false;
  clear();

  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] received a new path to follow...",
    _robot_name.c_str());

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
    _state == RobotState::MOVING) && !_quit_follow_thread)
  {
    // Save the current_cmd_id before checking if we need to
    // abort. We should always be told to abort before the
    // current_cmd_id gets modified, so whatever the value of
    // current_cmd_id is before being told to abort will be the
    // value that we want. If we are saving the wrong value
    // here, then the next thing we will be told to do is abort.
    const auto cmd_id = _current_cmd_id;
    // Check if we need to abort
    if (_quit_follow_thread)
    {
      RCLCPP_INFO(
        _node->get_logger(),
        "%s aborting path request",
        _robot_name.c_str());
      return;
    }

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
      target.cmd_id = next_cmd_id();
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
      const auto need_to_replan = _get_position().replan;
      if (need_to_replan.has_value() && need_to_replan.value())
        replan();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::lock_guard<std::mutex> lock(_mutex);

      // Check if we reached the target
      if (_navigation_cb(cmd_id))
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "Robot [%s] has reached its target waypoint for cmd_id %d",
          _robot_name.c_str(), cmd_id);
        _state = RobotState::IDLE;
        if (_target_waypoint.has_value() &&
          _target_waypoint.value().graph_index.has_value())
        {
          _on_waypoint = _target_waypoint.value().graph_index;
          _last_known_waypoint = _on_waypoint;
        }
        else
        {
          _on_waypoint = std::nullopt; // still on a lane
        }
      }
      else
      {
        // Update the lane the robot is on
        _on_lane = get_current_lane();
        if (_on_lane.has_value())
        {
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
  }

  if (_remaining_waypoints.empty() && _state == RobotState::IDLE)
  {
    // The robot is done navigating through all the waypoints
    assert(_path_finished_callback);
    _path_finished_callback();
    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] has successfully completed navigating along requested path.",
      _robot_name.c_str());
    clear();
    _path_finished_callback = nullptr;
    _next_arrival_estimator = nullptr;
  }
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::stop()
{
  if (_debug)
  {
    auto plan_id = _updater->unstable().current_plan_id();
    RCLCPP_INFO(
      _node->get_logger(),
      "Stop for %s with PlanId %d",
      _robot_name.c_str(), plan_id);
  }

  interrupt();
  // Stop the robot. Tracking variables should remain unchanged.
  std::lock_guard<std::mutex> lock(_mutex);

  _quit_stop_thread = false;
  _stop_thread = std::thread(
    [w = weak_from_this()]()
    {
      auto me = w.lock();
      if (!me)
        return;
      me->stop_robot();
    }
  );
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::stop_robot()
{
  while (!_quit_stop_thread)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Requesting %s to stop...",
      _robot_name.c_str());

    if (_stop(next_cmd_id()))
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::replan()
{
  if (!_updater)
    return;

  const auto& now = std::chrono::steady_clock::time_point(
    std::chrono::nanoseconds(_node->get_clock()->now().nanoseconds()));
  if (_last_replan_time.has_value())
  {
    // TODO: Make the 15s replan cooldown configurable
    auto current_cooldown =
      std::chrono::duration_cast<std::chrono::seconds>(
      now - _last_replan_time.value()).count();
    if (current_cooldown < 15.0)
      return;
  }

  _last_replan_time = now;
  _updater->replan();

  RCLCPP_INFO(
    _node->get_logger(),
    "Requesting replan for %s because of an obstacle",
    _robot_name.c_str());
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

  interrupt();
  if (_dock_thread.joinable())
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "[stop] _dock_thread present. Calling join");
    _quit_dock_thread = true;
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

  _quit_dock_thread = false;
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
  const auto cmd_id = next_cmd_id();
  _docking_cb = _dock(_dock_name, cmd_id);

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
    // TODO: This should open an issue ticket for the robot
    // to tell the operator that the robot cannot proceed
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
    "Robot [%s] is docking at %s...",
    _robot_name.c_str(), _dock_name.c_str());

  while (!_docking_cb(cmd_id))
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
    if (_quit_dock_thread)
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
    // TODO(XY): configure default max_merge_waypoint_distance
    _updater->update_position(_map_name, _position, 0.1, _lane_merge_distance);
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
std::size_t EasyFullControl::EasyCommandHandle::next_cmd_id()
{
  _current_cmd_id++;
  if (_debug)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Issuing cmd_id for %s: %d",
      _robot_name.c_str(), _current_cmd_id);
  }
  return _current_cmd_id;
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::clear()
{
  _target_waypoint = std::nullopt;
  _remaining_waypoints.clear();
  _state = RobotState::IDLE;
}

//==============================================================================
void EasyFullControl::EasyCommandHandle::interrupt()
{
  if (_debug)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Interrupting %s (latest cmd_id is %d)",
      _robot_name.c_str(), _current_cmd_id);
  }

  _quit_follow_thread = true;
  _quit_dock_thread = true;
  _quit_stop_thread = true;

  if (_follow_thread.joinable())
    _follow_thread.join();
  if (_dock_thread.joinable())
    _dock_thread.join();
  if (_stop_thread.joinable())
    _stop_thread.join();
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

  // If the robot is already in the middle of two waypoints, then we can
  // truncate all the waypoints that come before it.
  auto begin_at_index = 0;
  Eigen::Vector2d p(_position.x(), _position.y());
  for (int i = wps.size() - 1; i >= 0; i--)
  {
    int i0, i1;
    i0 = i;
    i1 = i + 1;
    Eigen::Vector2d p0(wps[i0].position.x(), wps[i0].position.y());
    Eigen::Vector2d p1(wps[i1].position.x(), wps[i1].position.y());
    auto dp_lane = p1 - p0;
    double lane_length = dp_lane.norm();
    if (lane_length < 1e-3)
      continue;
    auto n_lane = dp_lane / lane_length;
    auto p_l = p - p0;
    double p_l_proj = p_l.dot(n_lane);

    if (lane_length < p_l_proj)
    {
      // Check if the robot's position is close enough to the lane
      // endpoint to merge it
      if ((p - p1).norm() <= _lane_merge_distance)
      {
        begin_at_index = i1;
        break;
      }
      // Otherwise, continue to the next lane because the robot is not
      // between the lane endpoints
      continue;
    }
    if (p_l_proj < 0.0)
    {
      // Check if the robot's position is close enough to the lane
      // start point to merge it
      if ((p - p0).norm() <= _lane_merge_distance)
      {
        begin_at_index = i0;
        break;
      }
      // Otherwise, continue to the next lane because the robot is not
      // between the lane endpoints
      continue;
    }

    double lane_dist = (p_l - p_l_proj * n_lane).norm();
    if (lane_dist <= _lane_merge_distance)
    {
      begin_at_index = i1;
      break;
    }
  }

  if (begin_at_index > 0)
    wps.erase(wps.begin(), wps.begin() + begin_at_index);

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
bool EasyFullControl::add_robot(
  const std::string& robot_name,
  Start pose,
  GetPosition get_position,
  std::function<ProcessCompleted(
    const EasyFullControl::Target target)> navigate,
  std::function<ProcessCompleted(
    const std::string& dock_name, std::size_t cmd_id)> dock,
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
  double lane_merge_distance = 0.1;
  const YAML::Node rmf_config = _pimpl->_fleet_config["rmf_fleet"];
  if (rmf_config["lane_merge_distance"])
    lane_merge_distance = rmf_config["lane_merge_distance"].as<double>();

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

  auto initial_position = get_position().position;
  double initial_battery_soc = get_position().battery_percent;

  // Create an EasyCommandHandle for this fella somehow somewhere
  const auto command = std::make_shared<EasyCommandHandle>(
    _pimpl->_adapter->node(),
    _pimpl->_fleet_name,
    robot_name,
    _pimpl->_graph,
    _pimpl->_traits,
    _pimpl->_rmf_to_robot_transformer,
    map_name,
    max_delay,
    lane_merge_distance,
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

  // Set up robot-rmf transformer if any
  if (_fleet_config["reference_coordinates"])
  {
    const YAML::Node ref_coord = _fleet_config["reference_coordinates"];
    const std::vector<std::vector<double>> rmf_coord =
      ref_coord["rmf"].as<std::vector<std::vector<double>>>();
    const std::vector<std::vector<double>> robot_coord =
      ref_coord["robot"].as<std::vector<std::vector<double>>>();

    // Calculate based on first two sets of points provided
    Eigen::Vector3d rmf0(rmf_coord[0][0], rmf_coord[0][1], 0.0);
    Eigen::Vector3d robot0(robot_coord[0][0], robot_coord[0][1], 0.0);
    Eigen::Vector3d rmf1(rmf_coord[1][0], rmf_coord[1][1], 0.0);
    Eigen::Vector3d robot1(robot_coord[1][0], robot_coord[1][1], 0.0);

    // Find scale
    auto scale = robot0.norm() / rmf0.norm();

    // Apply scale to rmf0 and rmf1 coords
    rmf0 = scale * rmf0;
    rmf1 = scale * rmf1;

    // Find rotation matrix
    Eigen::Vector3d rmf_u_vector, robot_u_vector;
    rmf_u_vector = (rmf1 - rmf0).normalized();
    robot_u_vector = (robot1 - robot0).normalized();
    Eigen::Matrix3d R = Eigen::Quaterniond().setFromTwoVectors(
      rmf_u_vector, robot_u_vector).toRotationMatrix();

    // Find translation vector
    rmf0 = R * rmf0;  // new rotated origin
    auto translation = robot0 - rmf0;

    _rmf_to_robot_transformer =
      [scale, R, translation](Eigen::Vector3d rmf_coord)
      {
        Eigen::Vector3d robot_coord;
        // Apply scale
        robot_coord = scale * rmf_coord;
        // Apply rotation matrix
        robot_coord = R * robot_coord;
        // Apply translation
        robot_coord = robot_coord + translation;
        return robot_coord;
      };
  }
  else
  {
    // No reference coordinates provided, just return the same coordinate
    _rmf_to_robot_transformer =
      [](Eigen::Vector3d rmf_coord)
      {
        return rmf_coord;
      };
  }

  return true;
}


} // namespace agv
} // namespace rmf_fleet_adapter
