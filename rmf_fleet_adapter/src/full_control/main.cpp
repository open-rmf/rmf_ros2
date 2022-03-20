/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

// Internal implementation-specific headers
#include "../rmf_fleet_adapter/ParseArgs.hpp"
#include "../rmf_fleet_adapter/load_param.hpp"

// Public rmf_fleet_adapter API headers
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>

// Standard topic names for communicating with fleet drivers
#include <rmf_fleet_adapter/StandardNames.hpp>

// Fleet driver state/command messages
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/srv/lift_clearance.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/closed_lanes.hpp>
#include <rmf_fleet_msgs/msg/interrupt_request.hpp>

// RMF Task messages
#include <rmf_task_msgs/msg/task_type.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic_ros2/Time.hpp>

// Utility functions for estimating where a robot is on the graph based on
// the information provided by fleet drivers.
#include "../rmf_fleet_adapter/estimation.hpp"

// Public rmf_task API headers
#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ParkRobotFactory.hpp>

// Public rmf_traffic API headers
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/Route.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <unordered_set>
#include <optional>

//==============================================================================
rmf_fleet_adapter::agv::RobotUpdateHandle::Unstable::Decision
convert_decision(uint32_t decision)
{
  using namespace rmf_fleet_adapter::agv;
  switch (decision)
  {
    case rmf_fleet_msgs::srv::LiftClearance::Response::DECISION_CLEAR:
      return RobotUpdateHandle::Unstable::Decision::Clear;
    case rmf_fleet_msgs::srv::LiftClearance::Response::DECISION_CROWDED:
      return RobotUpdateHandle::Unstable::Decision::Crowded;
  }

  std::cerr << "Received undefined value for lift clearance service: "
            << decision << std::endl;
  return RobotUpdateHandle::Unstable::Decision::Undefined;
}

//==============================================================================
struct DistanceFromGraph
{
  double value;
  std::size_t index;

  enum Type
  {
    Waypoint = 0,
    Lane
  };
  Type type;
};

//==============================================================================
std::optional<DistanceFromGraph> distance_from_graph(
  const rmf_fleet_msgs::msg::Location& l,
  const rmf_traffic::agv::Graph& graph)
{
  std::optional<DistanceFromGraph> output;
  const Eigen::Vector2d p = {l.x, l.y};
  const std::string& map = l.level_name;

  for (std::size_t i = 0; i < graph.num_waypoints(); ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    if (wp.get_map_name() != map)
      continue;

    const double dist = (wp.get_location() - p).norm();
    if (!output.has_value() || (dist < output->value))
    {
      output = DistanceFromGraph{dist, i, DistanceFromGraph::Waypoint};
    }
  }

  for (std::size_t i = 0; i < graph.num_lanes(); ++i)
  {
    const auto& lane = graph.get_lane(i);
    const auto& wp0 = graph.get_waypoint(lane.entry().waypoint_index());
    const auto& wp1 = graph.get_waypoint(lane.exit().waypoint_index());

    if (map != wp0.get_map_name() && map != wp1.get_map_name())
      continue;

    const Eigen::Vector2d p0 = wp0.get_location();
    const Eigen::Vector2d p1 = wp1.get_location();

    const Eigen::Vector2d dp = p - p0;
    const Eigen::Vector2d dp1 = p1 - p0;

    const double lane_length = dp1.norm();
    if (lane_length < 1e-8)
      continue;

    const double u = dp.dot(dp1)/lane_length;
    if (u < 0 || lane_length < u)
      continue;

    const double dist = (dp - u*dp1/lane_length).norm();
    if (!output.has_value() || (dist < output->value))
    {
      output = DistanceFromGraph{dist, i, DistanceFromGraph::Lane};
    }
  }

  return output;
}

//==============================================================================
class FleetDriverRobotCommandHandle
  : public rmf_fleet_adapter::agv::RobotCommandHandle,
  public std::enable_shared_from_this<FleetDriverRobotCommandHandle>
{
public:

  using PathRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr;

  using ModeRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr;

  using ActionExecution =
    rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution;

  FleetDriverRobotCommandHandle(
    rclcpp::Node& node,
    std::string fleet_name,
    std::string robot_name,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits,
    PathRequestPub path_request_pub,
    ModeRequestPub mode_request_pub)
  : _node(&node),
    _path_request_pub(std::move(path_request_pub)),
    _mode_request_pub(std::move(mode_request_pub))
  {
    _current_path_request.fleet_name = fleet_name;
    _current_path_request.robot_name = robot_name;

    _current_dock_request.fleet_name = fleet_name;
    _current_dock_request.robot_name = robot_name;
    _current_dock_request.mode.mode = _current_dock_request.mode.MODE_DOCKING;

    rmf_fleet_msgs::msg::ModeParameter p;
    p.name = "docking";
    _current_dock_request.parameters.push_back(std::move(p));

    _travel_info.graph = std::move(graph);
    _travel_info.traits = std::move(traits);
    _travel_info.fleet_name = std::move(fleet_name);
    _travel_info.robot_name = std::move(robot_name);
  }

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) final
  {
    auto lock = _lock();
    _clear_last_command();

    _travel_info.target_plan_index = std::nullopt;
    _travel_info.waypoints = waypoints;
    _travel_info.next_arrival_estimator = std::move(next_arrival_estimator);
    _travel_info.path_finished_callback = std::move(path_finished_callback);
    _interrupted = false;

    _current_path_request.task_id = std::to_string(++_current_task_id);
    _current_path_request.path.clear();
    for (const auto& wp : waypoints)
    {
      rmf_fleet_msgs::msg::Location location;
      const Eigen::Vector3d p = wp.position();
      location.t = rmf_traffic_ros2::convert(wp.time());
      location.x = p.x();
      location.y = p.y();
      location.yaw = p.z();

      // The speed limit is set as the minimum of all the approach lanes' limits
      std::optional<double> speed_limit = std::nullopt;
      for (const auto& lane_idx : wp.approach_lanes())
      {
        const auto& lane = _travel_info.graph->get_lane(lane_idx);
        const auto& lane_limit = lane.properties().speed_limit();
        if (lane_limit.has_value())
        {
          if (speed_limit.has_value())
            speed_limit = std::min(speed_limit.value(), lane_limit.value());
          else
            speed_limit = lane_limit.value();
        }
      }
      if (speed_limit.has_value())
      {
        location.obey_approach_speed_limit = true;
        location.approach_speed_limit = speed_limit.value();
      }
      else
      {
        location.obey_approach_speed_limit = false;
      }

      // Note: if the waypoint is not on a graph index, then we'll just leave
      // the level_name blank. That information isn't likely to get used by the
      // fleet driver anyway.
      if (wp.graph_index())
      {
        location.level_name =
          _travel_info.graph->get_waypoint(*wp.graph_index()).get_map_name();
      }

      _current_path_request.path.emplace_back(std::move(location));
    }

    _path_requested_time = std::chrono::steady_clock::now();
    _path_request_pub->publish(_current_path_request);
  }

  void stop() final
  {
    auto lock = _lock();
    _clear_last_command();

    if (!_last_known_state.has_value())
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "Cannot ask slotcar [%s] to stop because we have never received a "
        "state from it.",
        _travel_info.robot_name.c_str());
      return;
    }

    rmf_fleet_msgs::msg::Location location;
    _current_path_request.task_id = std::to_string(++_current_task_id);
    _current_path_request.path.clear();
    _current_path_request.path.push_back(_last_known_state->location);

    _path_requested_time = std::chrono::steady_clock::now();
    _path_request_pub->publish(_current_path_request);
  }

  class DockFinder : public rmf_traffic::agv::Graph::Lane::Executor
  {
  public:

    bool is_dock = false;

    DockFinder(const std::string& dock_name)
    : _dock_name(std::move(dock_name))
    {
      // Do nothing
    }

    void execute(const Dock& dock) final
    {
      if (dock.dock_name() == _dock_name)
        is_dock = true;
    }

    void execute(const Wait&) final {}
    void execute(const DoorOpen&) final {}
    void execute(const DoorClose&) final {}
    void execute(const LiftSessionBegin&) final {}
    void execute(const LiftMove&) final {}
    void execute(const LiftDoorOpen&) final {}
    void execute(const LiftSessionEnd&) final {}

  private:
    const std::string& _dock_name;
  };

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final
  {
    auto lock = _lock();
    _clear_last_command();

    _dock_finished_callback = std::move(docking_finished_callback);
    _current_dock_request.parameters.front().value = dock_name;
    _current_dock_request.task_id = std::to_string(++_current_task_id);

    _dock_requested_time = std::chrono::steady_clock::now();
    _mode_request_pub->publish(_current_dock_request);

    // TODO(MXG): We should come up with a better way to identify the docking
    // lanes.
    _dock_target_wp = rmf_utils::nullopt;
    DockFinder finder(dock_name);
    for (std::size_t i = 0; i < _travel_info.graph->num_lanes(); ++i)
    {
      const auto& lane = _travel_info.graph->get_lane(i);
      const auto entry_event = lane.entry().event();
      if (entry_event)
      {
        entry_event->execute(finder);
        if (finder.is_dock)
        {
          _dock_target_wp = lane.entry().waypoint_index();
          break;
        }
      }
    }

    assert(_dock_target_wp);

    const auto& wp = _travel_info.graph->get_waypoint(*_dock_target_wp);
    const std::string wp_name = wp.name() ?
      *wp.name() : std::to_string(wp.index());

    RCLCPP_INFO(
      _node->get_logger(),
      "Requesting robot [%s] of [%s] to dock into waypoint [%s]",
      _current_path_request.robot_name.c_str(),
      _current_path_request.fleet_name.c_str(),
      wp_name.c_str());
  }

  void update_state(const rmf_fleet_msgs::msg::RobotState& state)
  {
    auto lock = _lock();
    _last_known_state = state;

    // Update battery soc
    const double battery_soc = state.battery_percent / 100.0;
    if (battery_soc >= 0.0 && battery_soc <= 1.0)
    {
      _travel_info.updater->update_battery_soc(battery_soc);
    }
    else
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "Battery percentage reported by the robot is outside of the valid "
        "range [0,100] and hence the battery soc will not be updated. It is "
        "critical to update the battery soc with a valid battery percentage "
        "for task allocation planning.");
    }

    // Reset these each time. They will get filled in by the estimation
    // functions as necessary.
    _travel_info.target_plan_index = std::nullopt;

    if (_travel_info.path_finished_callback)
    {
      // If we have a path_finished_callback, then the robot should be
      // following a path

      // There should not be a docking command happening
      assert(!_dock_finished_callback);

      // The arrival estimator should be available
      assert(_travel_info.next_arrival_estimator);

      if (state.task_id != _current_path_request.task_id)
      {
        // The robot has not received our path request yet
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::milliseconds(200) < now - _path_requested_time)
        {
          // We published the request a while ago, so we'll send it again in
          // case it got dropped.
          _path_requested_time = now;
          _path_request_pub->publish(_current_path_request);
        }

        return estimate_state(_node, state.location, _travel_info);
      }

      if (state.mode.mode == state.mode.MODE_ADAPTER_ERROR)
      {
        if (_interrupted)
        {
          // This interruption was already noticed
          return;
        }

        RCLCPP_INFO(
          _node->get_logger(),
          "Fleet driver [%s] reported a need to replan for [%s]",
          _current_path_request.fleet_name.c_str(),
          _current_path_request.robot_name.c_str());

        _interrupted = true;
        estimate_state(_node, state.location, _travel_info);
        return _travel_info.updater->replan();
      }

      if (state.path.empty())
      {
        // When the state path is empty, that means the robot believes it has
        // arrived at its destination.
        return check_path_finish(_node, state, _travel_info);
      }

      return estimate_path_traveling(_node, state, _travel_info);
    }
    else if (_dock_finished_callback)
    {
      const auto now = std::chrono::steady_clock::now();
      // If we have a _dock_finished_callback, then the robot should be docking
      if (state.task_id != _current_dock_request.task_id)
      {
        if (std::chrono::milliseconds(200) < now - _dock_requested_time)
        {
          // We published the request a while ago, so we'll send it again in
          // case it got dropped.
          _dock_requested_time = now;
          _mode_request_pub->publish(_current_dock_request);
        }

        return;
      }

      if (state.mode.mode != state.mode.MODE_DOCKING)
      {
        estimate_waypoint(_node, state.location, _travel_info);
        _travel_info.last_known_wp = *_dock_target_wp;
        _dock_finished_callback();
        _dock_finished_callback = nullptr;

        return;
      }

      // Update the schedule with the docking path of the robot
      if (!state.path.empty() &&
        std::chrono::seconds(1) < now - _dock_schedule_time)
      {
        std::vector<Eigen::Vector3d> positions;
        positions.push_back(
          {state.location.x, state.location.y, state.location.yaw});
        for (const auto& p : state.path)
          positions.push_back({p.x, p.y, p.yaw});

        const rmf_traffic::Trajectory trajectory =
          rmf_traffic::agv::Interpolate::positions(
          *_travel_info.traits,
          rmf_traffic_ros2::convert(state.location.t),
          positions);

        if (trajectory.size() < 2)
          return;

        if (auto participant =
          _travel_info.updater->unstable().get_participant())
        {
          participant->set(
            participant->assign_plan_id(),
            {rmf_traffic::Route{state.location.level_name, trajectory}});
          _dock_schedule_time = now;
        }
      }
    }
    else
    {
      // If we don't have a finishing callback, then the robot is not under our
      // command
      estimate_state(_node, state.location, _travel_info);
    }
  }

  void set_updater(rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
  {
    _travel_info.updater = std::move(updater);
    // Set the action_executor for the robot
    const auto teleop_executioner =
      [w = weak_from_this()](
      const std::string&,
      const nlohmann::json&,
      ActionExecution execution)
      {
        // We do not do anything here. The user can can move the robot by
        // sending PathRequest msgs. Instead we simply store the completed
        // callback which will be called when we receive a RobotModeRequest.
        const auto self = w.lock();
        if (!self)
          return;
        self->set_action_execution(execution);
      };
    _travel_info.updater->set_action_executor(teleop_executioner);
  }

  void newly_closed_lanes(const std::unordered_set<std::size_t>& closed_lanes)
  {
    bool need_to_replan = false;

    if (_travel_info.target_plan_index.has_value())
    {
      const auto& target_wp =
        _travel_info.waypoints.at(*_travel_info.target_plan_index);

      const auto& current_lanes = target_wp.approach_lanes();
      for (const auto& l : current_lanes)
      {
        if (closed_lanes.count(l))
        {
          need_to_replan = true;
          const auto& lane = _travel_info.graph->get_lane(l);
          const auto& loc = _last_known_state->location;
          const Eigen::Vector2d p = {loc.x, loc.y};

          const auto& wp0 =
            _travel_info.graph->get_waypoint(lane.entry().waypoint_index());
          const Eigen::Vector2d p0 = wp0.get_location();

          const auto& wp1 =
            _travel_info.graph->get_waypoint(lane.exit().waypoint_index());
          const Eigen::Vector2d p1 = wp1.get_location();

          const bool before_blocked_lane = (p-p0).dot(p1-p0) < 0.0;
          const bool after_blocked_lane = (p-p1).dot(p1-p0) >= 0.0;
          if (!before_blocked_lane && !after_blocked_lane)
          {
            // The robot is currently on a lane that has been closed. We take
            // this to mean that the robot needs to reverse.
            const Eigen::Vector3d position = {p.x(), p.y(), loc.yaw};
            const auto& return_waypoint = wp0.index();
            const auto* reverse_lane =
              _travel_info.graph->lane_from(wp1.index(), wp0.index());

            if (reverse_lane)
            {
              // We know what lane will reverse us back to the beginning of our
              // current lane, so we will update our position by saying that we
              // are on that lane.
              std::vector<std::size_t> lanes;
              lanes.push_back(reverse_lane->index());
              _travel_info.updater->update_position(position, lanes);
            }
            else
            {
              // There isn't an explicit lane for getting back to the beginning of
              // our current lane, so we will update with only our current
              // position and the waypoint index that we intend to return to.
              _travel_info.updater->update_position(position, return_waypoint);
            }
          }
        }
      }
    }

    if (!need_to_replan && _travel_info.target_plan_index.has_value())
    {
      // Check if the remainder of the current plan has been invalidated by the
      // lane closure.
      const auto next_index = *_travel_info.target_plan_index;
      for (std::size_t i = next_index; i < _travel_info.waypoints.size(); ++i)
      {
        for (const auto& lane : _travel_info.waypoints[i].approach_lanes())
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
      _travel_info.updater->replan();
  }

  void set_action_execution(ActionExecution action_execution)
  {
    _action_execution = action_execution;
  }

  void complete_robot_action()
  {
    if (!_action_execution.has_value())
      return;

    _action_execution->finished();
    _action_execution = std::nullopt;

    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] has completed the action it was performing",
      _travel_info.robot_name.c_str());
  }

  void handle_interrupt_request(
    const rmf_fleet_msgs::msg::InterruptRequest& request)
  {
    const auto it = _interruptions.find(request.interrupt_id);
    if (it == _interruptions.end())
    {
      if (request.type == request.TYPE_RESUME)
        return;

      _interruptions.insert(
        {
          request.interrupt_id,
          _travel_info.updater->interrupt(
            request.labels,
            [id = request.interrupt_id, name = _travel_info.robot_name]()
            {
              std::cout << "[" << name << "] is interrupted for " << id
                        << "!" << std::endl;
            })
        });

      return;
    }
    else
    {
      if (request.type == request.TYPE_INTERRUPT)
        return;

      it->second.resume(request.labels);
      std::cout << "Asking [" << _travel_info.robot_name << "] to resume for "
                << request.interrupt_id << std::endl;

      _interruptions.erase(it);
    }
  }

private:

  rclcpp::Node* _node;

  PathRequestPub _path_request_pub;
  rmf_fleet_msgs::msg::PathRequest _current_path_request;
  std::chrono::steady_clock::time_point _path_requested_time;
  TravelInfo _travel_info;
  std::optional<rmf_fleet_msgs::msg::RobotState> _last_known_state;
  bool _interrupted = false;

  rmf_fleet_msgs::msg::ModeRequest _current_dock_request;
  rmf_utils::optional<std::size_t> _dock_target_wp;
  std::chrono::steady_clock::time_point _dock_requested_time;
  std::chrono::steady_clock::time_point _dock_schedule_time =
    std::chrono::steady_clock::now();
  RequestCompleted _dock_finished_callback;
  ModeRequestPub _mode_request_pub;

  using Interruption = rmf_fleet_adapter::agv::RobotUpdateHandle::Interruption;
  std::unordered_map<std::string, Interruption> _interruptions;

  uint32_t _current_task_id = 0;

  std::recursive_mutex _mutex;

  // ActionExecution for managing teleop action
  std::optional<ActionExecution> _action_execution = std::nullopt;

  std::unique_lock<std::recursive_mutex> _lock()
  {
    std::unique_lock<std::recursive_mutex> lock(_mutex, std::defer_lock);
    while (!lock.try_lock())
    {
      // Intentionally busy wait
    }

    return lock;
  }

  void _clear_last_command()
  {
    _travel_info.next_arrival_estimator = nullptr;
    _travel_info.path_finished_callback = nullptr;
    _dock_finished_callback = nullptr;
  }
};

using FleetDriverRobotCommandHandlePtr =
  std::shared_ptr<FleetDriverRobotCommandHandle>;

//==============================================================================
/// This is an RAII class that keeps the connections to the fleet driver alive.
struct Connections : public std::enable_shared_from_this<Connections>
{
  /// The API for adding new robots to the adapter
  rmf_fleet_adapter::agv::FleetUpdateHandlePtr fleet;

  /// The API for running the fleet adapter
  rmf_fleet_adapter::agv::AdapterPtr adapter;

  /// The navigation graph for the robot
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  /// The traits of the vehicles
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits;

  /// The topic subscription for responding to new fleet states
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
    fleet_state_sub;

  /// The publisher for sending out path requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr
    path_request_pub;

  /// The publisher for sending out mode requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr
    mode_request_pub;

  /// The topic subscription for ending teleop actions
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr
    mode_request_sub;

  /// The client for listening to whether there is clearance in a lift
  rclcpp::Client<rmf_fleet_msgs::srv::LiftClearance>::SharedPtr
    lift_watchdog_client;

  /// The topic subscription for listening for lane closure requests
  rclcpp::Subscription<rmf_fleet_msgs::msg::LaneRequest>::SharedPtr
    lane_closure_request_sub;

  /// The publisher for sending out closed lane statuses
  rclcpp::Publisher<rmf_fleet_msgs::msg::ClosedLanes>::SharedPtr
    closed_lanes_pub;

  /// Container for remembering which lanes are currently closed
  std::unordered_set<std::size_t> closed_lanes;

  /// The topic subscription for robot interruption requests
  rclcpp::Subscription<rmf_fleet_msgs::msg::InterruptRequest>::SharedPtr
    interrupt_request_sub;

  /// The container for robot update handles
  std::unordered_map<std::string, FleetDriverRobotCommandHandlePtr> robots;

  void add_robot(
    const std::string& fleet_name,
    const rmf_fleet_msgs::msg::RobotState& state)
  {
    const auto& robot_name = state.name;
    const auto command = std::make_shared<FleetDriverRobotCommandHandle>(
      *adapter->node(), fleet_name, robot_name, graph, traits,
      path_request_pub, mode_request_pub);

    const auto& l = state.location;
    const auto& starts = rmf_traffic::agv::compute_plan_starts(
      *graph, state.location.level_name, {l.x, l.y, l.yaw},
      rmf_traffic_ros2::convert(adapter->node()->now()));

    if (starts.empty())
    {
      const std::optional<DistanceFromGraph> distance =
        distance_from_graph(state.location, *graph);

      std::string hint;
      if (!distance.has_value())
      {
        hint = "None of the waypoints in the graph are on a map called ["
          + l.level_name + "].";
      }
      else
      {
        const auto to_name = [&](const std::size_t index) -> std::string
          {
            const auto& wp = graph->get_waypoint(index);
            if (wp.name())
              return *wp.name();

            return "#" + std::to_string(index);
          };

        if (distance->type == DistanceFromGraph::Lane)
        {
          const auto& lane = graph->get_lane(distance->index);
          hint = "The closest lane on the navigation graph ["
            + std::to_string(distance->index) + "] connects waypoint ["
            + to_name(lane.entry().waypoint_index()) + "] to ["
            + to_name(lane.exit().waypoint_index()) + "] and is a distance of ["
            + std::to_string(distance->value) + "m] from the robot.";
        }
        else
        {
          hint = "The closest waypoint on the navigation graph ["
            + to_name(distance->index) + "] is a distance of ["
            + std::to_string(distance->value) + "m] from the robot.";
        }
      }

      RCLCPP_ERROR(
        adapter->node()->get_logger(),
        "Unable to compute a StartSet for robot [%s] using level_name [%s] and "
        "location [%f, %f, %f] specified in its RobotState message. This can "
        "happen if the level_name in the RobotState message does not match any "
        "of the map names in the navigation graph supplied or if the location "
        "reported in the RobotState message is far way from the navigation "
        "graph. This robot will not be added to the fleet [%s]. The following "
        "hint may help with debugging: %s",
        state.name.c_str(),
        state.location.level_name.c_str(),
        l.x, l.y, l.yaw,
        fleet_name.c_str(),
        hint.c_str());

      return;
    }

    fleet->add_robot(
      command, robot_name, traits->profile(),
      starts,
      [c = weak_from_this(), command, robot_name = std::move(robot_name)](
        const rmf_fleet_adapter::agv::RobotUpdateHandlePtr& updater)
      {
        const auto connections = c.lock();
        if (!connections)
          return;

        auto lock = connections->lock();

        if (connections->lift_watchdog_client)
        {
          updater->unstable().set_lift_entry_watchdog(
            [robot_name, connections](
              const std::string& lift_name,
              auto decide)
            {
              auto request =
              std::make_shared<rmf_fleet_msgs::srv::LiftClearance::Request>(
                rmf_fleet_msgs::build<rmf_fleet_msgs::srv::LiftClearance::Request>()
                .robot_name(robot_name)
                .lift_name(lift_name));

              const auto client = connections->lift_watchdog_client;
              if (!client->service_is_ready())
              {
                RCLCPP_ERROR(
                  connections->adapter->node()->get_logger(),
                  "Failed to get lift clearance service");
                decide(convert_decision(0));
                return;
              }

              client->async_send_request(
                request,
                [decide](
                  rclcpp::Client<rmf_fleet_msgs::srv::LiftClearance>::
                  SharedFuture response)
                {
                  const auto r = response.get();
                  decide(convert_decision(r->decision));
                });
            });
        }

        command->set_updater(updater);
        connections->robots[robot_name] = command;
      });
  }

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
std::shared_ptr<Connections> make_fleet(
  const rmf_fleet_adapter::agv::AdapterPtr& adapter)
{
  const auto& node = adapter->node();
  std::shared_ptr<Connections> connections = std::make_shared<Connections>();
  connections->adapter = adapter;

  const std::string fleet_name_param_name = "fleet_name";
  const std::string fleet_name = node->declare_parameter(
    "fleet_name", std::string());
  if (fleet_name.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", fleet_name_param_name.c_str());

    return nullptr;
  }

  connections->traits = std::make_shared<rmf_traffic::agv::VehicleTraits>(
    rmf_fleet_adapter::get_traits_or_default(
      *node, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5));

  const std::string nav_graph_param_name = "nav_graph_file";
  const std::string graph_file =
    node->declare_parameter(nav_graph_param_name, std::string());
  if (graph_file.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", nav_graph_param_name.c_str());

    return nullptr;
  }

  connections->graph =
    std::make_shared<rmf_traffic::agv::Graph>(
    rmf_fleet_adapter::agv::parse_graph(graph_file, *connections->traits));

  std::cout << "The fleet [" << fleet_name
            << "] has the following named waypoints:\n";
  for (const auto& key : connections->graph->keys())
    std::cout << " -- " << key.first << std::endl;

  std::optional<std::string> server_uri = std::nullopt;
  const std::string uri =
    node->declare_parameter("server_uri", std::string());
  if (!uri.empty())
  {
    RCLCPP_INFO(
      node->get_logger(),
      "API server URI: [%s]", uri.c_str());

    server_uri = uri;
  }

  connections->fleet = adapter->add_fleet(
    fleet_name, *connections->traits, *connections->graph, server_uri);

  // We disable fleet state publishing for this fleet adapter because we expect
  // the fleet drivers to publish these messages.
  connections->fleet->fleet_state_topic_publish_period(std::nullopt);

  connections->closed_lanes_pub =
    adapter->node()->create_publisher<rmf_fleet_msgs::msg::ClosedLanes>(
    rmf_fleet_adapter::ClosedLaneTopicName,
    rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local());

  connections->lane_closure_request_sub =
    adapter->node()->create_subscription<rmf_fleet_msgs::msg::LaneRequest>(
    rmf_fleet_adapter::LaneClosureRequestTopicName,
    rclcpp::SystemDefaultsQoS(),
    [w = connections->weak_from_this(), fleet_name](
      rmf_fleet_msgs::msg::LaneRequest::UniquePtr request_msg)
    {
      const auto connections = w.lock();
      if (!connections)
        return;

      if (request_msg->fleet_name != fleet_name &&
      !request_msg->fleet_name.empty())
        return;

      connections->fleet->open_lanes(request_msg->open_lanes);
      connections->fleet->close_lanes(request_msg->close_lanes);

      std::unordered_set<std::size_t> newly_closed_lanes;
      for (const auto& l : request_msg->close_lanes)
      {
        if (connections->closed_lanes.count(l) == 0)
          newly_closed_lanes.insert(l);

        connections->closed_lanes.insert(l);
      }

      for (const auto& l : request_msg->open_lanes)
        connections->closed_lanes.erase(l);

      for (auto& [_, robot] : connections->robots)
        robot->newly_closed_lanes(newly_closed_lanes);

      rmf_fleet_msgs::msg::ClosedLanes state_msg;
      state_msg.fleet_name = fleet_name;
      state_msg.closed_lanes.insert(
        state_msg.closed_lanes.begin(),
        connections->closed_lanes.begin(),
        connections->closed_lanes.end());

      connections->closed_lanes_pub->publish(state_msg);
    });

  connections->interrupt_request_sub =
    adapter->node()->create_subscription<rmf_fleet_msgs::msg::InterruptRequest>(
    rmf_fleet_adapter::InterruptRequestTopicName,
    rclcpp::SystemDefaultsQoS(),
    [w = connections->weak_from_this(), fleet_name](
      rmf_fleet_msgs::msg::InterruptRequest::UniquePtr request_msg)
    {
      const auto connections = w.lock();
      if (!connections)
        return;

      if (request_msg->fleet_name != fleet_name)
        return;

      const auto rit = connections->robots.find(request_msg->robot_name);
      if (rit == connections->robots.end())
      {
        RCLCPP_WARN(
          connections->adapter->node()->get_logger(),
          "Could not find robot [%s] in fleet [%s]",
          request_msg->robot_name.c_str(),
          fleet_name.c_str());
        return;
      }

      rit->second->handle_interrupt_request(*request_msg);
    });

  /* *INDENT-OFF* */
  connections->mode_request_sub =
    adapter->node()->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
    "/action_execution_notice",
    rclcpp::SystemDefaultsQoS(),
    [w = connections->weak_from_this(), fleet_name](
      rmf_fleet_msgs::msg::ModeRequest::UniquePtr msg)
    {
      if (msg->fleet_name.empty() ||
        msg->fleet_name != fleet_name ||
        msg->robot_name.empty())
      {
        return;
      }

      if (msg->mode.mode == msg->mode.MODE_IDLE)
      {
        const auto self = w.lock();
        if (!self)
          return;

        const auto command_it = self->robots.find(msg->robot_name);
        if (command_it == self->robots.end())
          return;

        command_it->second->complete_robot_action();
      }
    });
  /* *INDENT-ON* */

  // Parameters required for task planner
  // Battery system
  auto battery_system_optional = rmf_fleet_adapter::get_battery_system(
    *node, 24.0, 40.0, 8.8);
  if (!battery_system_optional)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for battery system");

    return nullptr;
  }
  auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
    *battery_system_optional);

  // Mechanical system and motion_sink
  auto mechanical_system_optional = rmf_fleet_adapter::get_mechanical_system(
    *node, 70.0, 40.0, 0.22);
  if (!mechanical_system_optional)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for mechanical system");

    return nullptr;
  }
  rmf_battery::agv::MechanicalSystem& mechanical_system =
    *mechanical_system_optional;

  std::shared_ptr<rmf_battery::agv::SimpleMotionPowerSink> motion_sink =
    std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
    *battery_system, mechanical_system);

  // Ambient power system
  const double ambient_power_drain =
    rmf_fleet_adapter::get_parameter_or_default(
    *node, "ambient_power_drain", 20.0);
  auto ambient_power_system = rmf_battery::agv::PowerSystem::make(
    ambient_power_drain);
  if (!ambient_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for ambient power system");

    return nullptr;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> ambient_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *ambient_power_system);

  // Tool power system
  const double tool_power_drain = rmf_fleet_adapter::get_parameter_or_default(
    *node, "tool_power_drain", 10.0);
  auto tool_power_system = rmf_battery::agv::PowerSystem::make(
    tool_power_drain);
  if (!tool_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for tool power system");

    return nullptr;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> tool_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  // Drain battery
  const bool drain_battery = rmf_fleet_adapter::get_parameter_or_default(
    *node, "drain_battery", false);
  // Recharge threshold
  const double recharge_threshold = rmf_fleet_adapter::get_parameter_or_default(
    *node, "recharge_threshold", 0.2);
  // Recharge state of charge
  const double recharge_soc = rmf_fleet_adapter::get_parameter_or_default(
    *node, "recharge_soc", 1.0);
  const std::string finishing_request_string =
    node->declare_parameter("finishing_request", "nothing");
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

  if (!connections->fleet->set_task_planner_params(
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

    return nullptr;
  }

  std::unordered_set<uint8_t> task_types;
  if (node->declare_parameter<bool>("perform_loop", false))
  {
    task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_LOOP);
  }

  // If the perform_deliveries parameter is true, then we just blindly accept
  // all delivery requests.
  if (node->declare_parameter<bool>("perform_deliveries", false))
  {
    task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_DELIVERY);
    connections->fleet->accept_delivery_requests(
      [](const rmf_task_msgs::msg::Delivery&) { return true; });
  }

  if (node->declare_parameter<bool>("perform_cleaning", false))
  {
    task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_CLEAN);
  }

  connections->fleet->accept_task_requests(
    [task_types](const rmf_task_msgs::msg::TaskProfile& msg)
    {
      if (task_types.find(msg.description.task_type.type) != task_types.end())
        return true;

      return false;
    });

  const auto consider =
    [](const nlohmann::json& /*description*/,
      rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      // We accept all actions since full_control may be used for different
      // types of robots.
      confirm.accept();
    };

  // Configure this fleet to perform any kind of teleop action
  connections->fleet->add_performable_action(
    "teleop",
    consider);

  if (node->declare_parameter<bool>("disable_delay_threshold", false))
  {
    connections->fleet->default_maximum_delay(rmf_utils::nullopt);
  }
  else
  {
    connections->fleet->default_maximum_delay(
      rmf_fleet_adapter::get_parameter_or_default_time(
        *node, "delay_threshold", 10.0));
  }

  connections->path_request_pub = node->create_publisher<
    rmf_fleet_msgs::msg::PathRequest>(
    rmf_fleet_adapter::PathRequestTopicName, rclcpp::SystemDefaultsQoS());

  connections->mode_request_pub = node->create_publisher<
    rmf_fleet_msgs::msg::ModeRequest>(
    rmf_fleet_adapter::ModeRequestTopicName, rclcpp::SystemDefaultsQoS());

  connections->fleet_state_sub = node->create_subscription<
    rmf_fleet_msgs::msg::FleetState>(
    rmf_fleet_adapter::FleetStateTopicName,
    rclcpp::SystemDefaultsQoS(),
    [c = std::weak_ptr<Connections>(connections), fleet_name](
      const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
    {
      if (msg->name != fleet_name)
        return;

      const auto connections = c.lock();
      if (!connections)
        return;

      for (const auto& state : msg->robots)
      {
        const auto insertion = connections->robots.insert({state.name,
          nullptr});
        const bool new_robot = insertion.second;
        if (new_robot)
        {
          // We have not seen this robot before, so let's add it to the fleet.
          connections->add_robot(fleet_name, state);
        }

        const auto& command = insertion.first->second;
        if (command)
        {
          // We are ready to command this robot, so let's update its state
          command->update_state(state);
        }
      }
    });

  const std::string lift_clearance_srv =
    node->declare_parameter<std::string>(
    "experimental_lift_watchdog_service", "");
  if (!lift_clearance_srv.empty())
  {
    connections->lift_watchdog_client =
      node->create_client<rmf_fleet_msgs::srv::LiftClearance>(
      lift_clearance_srv);
  }

  return connections;
}

//==============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("fleet_adapter");
  if (!adapter)
    return 1;

  const auto fleet_connections = make_fleet(adapter);
  if (!fleet_connections)
    return 1;

  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Fleet Adapter");

  // Start running the adapter and wait until it gets stopped by SIGINT
  adapter->start().wait();

  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();
}
