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

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>

// Public rmf_task API headers
#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ParkRobotFactory.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include <thread>

//==============================================================================
namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
namespace {
/// Implements a state machine to send waypoints from follow_new_path() one
/// at a time to the robot via its API. Also updates state of robot via a timer.
class EasyCommandHandle : public RobotCommandHandle,
  public std::enable_shared_from_this<EasyCommandHandle>
{
public:
  using Planner = rmf_traffic::agv::Planner;
  using Graph = rmf_traffic::agv::Graph;
  using VehicleTraits = rmf_traffic::agv::VehicleTraits;
  using ActionExecutor = RobotUpdateHandle::ActionExecutor;
  using RobotState = EasyFullControl::RobotState;
  using GetStateCallback = EasyFullControl::GetStateCallback;
  using GoalCompletedCallback = EasyFullControl::GoalCompletedCallback;
  using NavigationRequest = EasyFullControl::NavigationRequest;
  using StopRequest = EasyFullControl::StopRequest;
  using DockRequest = EasyFullControl::DockRequest;

  // State machine values.
  enum class InternalRobotState : uint8_t
  {
    IDLE = 0,
    MOVING = 1
  };

  // Custom waypoint created from Plan::Waypoint.
  struct InternalPlanWaypoint
  {
    // Index in follow_new_path
    std::size_t index;
    Eigen::Vector3d position;
    rmf_traffic::Time time;
    std::optional<std::size_t> graph_index;
    std::vector<std::size_t> approach_lanes;

    InternalPlanWaypoint(
      std::size_t index_,
      const rmf_traffic::agv::Plan::Waypoint& wp)
    : index(index_),
      position(wp.position()),
      time(wp.time()),
      graph_index(wp.graph_index()),
      approach_lanes(wp.approach_lanes())
    {
      // Do nothing
    }
  };

  EasyCommandHandle(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<Graph> graph,
    std::shared_ptr<VehicleTraits> traits,
    const std::string& robot_name,
    Planner::Start plan_start,
    RobotState start_state,
    GetStateCallback get_state,
    NavigationRequest handle_nav_request,
    StopRequest handle_stop,
    DockRequest handle_dock,
    ActionExecutor action_executor,
    double max_merge_waypoint_distance,
    double max_merge_lane_distance,
    double min_lane_length,
    rmf_traffic::Duration update_interval);

  ~EasyCommandHandle();

  // Implement base class methods.
  void stop() final;

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) final;

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final;

  // Callback to set the RobotUpdateHandle along with action executor.
  void set_updater(RobotUpdateHandlePtr updater);

  // Variables
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr update_timer;
  RobotUpdateHandlePtr updater;
  std::shared_ptr<Graph> graph;
  std::shared_ptr<VehicleTraits> traits;
  rmf_traffic::Duration update_interval;

  // Callbacks from user
  std::string robot_name;
  EasyFullControl::RobotState state;
  EasyFullControl::GetStateCallback get_state;
  NavigationRequest handle_nav_request;
  StopRequest handle_stop;
  DockRequest handle_dock;
  // Store the ActionExecutor for this robot till RobotUpdateHandle is ready.
  ActionExecutor action_executor;

  double max_merge_waypoint_distance;
  double max_merge_lane_distance;
  double min_lane_length;

  // Internal tracking variables.
  InternalRobotState _state;
  std::optional<std::size_t> on_waypoint = std::nullopt;
  std::optional<std::size_t> last_known_waypoint = std::nullopt;
  std::optional<std::size_t> on_lane = std::nullopt;
  std::optional<std::size_t> dock_waypoint_index = std::nullopt;
  std::optional<rclcpp::Time> last_replan_time = std::nullopt;
  std::string dock_name;

  std::optional<InternalPlanWaypoint> target_waypoint;
  std::vector<InternalPlanWaypoint> remaining_waypoints;

  std::mutex mutex;
  std::thread follow_thread;
  std::thread dock_thread;
  std::thread stop_thread;
  std::atomic_bool quit_follow_thread = false;
  std::atomic_bool quit_stop_thread = false;
  std::atomic_bool quit_dock_thread = false;
  std::atomic_bool navigation_completed;
  RequestCompleted path_finished_callback;
  RequestCompleted docking_finished_callback;
  ArrivalEstimator next_arrival_estimator;

  // Internal functions
  void clear();
  void interrupt();
  void replan();
  void parse_waypoints(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints);
  void update();
  std::optional<std::size_t> get_current_lane();
  void start_follow();
  void start_dock();
};

//==============================================================================
EasyCommandHandle::EasyCommandHandle(
  rclcpp::Node::SharedPtr node_,
  std::shared_ptr<Graph> graph_,
  std::shared_ptr<VehicleTraits> traits_,
  const std::string& robot_name_,
  Planner::Start plan_start,
  EasyFullControl::RobotState start_state_,
  EasyFullControl::GetStateCallback get_state_,
  EasyFullControl::NavigationRequest handle_nav_request_,
  EasyFullControl::StopRequest handle_stop_,
  EasyFullControl::DockRequest handle_dock_,
  RobotUpdateHandle::ActionExecutor action_executor_,
  double max_merge_waypoint_distance_,
  double max_merge_lane_distance_,
  double min_lane_length_,
  rmf_traffic::Duration update_interval_)
: node(std::move(node_)),
  graph(std::move(graph_)),
  traits(std::move(traits_)),
  robot_name(std::move(robot_name_)),
  state(std::move(start_state_)),
  get_state(std::move(get_state_)),
  handle_nav_request(std::move(handle_nav_request_)),
  handle_stop(std::move(handle_stop_)),
  handle_dock(std::move(handle_dock_)),
  action_executor(std::move(action_executor_)),
  max_merge_waypoint_distance(std::move(max_merge_waypoint_distance_)),
  max_merge_lane_distance(std::move(max_merge_lane_distance_)),
  min_lane_length(std::move(min_lane_length_)),
  update_interval(std::move(update_interval_))
{
  if (plan_start.lane().has_value())
  {
    on_waypoint = std::nullopt;
    on_lane = plan_start.lane().value();
  }
  else
  {
    on_waypoint = plan_start.waypoint();
    last_known_waypoint = plan_start.waypoint();
  }

  updater = nullptr;
  // Initialize the update timer
  // (YV): It is okay to capture "this" raw ptr here as the callback will not
  // outlive the node.
  update_timer = node->create_wall_timer(
    update_interval,
    [this]()
    {
      update();
    }
  );
}

//==============================================================================
EasyCommandHandle::~EasyCommandHandle()
{
  if (stop_thread.joinable())
  {
    quit_stop_thread = true;
    stop_thread.join();
  }

  if (follow_thread.joinable())
  {
    quit_follow_thread = true;
    follow_thread.join();
  }

  if (dock_thread.joinable())
  {
    quit_dock_thread = true;
    dock_thread.join();
  }
}

//==============================================================================
void EasyCommandHandle::update()
{
  if (!updater)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Unable to update state of robot [%s] as the RobotUpdateHandle for this "
      "robot is not available yet",
      robot_name.c_str()
    );
  }
  // Make a temporary copy of the new state to check if the charger waypoint
  // has changed.
  auto _state = get_state();
  if (_state.charger_name() != state.charger_name())
  {
    const auto& charger_name = _state.charger_name();
    auto charger_wp = graph->find_waypoint(charger_name);
    if (charger_wp)
    {
      updater->set_charger_waypoint(charger_wp->index());
    }
    else
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Unable to set charger waypoint of robot [%s] to [%s] as no such "
        "waypoint exists on the navigation graph provided for the fleet.",
        robot_name.c_str(),
        charger_name.c_str()
      );
    }
  }
  state = std::move(_state);
  updater->update_battery_soc(state.battery_soc());
  const auto& position = state.location();
  // Here we call the appropriate RobotUpdateHandle::update() method depending
  // on the state of the tracker variables.
  // If robot is on a waypoint.
  if (on_waypoint.has_value())
  {
    const std::size_t& wp = on_waypoint.value();
    const double& ori = position[2];
    RCLCPP_DEBUG(
      node->get_logger(),
      "[%s] Calling update with waypoint [%ld] and orientation [%.2f]",
      robot_name.c_str(), wp, ori);
    updater->update_position(wp, ori);
  }
  // If the robot is on a lane.
  else if (on_lane.has_value())
  {
    // It is recommended to update with both the forward and backward lane if
    // present.
    std::vector<std::size_t> lanes = {on_lane.value()};
    const auto& forward_lane = graph->get_lane(on_lane.value());
    const auto& entry_index = forward_lane.entry().waypoint_index();
    const auto& exit_index = forward_lane.exit().waypoint_index();
    const auto reverse_lane = graph->lane_from(exit_index, entry_index);
    if (reverse_lane)
      lanes.push_back(reverse_lane->index());
    RCLCPP_DEBUG(
      node->get_logger(),
      "[%s] Calling update with position [%.2f, %.2f, %.2f] and lane count "
      "[%ld].",
      robot_name.c_str(), position[0], position[1], position[2],
      lanes.size());
    updater->update_position(position, lanes);
  }
  // If the robot is merging into a waypoint.
  else if (target_waypoint.has_value() &&
    target_waypoint.value().graph_index.has_value())
  {
    const auto& graph_index = target_waypoint.value().graph_index.value();
    RCLCPP_DEBUG(
      node->get_logger(),
      "[%s] Calling update with position [%.2f, %.2f, %.2f] and target "
      "waypoint [%ld].",
      robot_name.c_str(), position[0], position[1], position[2],
      graph_index);
    updater->update_position(position, graph_index);
  }
  // If the robot is docking.
  else if (dock_waypoint_index.has_value())
  {
    const auto& graph_index = dock_waypoint_index.value();
    RCLCPP_DEBUG(
      node->get_logger(),
      "[%s] Calling update with position [%.2f, %.2f, %.2f] and dock waypoint "
      "[%ld].",
      robot_name.c_str(), position[0], position[1], position[2],
      graph_index);
    updater->update_position(position, graph_index);
  }
  // If the robot is lost.
  else
  {
    RCLCPP_DEBUG(
      node->get_logger(),
      "[%s] Calling update with map_name [%s] and position [%.2f, %.2f, %.2f]",
      robot_name.c_str(),
      state.map_name().c_str(), position[0], position[1], position[2]);
    updater->update_position(
      state.map_name(),
      state.location(),
      max_merge_waypoint_distance,
      max_merge_lane_distance,
      min_lane_length);
  }
}

//==============================================================================
void EasyCommandHandle::set_updater(
  RobotUpdateHandlePtr updater_)
{
  updater = std::move(updater_);
  updater->set_action_executor(std::move(action_executor));
  RCLCPP_INFO(
    node->get_logger(),
    "Successfully set the RobotUpdateHandle for robot [%s]",
    robot_name.c_str()
  );
}

//==============================================================================
void EasyCommandHandle::stop()
{
  if (updater)
  {
    const auto plan_id = updater->unstable().current_plan_id();
    RCLCPP_DEBUG(
      node->get_logger(),
      "Stoping robot [%s] with PlanId [%ld]",
      robot_name.c_str(), plan_id);
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Received request to stop robot [%s].",
    robot_name.c_str()
  );

  interrupt();
  quit_stop_thread = false;
  stop_thread = std::thread(
    [w = weak_from_this()]()
    {
      auto stopped = false;
      while (!stopped)
      {
        auto me = w.lock();
        if (!me)
        {
          continue;
        }
        if (me->quit_stop_thread)
        {
          return;
        }
        if (me->handle_stop())
        {
          RCLCPP_INFO(
            me->node->get_logger(),
            "Successfully stopped robot [%s].",
            me->robot_name.c_str()
          );
          break;
        }
        RCLCPP_ERROR(
          me->node->get_logger(),
          "Unable to stop robot [%s] using its StopRequest callback. Retrying "
          "in [0.1] seconds.",
          me->robot_name.c_str()
        );
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
}

//==============================================================================
void EasyCommandHandle::replan()
{
  if (!updater)
    return;

  const auto& now = node->get_clock()->now();
  if (last_replan_time.has_value())
  {
    // TODO(MXG): Make the 15s replan cooldown configurable
    if (now - last_replan_time.value() < rclcpp::Duration::from_seconds(15.0))
      return;
  }

  last_replan_time = now;
  updater->replan();

  RCLCPP_INFO(
    node->get_logger(),
    "Requesting replan for %s because of an obstacle",
    robot_name.c_str());
}

//==============================================================================
void EasyCommandHandle::follow_new_path(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  ArrivalEstimator next_arrival_estimator_,
  RequestCompleted path_finished_callback_)
{
  RCLCPP_DEBUG(
    node->get_logger(),
    "follow_new_path for robot [%s] with PlanId [%ld]",
    robot_name.c_str(), updater->unstable().current_plan_id()
  );


  if (waypoints.empty() ||
    next_arrival_estimator_ == nullptr ||
    path_finished_callback_ == nullptr)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Received a new path for robot [%s] with invalid parameters. "
      " Ignoring...",
      robot_name.c_str()
    );
    return;
  }

  interrupt();
  if (follow_thread.joinable())
  {
    RCLCPP_INFO(
      node->get_logger(),
      "[stop] _follow_thread present. Calling join");
    quit_follow_thread = true;
    follow_thread.join();
  }
  quit_follow_thread = false;
  clear();

  RCLCPP_INFO(
    node->get_logger(),
    "Received a new path with [%ld] waypoints for robot [%s]. "
    "Calling stop() and following new path...",
    waypoints.size(),
    robot_name.c_str()
  );

  // TODO(YV): Accept a boolean parameter to indicate whether robots form this
  // fleet need to be explicitly stopped before sending a new goal.
  // We stop the robot in case the NavigationRequest API does not preempt the
  // previous goal.
  // stop();

  // Lock mutex
  std::lock_guard<std::mutex> lock(mutex);

  // Reset internal trackers
  //clear();

  parse_waypoints(waypoints);
  RCLCPP_DEBUG(
    node->get_logger(),
    "remaining_waypoints: [%ld]. target_waypoint.has_value: [%ld]",
    remaining_waypoints.size(), target_waypoint.has_value());

  next_arrival_estimator = std::move(next_arrival_estimator_);
  path_finished_callback = std::move(path_finished_callback_);

  // With the new event based traffic system, we no longer need to check if the
  // robot has to wait at its current location. A follow_new_path() request
  // is only sent once the robot is ready to move. Hence we can immediately
  // request the robot to navigate through the waypoints.
  follow_thread = std::thread(
    [w = weak_from_this()]()
    {
      auto me = w.lock();
      if (!me)
        return;
      me->start_follow();
    });
}

//==============================================================================
void EasyCommandHandle::dock(
  const std::string& dock_name_,
  RequestCompleted docking_finished_callback_)
{
  RCLCPP_INFO(
    node->get_logger(),
    "Received a request to dock robot [%s] at [%s]...",
    robot_name.c_str(),
    dock_name_.c_str());

  if (dock_thread.joinable())
  {
    RCLCPP_DEBUG(
      node->get_logger(),
      "[stop] _dock_thread present. Calling join");
    quit_dock_thread = true;
    dock_thread.join();
  }

  dock_name = dock_name_;
  assert(docking_finished_callback != nullptr);
  docking_finished_callback = std::move(docking_finished_callback_);

  // Get the waypoint that the robot is trying to dock into
  const auto dock_waypoint = graph->find_waypoint(dock_name);
  if (dock_waypoint == nullptr)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Unable to dock at dock_name [%s] as the internal implementation of "
      "this adapter assumes that the dock_name property matches the name of "
      "the waypoint on the graph that the robot is docking to.",
      dock_name.c_str()
    );
    docking_finished_callback();
  }

  dock_waypoint_index = dock_waypoint->index();

  {
    std::lock_guard<std::mutex> lock(mutex);
    on_waypoint = std::nullopt;
    on_lane = std::nullopt;
  }

  quit_dock_thread = false;
  dock_thread = std::thread(
    [w = weak_from_this()]()
    {
      auto me = w.lock();
      if (!me)
        return;
      me->start_dock();
    });
}

//==============================================================================
void EasyCommandHandle::start_follow()
{
  // Function to be overwritten with GoalCompletedCallback returned from
  // navigation handle.
  GoalCompletedCallback nav_completed_cb;

  while ((!remaining_waypoints.empty() ||
    _state == InternalRobotState::MOVING) && !quit_follow_thread)
  {
    if (_state == InternalRobotState::IDLE || !target_waypoint.has_value())
    {
      // Assign the next waypoint.
      {
        std::lock_guard<std::mutex> lock(mutex);
        target_waypoint = remaining_waypoints[0];
      }
      const auto& target_pose = target_waypoint->position;
      const auto& map_name = target_waypoint->graph_index.has_value() ?
        graph->get_waypoint(*target_waypoint->graph_index).get_map_name() :
        state.map_name();
      RCLCPP_INFO(
        node->get_logger(),
        "Requesting robot [%s] to navigate to Open-RMF coordinates "
        "[%.2f, %.2f, %.2f] on level [%s].",
        robot_name.c_str(),
        target_pose[0], target_pose[1], target_pose[2],
        map_name.c_str()
      );
      nav_completed_cb = handle_nav_request(
        map_name,
        target_pose,
        updater);

      if (nav_completed_cb != nullptr)
      {
        std::lock_guard<std::mutex> lock(mutex);
        remaining_waypoints.erase(remaining_waypoints.begin());
        _state = InternalRobotState::MOVING;
      }
      else
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Did not receive a valid GoalCompletionCallback from calling "
          "handle_nav_request() for robot [%s]. Retrying navigation request in "
          "[%.2f] seconds.",
          robot_name.c_str(), update_interval.count()/1e9);
        std::this_thread::sleep_for(update_interval);
      }
    }
    else if (_state == InternalRobotState::MOVING)
    {
      // Variables which the nav_completed_cb will overwrite.
      const auto now = std::chrono::steady_clock::now();
      const rmf_traffic::Trajectory t =
        rmf_traffic::agv::Interpolate::positions(
          *traits,
          now,
          {state.location(), target_waypoint->position}
      );
      auto trajectory_time = t.finish_time();
      rmf_traffic::Duration remaining_time  =
        trajectory_time ? *trajectory_time - now : rmf_traffic::time::from_seconds(5.0);
      bool request_replan = false;
      std::this_thread::sleep_for(update_interval);
      if (nav_completed_cb(remaining_time, request_replan))
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Robot [%s] has reached its target waypoint.",
          robot_name.c_str()
        );
        _state = InternalRobotState::IDLE;

        std::lock_guard<std::mutex> lock(mutex);
        if (target_waypoint.has_value() && target_waypoint->graph_index.has_value())
        {
          on_waypoint = target_waypoint->graph_index;
          last_known_waypoint = on_waypoint;
        }
      }
      else
      {
        // If the user requested a replan for this robot, trigger one.
        if (request_replan)
        {
          replan();
        }
        // Still on a lane.
        std::lock_guard<std::mutex> lock(mutex);
        on_waypoint = std::nullopt;
        on_lane = get_current_lane();
        if (on_lane.has_value())
        {
          on_waypoint = std::nullopt;
        }
        else
        {
          auto dist = [](
            const Eigen::Vector3d& a, const Eigen::Vector3d& b) -> double
            {
              return (a.block<2, 1>(0, 0) - b.block<2, 1>(0, 0)).norm();
            };

          // The robot may either be on the previous or target waypoint.
          const auto& last_location =
            graph->get_waypoint(last_known_waypoint.value()).get_location();
          const Eigen::Vector3d last_pose =
          {last_location[0], last_location[1], 0.0};
          if (target_waypoint->graph_index.has_value() &&
            dist(state.location(), target_waypoint->position) < 0.5)
          {
            on_waypoint = target_waypoint->graph_index.value();
          }
          else if (last_known_waypoint.has_value() &&
            dist(state.location(), last_pose) < 0.5)
          {
            on_waypoint = last_known_waypoint.value();
          }
          else
          {
            // The robot is probably off-grid
            on_waypoint = std::nullopt;
            on_lane = std::nullopt;
          }
        }

        // Update arrival estimate
        if (target_waypoint.has_value())
        {
          next_arrival_estimator(
            target_waypoint->index,
            remaining_time);
        }
      }
    }
  }

  // The robot is done navigating through all the waypoints
  assert(path_finished_callback);
  path_finished_callback();
  RCLCPP_INFO(
    node->get_logger(),
    "Robot [%s] has successfully navigated along the requested path.",
    robot_name.c_str());
  path_finished_callback = nullptr;
  next_arrival_estimator = nullptr;
}

//==============================================================================
void EasyCommandHandle::start_dock()
{
  auto docking_cb = handle_dock(dock_name, updater);
  rmf_traffic::Duration remaining_time;
  bool request_replan;
  while (!docking_cb(remaining_time, request_replan) && !quit_dock_thread)
  {
    RCLCPP_DEBUG(
      node->get_logger(),
      "Waiting for docking to finish for robot [%s]",
      robot_name.c_str()
    );
    std::this_thread::sleep_for(update_interval);
  }

  std::lock_guard<std::mutex> lock(mutex);
  on_waypoint = dock_waypoint_index;
  dock_waypoint_index = std::nullopt;
  docking_finished_callback();
  RCLCPP_INFO(
    node->get_logger(),
    "Robot [%s] has completed docking",
    robot_name.c_str()
  );
}

//==============================================================================
void EasyCommandHandle::clear()
{
  target_waypoint = std::nullopt;
  remaining_waypoints.clear();
  _state = InternalRobotState::IDLE;
  quit_follow_thread = false;
}

//==============================================================================
void EasyCommandHandle::interrupt()
{
  RCLCPP_DEBUG(
    node->get_logger(),
    "Interrupting %s",
    robot_name.c_str()
  );

  quit_follow_thread = true;
  quit_dock_thread = true;
  quit_stop_thread = true;

  if (follow_thread.joinable())
    follow_thread.join();
  if (dock_thread.joinable())
    dock_thread.join();
  if (stop_thread.joinable())
    stop_thread.join();
}

//==============================================================================
void EasyCommandHandle::parse_waypoints(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints)
{
  if (waypoints.empty())
    return;

  std::vector<InternalPlanWaypoint> wps;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    wps.push_back(InternalPlanWaypoint(i, waypoints[i]));

  // If the robot is already in the middle of two waypoints, then we can
  // truncate all the waypoints that come before it.
  auto begin_at_index = 0;
  const auto& p = state.location().block<2, 1>(0, 0);
  for (int i = wps.size() - 1; i >= 0; i--)
  {
    std::size_t i0, i1;
    i0 = i;
    i1 = i + 1;
    Eigen::Vector2d p0(wps[i0].position.x(), wps[i0].position.y());
    Eigen::Vector2d p1(wps[i1].position.x(), wps[i1].position.y());
    const auto dp_lane = p1 - p0;
    const double lane_length = dp_lane.norm();
    if (lane_length < 1e-3)
    {
      continue;
    }
    const auto n_lane = dp_lane / lane_length;
    const auto p_l = p - p0;
    const double p_l_proj = p_l.dot(n_lane);

    if (lane_length < p_l_proj)
    {
      // Check if the robot's position is close enough to the lane
      // endpoint to merge it
      if ((p - p1).norm() <= max_merge_lane_distance)
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
      if ((p - p0).norm() <= max_merge_lane_distance)
      {
        begin_at_index = i0;
        break;
      }
      // Otherwise, continue to the next lane because the robot is not
      // between the lane endpoints
      continue;
    }

    const double lane_dist = (p_l - p_l_proj * n_lane).norm();
    if (lane_dist <= max_merge_lane_distance)
    {
      begin_at_index = i1;
      break;
    }
  }

  if (begin_at_index > 0)
  {
    wps.erase(wps.begin(), wps.begin() + begin_at_index);
  }

  target_waypoint = std::nullopt;
  remaining_waypoints = std::move(wps);
  return;
}

//==============================================================================
std::optional<std::size_t> EasyCommandHandle::get_current_lane()
{
  const auto projection = [](
    const Eigen::Vector2d& current_position,
    const Eigen::Vector2d& target_position,
    const Eigen::Vector2d& lane_entry,
    const Eigen::Vector2d& lane_exit) -> double
    {
      return (current_position - target_position).dot(lane_exit - lane_entry);
    };

  if (!target_waypoint.has_value())
    return std::nullopt;
  const auto& approach_lanes = target_waypoint->approach_lanes;
  // Empty approach lanes signifies the robot will rotate at the waypoint.
  // Here we rather update that the robot is at the waypoint rather than
  // approaching it.
  if (approach_lanes.empty())
    return std::nullopt;

  for (const auto& lane_index : approach_lanes)
  {
    const auto& lane = graph->get_lane(lane_index);
    const auto& p0 =
      graph->get_waypoint(lane.entry().waypoint_index()).get_location();
    const auto& p1 =
      graph->get_waypoint(lane.exit().waypoint_index()).get_location();
    const auto& p = state.location().block<2, 1>(0, 0);
    const bool before_lane = projection(p, p0, p0, p1) < 0.0;
    const bool after_lane = projection(p, p1, p0, p1) >= 0.0;
    if (!before_lane && !after_lane) // the robot is on this lane
      return lane_index;
  }
  return std::nullopt;
}
} // namespace anonymous

//==============================================================================
class EasyFullControl::Configuration::Implementation
{
public:
  std::string fleet_name;
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
  rmf_traffic::Duration update_interval;
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
  rmf_traffic::Duration max_delay,
  rmf_traffic::Duration update_interval)
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
        std::move(max_delay),
        std::move(update_interval)
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
rmf_traffic::Duration EasyFullControl::Configuration::update_interval() const
{
  return _pimpl->update_interval;
}

//==============================================================================
const std::vector<std::string>&
EasyFullControl::Configuration::action_categories() const
{
  return _pimpl->action_categories;
}

//==============================================================================
class EasyFullControl::RobotState::Implementation
{
public:
  std::string name;
  std::string charger_name;
  std::string map_name;
  Eigen::Vector3d location;
  double battery_soc;
};

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
double EasyFullControl::RobotState::battery_soc() const
{
  return _pimpl->battery_soc;
}

//==============================================================================
using EasyCommandHandlePtr = std::shared_ptr<EasyCommandHandle>;
class EasyFullControl::Implementation
{
public:
  std::string fleet_name;
  rmf_traffic::Duration update_interval;
  std::shared_ptr<VehicleTraits> traits;
  std::shared_ptr<Graph> graph;
  std::shared_ptr<Adapter> adapter;
  std::shared_ptr<FleetUpdateHandle> fleet_handle;
  // Map robot name to its EasyCommandHandle
  std::unordered_map<std::string, EasyCommandHandlePtr> cmd_handles;
  // TODO(YV): Get these constants from EasyCommandHandle::Configuration
  double max_merge_waypoint_distance = 0.3;
  double max_merge_lane_distance = 0.1;
  double min_lane_length = 1e-8;
};

//==============================================================================
EasyFullControl::EasyFullControl()
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<EasyFullControl> EasyFullControl::make(
  std::optional<Configuration> config_,
  const rclcpp::NodeOptions& options,
  std::optional<rmf_traffic::Duration> discovery_timeout)
{
  auto easy_adapter = std::shared_ptr<EasyFullControl>(new EasyFullControl);
  easy_adapter->_pimpl = rmf_utils::make_unique_impl<Implementation>();
  easy_adapter->_pimpl->adapter = Adapter::make(
    "easy_fleet_adapter",
    options,
    std::move(discovery_timeout));
  if (easy_adapter->_pimpl->adapter == nullptr)
  {
    return nullptr;
  }
  easy_adapter->_pimpl->adapter->start();

  const auto node = easy_adapter->_pimpl->adapter->node();
  auto get_config =
    [](rclcpp::Node::SharedPtr node) -> std::shared_ptr<Configuration>
    {
      const std::string& fleet_name = node->declare_parameter(
        "fleet_name", "tinyRobot");
      RCLCPP_INFO(
        node->get_logger(),
        "Configuring fleet [%s].",
        fleet_name.c_str()
      );

      // Traits and graph
      const double linear_velocity = node->declare_parameter(
        "linear_velocity", 0.7);
      const double linear_acceleration = node->declare_parameter(
        "linear_acceleration", 1.0);
      const double angular_velocity = node->declare_parameter(
        "angular_velocity", 0.5);
      const double angular_acceleration = node->declare_parameter(
        "angular_acceleration", 1.5);
      const double footprint_radius = node->declare_parameter(
        "footprint_radius", 0.3);
      const double vicinity_radius = node->declare_parameter(
        "vicinity_radius", 0.3);
      const bool reversible = node->declare_parameter(
        "reversible", false);
      auto traits = rmf_traffic::agv::VehicleTraits{
        {linear_velocity, linear_acceleration},
        {angular_velocity, angular_acceleration},
        rmf_traffic::Profile{
          rmf_traffic::geometry::make_final_convex<
            rmf_traffic::geometry::Circle>(footprint_radius),
          rmf_traffic::geometry::make_final_convex<
            rmf_traffic::geometry::Circle>(vicinity_radius)
        }
      };
      traits.get_differential()->set_reversible(reversible);
      const std::string nav_graph_file =
        node->declare_parameter("nav_graph_file", "");
      RCLCPP_INFO(
        node->get_logger(),
        "Parsing navgraph: %s",
        nav_graph_file.c_str()
      );
      auto graph =
        rmf_fleet_adapter::agv::parse_graph(nav_graph_file, traits);

      // Battery
      const double battery_voltage = node->declare_parameter(
        "battery_voltage", 24.0);
      const double battery_capacity = node->declare_parameter(
        "battery_capacity", 30.0);
      const double battery_charging_current = node->declare_parameter(
        "battery_charging_current", 5.0);
      const auto battery_opt = rmf_battery::agv::BatterySystem::make(
        battery_voltage, battery_capacity, battery_charging_current);
      if (!battery_opt.has_value())
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Invalid battery parameters");
        return nullptr;
      }
      const auto battery_system =
        std::make_shared<rmf_battery::agv::BatterySystem>(*battery_opt);

      // Sinks
      const double mass = node->declare_parameter(
        "mass", 20.0);
      const double inertia = node->declare_parameter(
        "inertia", 10.0);
      const double friction_coefficient = node->declare_parameter(
        "friction_coefficient", 0.22);
      const auto mechanical_opt = rmf_battery::agv::MechanicalSystem::make(
        mass, inertia, friction_coefficient);
      if (!mechanical_opt.has_value())
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Invalid mechanical parameters");
        return nullptr;
      }
      std::shared_ptr<rmf_battery::agv::SimpleMotionPowerSink> motion_sink =
        std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
        *battery_system, mechanical_opt.value());
      const double ambient_power_drain = node->declare_parameter(
        "ambient_power_drain", 20.0);
      const auto ambient_power_system = rmf_battery::agv::PowerSystem::make(
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
      const double tool_power_drain = node->declare_parameter(
        "tool_power_drain", 0.0);
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
      std::vector<std::string> actions = {"teleop"};
      actions = node->declare_parameter("actions", actions);

      const std::string server_uri_string =
        node->declare_parameter("server_uri", std::string());
      std::optional<std::string> server_uri = std::nullopt;
      if (!server_uri_string.empty())
        server_uri = server_uri_string;

      const double recharge_threshold = node->declare_parameter(
        "recharge_threshold", 0.2);
      const double recharge_soc = node->declare_parameter(
        "recharge_soc", 1.0);
      const bool drain_battery = node->declare_parameter(
        "drain_battery", true);

      const double max_delay = node->declare_parameter(
        "max_delay", 10.0);
      const double update_interval = node->declare_parameter(
        "update_interval", 0.5);

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

      auto config = std::make_shared<Configuration>(
        fleet_name,
        std::move(traits),
        std::move(graph),
        battery_system,
        motion_sink,
        ambient_sink,
        tool_sink,
        recharge_threshold,
        recharge_soc,
        drain_battery,
        actions,
        finishing_request,
        server_uri,
        rmf_traffic::time::from_seconds(max_delay),
        rmf_traffic::time::from_seconds(update_interval));

      return config;
    };

  std::shared_ptr<Configuration> config_from_params = nullptr;
  if (!config_.has_value())
  {
    config_from_params = get_config(node);
    if (!config_from_params)
    {
      RCLCPP_ERROR(
      node->get_logger(),
      "Unable to configure fleet using provided configuration or ROS 2 "
      "parameters"
      );
      return nullptr;
    }
  }

  auto config = config_.has_value() ? config_.value() : *config_from_params;
  easy_adapter->_pimpl->fleet_name = config.fleet_name();
  easy_adapter->_pimpl->update_interval = config.update_interval();
  easy_adapter->_pimpl->traits = std::make_shared<VehicleTraits>(
    config.vehicle_traits());
  easy_adapter->_pimpl->graph = std::make_shared<Graph>(config.graph());

  // Create a FleetUpdateHandle
  easy_adapter->_pimpl->fleet_handle =
    easy_adapter->_pimpl->adapter->add_fleet(
    config.fleet_name(),
    config.vehicle_traits(),
    config.graph(),
    config.server_uri()
    );

  bool ok = easy_adapter->_pimpl->fleet_handle->set_task_planner_params(
    config.battery_system(),
    config.motion_sink(),
    config.ambient_sink(),
    config.tool_sink(),
    config.recharge_threshold(),
    config.recharge_soc(),
    config.account_for_battery_drain(),
    config.finishing_request()
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
    [](const nlohmann::json& /*description*/,
      FleetUpdateHandle::Confirmation& confirm)
    {
      confirm.accept();
    };
  easy_adapter->_pimpl->fleet_handle->consider_delivery_requests(
    consider_all, consider_all);
  easy_adapter->_pimpl->fleet_handle->consider_cleaning_requests(consider_all);
  easy_adapter->_pimpl->fleet_handle->consider_patrol_requests(consider_all);
  easy_adapter->_pimpl->fleet_handle->consider_composed_requests(consider_all);
  for (const std::string& action : config.action_categories())
  {
    easy_adapter->_pimpl->fleet_handle->add_performable_action(action,
      consider_all);
  }

  easy_adapter->_pimpl->fleet_handle->default_maximum_delay(config.max_delay());

  RCLCPP_INFO(
    node->get_logger(),
    "Successfully initialized Full Control adapter for fleet [%s]",
    config.fleet_name().c_str()
  );

  return easy_adapter;
}

//==============================================================================
std::shared_ptr<rclcpp::Node> EasyFullControl::node()
{
  return _pimpl->adapter->node();
}

//==============================================================================
std::shared_ptr<FleetUpdateHandle> EasyFullControl::fleet_handle()
{
  return _pimpl->fleet_handle;
}

//==============================================================================
EasyFullControl& EasyFullControl::wait()
{
  _pimpl->adapter->wait();
  return *this;
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
    node->get_logger(),
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

  auto starts = rmf_traffic::agv::compute_plan_starts(
    *(_pimpl->graph),
    start_state.map_name(),
    start_state.location(),
    std::move(now),
    _pimpl->max_merge_waypoint_distance,
    _pimpl->max_merge_lane_distance,
    _pimpl->min_lane_length
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
      start_state.map_name().c_str(),
      loc[0], loc[1], loc[2]
    );
    return false;
  }

  if (get_state == nullptr ||
    handle_nav_request == nullptr ||
    handle_stop == nullptr ||
    handle_dock == nullptr)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "One more more required callbacks are invalid. The robot will not be "
      "added to the fleet"
    );
    return false;
  }

  insertion.first->second = std::make_shared<EasyCommandHandle>(
    node,
    _pimpl->graph,
    _pimpl->traits,
    robot_name,
    starts[0],
    std::move(start_state),
    std::move(get_state),
    std::move(handle_nav_request),
    std::move(handle_stop),
    std::move(handle_dock),
    std::move(action_executor),
    _pimpl->max_merge_waypoint_distance,
    _pimpl-> max_merge_lane_distance,
    _pimpl-> min_lane_length,
    _pimpl->update_interval
  );

  _pimpl->fleet_handle->add_robot(
    insertion.first->second,
    robot_name,
    _pimpl->traits->profile(),
    {starts[0]},
    [cmd_handle = insertion.first->second](
      const RobotUpdateHandlePtr& updater)
    {
      cmd_handle->set_updater(updater);
    });

  RCLCPP_INFO(
    node->get_logger(),
    "Successfully added robot [%s] to the fleet.", robot_name.c_str()
  );

  return true;
}

//==============================================================================

} // namespace agv
} // namespace rmf_fleet_adapter
