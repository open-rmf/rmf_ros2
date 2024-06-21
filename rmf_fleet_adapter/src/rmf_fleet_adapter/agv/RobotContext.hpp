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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__ROBOTCONTEXT_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__ROBOTCONTEXT_HPP

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/Transformation.hpp>
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rmf_fleet_msgs/msg/mutex_group_manual_release.hpp>

#include <rmf_traffic/schedule/Negotiator.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Mirror.hpp>

#include <rmf_task/State.hpp>
#include <rmf_task/Constraints.hpp>
#include <rmf_task/TaskPlanner.hpp>
#include <rmf_task/Activator.hpp>

#include <rclcpp/node.hpp>

#include <rmf_rxcpp/Publisher.hpp>
#include <rmf_rxcpp/Transport.hpp>
#include <rmf_rxcpp/RxJobs.hpp>

#include <rxcpp/rx-observable.hpp>

#include "Node.hpp"
#include "../Reporting.hpp"

#include <unordered_set>

namespace rmf_fleet_adapter {

// Forward declaration
class TaskManager;

namespace agv {

class FleetUpdateHandle;
class RobotContext;
using TransformDictionary = std::unordered_map<std::string, Transformation>;
using SharedPlanner = std::shared_ptr<
  std::shared_ptr<const rmf_traffic::agv::Planner>>;
using Destination = EasyFullControl::Destination;
using VertexStack = std::shared_ptr<std::unordered_set<std::size_t>>;
using TimeMsg = builtin_interfaces::msg::Time;

//==============================================================================
class EventPrinter : public rmf_traffic::agv::Graph::Lane::Executor
{
public:
  EventPrinter()
  {
    // Do nothing
  }

  void execute(const DoorOpen& e) override
  {
    text = "DoorOpen " + e.name();
  }

  void execute(const DoorClose& e) override
  {
    text = "DoorClose " + e.name();
  }

  void execute(const LiftSessionBegin& e) override
  {
    text = "LiftSessionBegin " + e.lift_name();
  }

  void execute(const LiftDoorOpen& e) override
  {
    text = "LiftDoorOpen " + e.lift_name();
  }

  void execute(const LiftSessionEnd& e) override
  {
    text = "LiftSessionEnd " + e.lift_name();
  }

  void execute(const LiftMove& e) override
  {
    text = "LiftMove " + e.lift_name();
  }

  void execute(const Wait&) override
  {
    text = "Wait";
  }

  void execute(const Dock& dock) override
  {
    text = "Dock " + dock.dock_name();
  }

  std::string text;
};

//==============================================================================
inline std::string print_waypoint(
  const std::size_t i_wp,
  const rmf_traffic::agv::Graph& graph)
{
  std::stringstream ss;
  const rmf_traffic::agv::Graph::Waypoint& wp = graph.get_waypoint(i_wp);

  ss << wp.get_map_name() << " <" << wp.get_location().transpose() << "> ["
     << wp.name_or_index() << "]";
  return ss.str();
}

//==============================================================================
inline std::string print_plan_waypoint(
  const rmf_traffic::agv::Plan::Waypoint& wp,
  const rmf_traffic::agv::Graph& graph,
  const rmf_traffic::Time t0 = rmf_traffic::Time(rmf_traffic::Duration(0)))
{
  std::stringstream ss;
  ss << "t=" << rmf_traffic::time::to_seconds(wp.time() - t0);
  if (wp.graph_index().has_value())
    ss << " #" << *wp.graph_index();
  ss << " <" << wp.position().transpose()
     << "> yaw=" << wp.position()[2] * 180.0 / M_PI;
  if (wp.event())
  {
    EventPrinter event;
    wp.event()->execute(event);
    ss << " event [" << event.text << "]";
  }

  if (wp.graph_index().has_value())
  {
    const auto& m = graph.get_waypoint(*wp.graph_index()).in_mutex_group();
    if (!m.empty())
    {
      ss << " initial mutex [" << m << "]";
    }
  }

  ss << " approach lanes:";
  for (const auto& l : wp.approach_lanes())
  {
    ss << " " << l;
    const auto& lane = graph.get_lane(l);
    const auto& m = lane.properties().in_mutex_group();
    if (!m.empty())
    {
      ss << " [" << m << "]";
    }
  }

  return ss.str();
}

//==============================================================================
inline std::string print_lane_node(
  const rmf_traffic::agv::Graph::Lane::Node& node,
  const rmf_traffic::agv::Graph& graph)
{
  std::stringstream ss;
  EventPrinter event;
  if (node.event())
    node.event()->execute(event);

  ss << "{ " << print_waypoint(node.waypoint_index(), graph);
  if (!event.text.empty())
    ss << " event " << event.text;
  ss << " }";

  return ss.str();
}

//==============================================================================
inline std::string print_lane(
  const std::size_t i_lane,
  const rmf_traffic::agv::Graph& graph)
{
  std::stringstream ss;
  const auto& lane = graph.get_lane(i_lane);
  ss << "lane " << i_lane << ": " << print_lane_node(lane.entry(), graph)
     << " -> " << print_lane_node(lane.exit(), graph);
  return ss.str();
}

//==============================================================================
inline std::string print_starts(
  const rmf_traffic::agv::Plan::StartSet& starts,
  const rmf_traffic::agv::Graph& graph)
{
  std::stringstream ss;
  for (const rmf_traffic::agv::Plan::Start& l : starts)
  {
    ss << "\n -- ";
    if (l.lane().has_value())
    {
      ss << print_lane(*l.lane(), graph);

      const auto& lane = graph.get_lane(*l.lane());
      if (l.waypoint() != lane.exit().waypoint_index())
      {
        ss << " !! MISMATCH BETWEEN KEY WAYPOINT AND LANE EXIT: key "
           << l.waypoint() << " vs exit " << lane.exit().waypoint_index();
      }
    }
    else
    {
      ss << print_waypoint(l.waypoint(), graph);
    }

    if (l.location().has_value())
    {
      ss << " | location <" << l.location()->transpose() << ">";
    }
    else
    {
      ss << " | on waypoint";
    }

    ss << " | orientation " << l.orientation() * 180.0 / M_PI;
  }

  return ss.str();
}

//==============================================================================
struct NavParams
{
  bool skip_rotation_commands;
  std::shared_ptr<TransformDictionary> transforms_to_robot_coords;
  double max_merge_waypoint_distance = 1e-3;
  double max_merge_lane_distance = 0.3;
  double min_lane_length = 1e-8;
  /// We iterate over these multipliers, applying them to the
  /// max_merge_lane_distance until at least one plan start is found.
  std::vector<double> multipliers = {1.0, 2.0, 3.0, 5.0, 10.0};

  std::optional<Eigen::Vector3d> to_rmf_coordinates(
    const std::string& map,
    Eigen::Vector3d position)
  {
    if (!transforms_to_robot_coords)
    {
      return position;
    }

    const auto tf_it = transforms_to_robot_coords->find(map);
    if (tf_it == transforms_to_robot_coords->end())
    {
      return std::nullopt;
    }

    return tf_it->second.apply_inverse(position);
  }

  std::optional<Eigen::Vector3d> to_robot_coordinates(
    const std::string& map,
    Eigen::Vector3d position)
  {
    if (!transforms_to_robot_coords)
      return position;

    const auto tf_it = transforms_to_robot_coords->find(map);
    if (tf_it == transforms_to_robot_coords->end())
    {
      return std::nullopt;
    }

    return tf_it->second.apply(position);
  }

  void search_for_location(
    const std::string& map,
    Eigen::Vector3d position,
    RobotContext& context);

  rmf_traffic::agv::Plan::StartSet compute_plan_starts(
    const rmf_traffic::agv::Graph& graph,
    const std::string& map_name,
    const Eigen::Vector3d position,
    const rmf_traffic::Time start_time) const
  {
    for (const double m : multipliers)
    {
      auto starts = rmf_traffic::agv::compute_plan_starts(
        graph,
        map_name,
        position,
        start_time,
        max_merge_waypoint_distance,
        m * max_merge_lane_distance,
        min_lane_length);

      if (!starts.empty())
        return process_locations(graph, starts);
    }

    return {};
  }

  std::unordered_map<std::size_t, VertexStack> stacked_vertices;

  void find_stacked_vertices(const rmf_traffic::agv::Graph& graph);

  std::string get_vertex_name(
    const rmf_traffic::agv::Graph& graph,
    std::optional<std::size_t> v) const;

  rmf_traffic::agv::Plan::StartSet process_locations(
    const rmf_traffic::agv::Graph& graph,
    rmf_traffic::agv::Plan::StartSet locations) const;

  rmf_traffic::agv::Plan::StartSet _descend_stacks(
    const rmf_traffic::agv::Graph& graph,
    rmf_traffic::agv::Plan::StartSet locations) const;

  // If one of the locations is associated with a lift vertex, filter it out if
  // the actual of the robot is outside the dimensions of the lift.
  rmf_traffic::agv::Plan::StartSet _lift_boundary_filter(
    const rmf_traffic::agv::Graph& graph,
    rmf_traffic::agv::Plan::StartSet locations) const;

  bool in_same_stack(std::size_t wp0, std::size_t wp1) const;
};

//==============================================================================
struct Location
{
  rmf_traffic::Time time;
  std::string map;
  Eigen::Vector3d position;
};

//==============================================================================
/// Store position information when the robot is lost, i.e. it has diverged too
/// far from its navigation graph.
struct Lost
{
  /// We may have localization information for the robot, even if it's too far
  /// from the navigation graph.
  std::optional<Location> location;

  /// An issue ticket to track when the robot is lost
  std::unique_ptr<Reporting::Ticket> ticket;
};

//==============================================================================
struct LiftDestination
{
  std::string lift_name;
  std::string destination_floor;
  bool requested_from_inside;

  inline bool matches(
    const std::string& desired_lift_name,
    const std::string& desired_floor) const
  {
    return desired_lift_name == lift_name && desired_floor == destination_floor;
  }
};

//==============================================================================
struct MutexGroupData
{
  std::string name;
  TimeMsg claim_time;
};

//==============================================================================
struct MutexGroupSwitch
{
  std::string from;
  std::string to;
  std::function<bool()> accept;
};

//==============================================================================
class RobotContext
  : public std::enable_shared_from_this<RobotContext>,
  public rmf_traffic::schedule::Negotiator
{
public:

  /// Get a handle to the command interface of the robot. This may return a
  /// nullptr if the robot has disconnected and/or its command API is no longer
  /// available.
  std::shared_ptr<RobotCommandHandle> command();

  /// Create a RobotUpdateHandle that reports to this RobotContext
  std::shared_ptr<RobotUpdateHandle> make_updater();

  /// This is the robot's current (x, y, yaw) position.
  Eigen::Vector3d position() const;

  /// This is the map that the robot is on.
  const std::string& map() const;

  /// Get the current time
  rmf_traffic::Time now() const;

  /// Get a clock that can be used by task loggers
  std::function<rmf_traffic::Time()> clock() const;

  /// This is the current "location" of the robot, which can be used to initiate
  /// a planning job
  const rmf_traffic::agv::Plan::StartSet& location() const;

  /// Set the current location for the robot in terms of a planner start set
  void set_location(rmf_traffic::agv::Plan::StartSet location_);

  /// If the robot is lost, this will let you view its localization and issue
  /// ticket
  const std::optional<Lost>& lost() const;

  /// Set that the robot is currently lost
  void set_lost(std::optional<Location> location);

  /// Filter closed lanes out of the planner start set. At least one start will
  /// be retained so that the planner can offer some solution, even if all
  /// plan starts are using closed lanes.
  void filter_closed_lanes();

  /// Get the current lane closures for the robot
  const rmf_traffic::agv::LaneClosure* get_lane_closures() const;

  /// Get a mutable reference to the schedule of this robot
  rmf_traffic::schedule::Participant& itinerary();

  /// Get a const-reference to the schedule of this robot
  const rmf_traffic::schedule::Participant& itinerary() const;

  using Mirror = rmf_traffic::schedule::Mirror;
  /// Get a const-reference to an interface that lets you get a snapshot of the
  /// schedule.
  const std::shared_ptr<const Mirror>& schedule() const;

  /// Get the schedule description of this robot
  const rmf_traffic::schedule::ParticipantDescription& description() const;

  /// Get the profile of this robot
  const std::shared_ptr<const rmf_traffic::Profile>& profile() const;

  /// Get the name of this robot
  const std::string& name() const;

  /// Get the group (fleet) of this robot
  const std::string& group() const;

  /// Get the requester ID to use for this robot when sending requests
  const std::string& requester_id() const;

  /// Get the traffic participant ID for this robot
  rmf_traffic::ParticipantId participant_id() const;

  /// Get the navigation graph used by this robot
  const rmf_traffic::agv::Graph& navigation_graph() const;

  /// Get an immutable reference to the planner for this robot
  const std::shared_ptr<const rmf_traffic::agv::Planner>& planner() const;

  /// Get an immutable reference to the planner to use for this robot during
  /// emergencies (fire alarm).
  const std::shared_ptr<const rmf_traffic::agv::Planner>& emergency_planner()
  const;

  /// Get the navigation params for this robot, if it has any. This will only be
  /// available for EasyFullControl robots.
  std::shared_ptr<NavParams> nav_params() const;

  /// Set the navigation params for this robot. This is used by EasyFullControl
  /// robots.
  void set_nav_params(std::shared_ptr<NavParams> value);

  class NegotiatorLicense;

  /// Set the schedule negotiator that will take responsibility for this robot.
  /// Hold onto the returned subscription to remain the negotiator for this
  /// robot.
  std::shared_ptr<NegotiatorLicense> set_negotiator(
    rmf_traffic::schedule::Negotiator* negotiator);

  /// This function will indicate that GoToPlace should have a stubborn
  /// negotiation behavior for as long as the returned handle is alive.
  std::shared_ptr<void> be_stubborn();

  /// If anything is holding onto a be_stubborn handle, this will return true.
  bool is_stubborn() const;

  struct Empty {};
  /// Use this to get notified when
  const rxcpp::observable<Empty>& observe_replan_request() const;

  /// Use this to get notified when the charging policy changes for this robot.
  const rxcpp::observable<Empty>& observe_charging_change() const;

  /// Request this robot to replan
  void request_replan();

  struct GraphChange
  {
    std::vector<std::size_t> closed_lanes;
  };
  const rxcpp::observable<GraphChange>& observe_graph_change() const;

  /// Notify this robot that the graph has changed.
  void notify_graph_change(GraphChange changes);

  /// Get a reference to the rclcpp node
  const std::shared_ptr<Node>& node();

  /// const-qualified node()
  std::shared_ptr<const Node> node() const;

  /// Get a reference to the worker for this robot. Use this worker to observe
  /// callbacks that can modify the state of the robot.
  const rxcpp::schedulers::worker& worker() const;

  /// Get the maximum allowable delay for this robot
  rmf_utils::optional<rmf_traffic::Duration> maximum_delay() const;

  /// Set the maximum allowable delay for this robot
  RobotContext& maximum_delay(rmf_utils::optional<rmf_traffic::Duration> value);

  // Documentation inherited from rmf_traffic::schedule::Negotiator
  void respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder) final;

  /// Get the task activator for this robot
  const rmf_task::ConstActivatorPtr& task_activator() const;

  const rmf_task::ConstParametersPtr& task_parameters() const;

  /// Set the state of this robot at the end of its current task
  RobotContext& current_task_end_state(const rmf_task::State& state);

  /// Get a mutable reference to the state of this robot at the end of its
  // current task
  const rmf_task::State& current_task_end_state() const;

  /// Make a way to get the state for this robot
  std::function<rmf_task::State()> make_get_state();

  /// Get the current task ID of the robot, or a nullptr if the robot is not
  /// performing any task
  const std::string* current_task_id() const;

  /// Set the current task ID of the robot, or give a nullopt if a task is not
  /// being performed.
  RobotContext& current_task_id(std::optional<std::string> id);

  /// Get a string copy of the current task ID of the robot, or an empty string
  /// if the robot is not performing any task
  std::string copy_current_task_id() const;

  /// Get the current battery state of charge
  double current_battery_soc() const;

  /// Set the current battery state of charge. Note: This function also
  /// publishes the battery soc via _battery_soc_publisher.
  RobotContext& current_battery_soc(const double battery_soc);

  /// The waypoint dedicated to this robot that it should go to if it needs to
  /// charge. This may be a waypoint that has a charger or it may be a parking
  /// spot where the robot should wait until a charger becomes available. Use
  /// waiting_for_charger() to determine which.
  std::size_t dedicated_charging_wp() const;

  /// When the robot reaches its dedicated_charging_wp, is it there to wait for
  /// a charger to become available (true) or to actually perform the charging
  /// (false)?
  bool waiting_for_charger() const;

  /// This function will indicate that the robot is currently charging for as
  /// long as the return value is held onto.
  std::shared_ptr<void> be_charging();

  /// Check if the robot is currently doing a battery charging task.
  bool is_charging() const;

  // Get a reference to the battery soc observer of this robot.
  const rxcpp::observable<double>& observe_battery_soc() const;

  /// Get a mutable reference to the task planner for this robot
  const std::shared_ptr<const rmf_task::TaskPlanner>& task_planner() const;

  /// Set the task planner for this robot
  RobotContext& task_planner(
    const std::shared_ptr<const rmf_task::TaskPlanner> task_planner);

  void set_lift_entry_watchdog(
    RobotUpdateHandle::Unstable::Watchdog watchdog,
    rmf_traffic::Duration wait_duration);

  const RobotUpdateHandle::Unstable::Watchdog& get_lift_watchdog() const;

  rmf_traffic::Duration get_lift_rewait_duration() const;

  /// Set the current mode of the robot. This mode should correspond to a
  /// constant in the RobotMode message
  [[deprecated]]
  void current_mode(uint32_t mode);

  /// Return the current mode of the robot
  [[deprecated]]
  uint32_t current_mode() const;

  /// Set the current mode of the robot.
  /// Specify a valid string as specified in the robot_state.json schema
  void override_status(std::optional<std::string> status);

  /// Return the current mode of the robot
  std::optional<std::string> override_status() const;

  /// Set the action executor for requesting this robot to execute a
  /// PerformAction activity
  void action_executor(RobotUpdateHandle::ActionExecutor action_executor);

  /// Get the action executor for requesting this robot to execute a
  /// PerformAction activity
  RobotUpdateHandle::ActionExecutor action_executor() const;

  /// Get the task manager for this robot, if it exists.
  std::shared_ptr<TaskManager> task_manager();

  /// Set the commission for this robot
  void set_commission(RobotUpdateHandle::Commission value);

  /// Get a reference to the robot's commission.
  const RobotUpdateHandle::Commission& commission() const;

  /// Lock the commission_mutex and return a copy of the robot's current
  /// commission.
  RobotUpdateHandle::Commission copy_commission() const;

  /// Reassign the tasks that have been dispatched for this robot
  void reassign_dispatched_tasks();

  Reporting& reporting();

  const Reporting& reporting() const;

  /// Tell the robot to localize near here
  bool localize(
    EasyFullControl::Destination estimate,
    EasyFullControl::CommandExecution execution) const;

  /// Set the callback for localizing the robot
  void set_localization(EasyFullControl::LocalizationRequest localization);

  /// Get the current lift destination request for this robot
  const LiftDestination* current_lift_destination() const;

  /// Ask for a certain lift to go to a certain destination and open the doors
  std::shared_ptr<void> set_lift_destination(
    std::string lift_name,
    std::string destination_floor,
    bool requested_from_inside);

  /// Indicate that the lift is no longer needed
  void release_lift();

  /// Check if a door is being held
  const std::optional<std::string>& holding_door() const;

  /// What mutex groups are currently locked by this robot.
  const std::unordered_map<std::string, TimeMsg>& locked_mutex_groups() const;

  /// What mutex groups are currently being requested (but have not yet been
  /// locked) by this robot.
  const std::unordered_map<std::string, TimeMsg>&
  requesting_mutex_groups() const;

  /// Set the mutex group that this robot needs to lock.
  const rxcpp::observable<std::string>& request_mutex_groups(
    std::unordered_set<std::string> groups,
    rmf_traffic::Time claim_time);

  /// Retain only the mutex groups listed in the set. Release all others.
  void retain_mutex_groups(const std::unordered_set<std::string>& groups);

  void schedule_itinerary(
    std::shared_ptr<rmf_traffic::PlanId> plan_id,
    rmf_traffic::schedule::Itinerary itinerary);

  void schedule_hold(
    std::shared_ptr<rmf_traffic::PlanId> plan_id,
    rmf_traffic::Time time,
    rmf_traffic::Duration wait,
    Eigen::Vector3d position,
    const std::string& map);

  /// Set the task manager for this robot. This should only be called in the
  /// TaskManager::make function.
  void _set_task_manager(std::shared_ptr<TaskManager> mgr);

  /// Set the negotiation license for this robot. This should only be called in
  /// the FleetUpdateHandle::add_robot function.
  void _set_negotiation_license(std::shared_ptr<void> license);

  /// Use this to trigger emergencies on/off. This should only be called in the
  /// FleetUpdateHandle::handle_emergency function.
  void _set_emergency(bool value);

  /// Use this to change the charging settings for the robot and trigger a
  /// charger change notification.
  void _set_charging(std::size_t wp, bool waiting_for_charger);

  /// Request a door to stay open. This should only be used by DoorOpen.
  void _hold_door(std::string door_name);

  /// Release a door. This should only be used by DoorClose
  void _release_door(const std::string& door_name);

  template<typename... Args>
  static std::shared_ptr<RobotContext> make(Args&&... args)
  {
    auto context = std::shared_ptr<RobotContext>(
      new RobotContext(std::forward<Args>(args)...));

    context->_lift_subscription = context->_node->lift_state()
      .observe_on(rxcpp::identity_same_worker(context->_worker))
      .subscribe([w = context->weak_from_this()](const auto& msg)
        {
          const auto self = w.lock();
          if (!self)
            return;

          self->_check_lift_state(*msg);
        });

    context->_door_subscription = context->_node->door_supervisor()
      .observe_on(rxcpp::identity_same_worker(context->_worker))
      .subscribe([w = context->weak_from_this()](const auto& msg)
        {
          const auto self = w.lock();
          if (!self)
            return;

          self->_check_door_supervisor(*msg);
        });

    context->_mutex_group_sanity_check = context->_node->mutex_group_states()
      .observe_on(rxcpp::identity_same_worker(context->_worker))
      .subscribe([w = context->weak_from_this()](const auto& msg)
        {
          const auto self = w.lock();
          if (!self)
            return;

          self->_check_mutex_groups(*msg);
        });

    context->_mutex_group_heartbeat = context->_node->try_create_wall_timer(
      std::chrono::seconds(2),
      [w = context->weak_from_this()]()
      {
        const auto self = w.lock();
        if (!self)
          return;

        self->_publish_mutex_group_requests();
      });

    context->_mutex_group_manual_release_sub =
      context->_node->create_subscription<
      rmf_fleet_msgs::msg::MutexGroupManualRelease>(
      MutexGroupManualReleaseTopicName,
      rclcpp::SystemDefaultsQoS()
      .reliable()
      .keep_last(10),
      [w = context->weak_from_this()](
        rmf_fleet_msgs::msg::MutexGroupManualRelease::SharedPtr msg)
      {
        if (const auto self = w.lock())
          self->_handle_mutex_group_manual_release(*msg);
      });

    return context;
  }

  bool debug_positions = false;

private:

  RobotContext(
    std::shared_ptr<RobotCommandHandle> command_handle,
    std::vector<rmf_traffic::agv::Plan::Start> _initial_location,
    rmf_traffic::schedule::Participant itinerary,
    std::shared_ptr<const Mirror> schedule,
    SharedPlanner planner,
    SharedPlanner emergency_planner,
    rmf_task::ConstActivatorPtr activator,
    rmf_task::ConstParametersPtr parameters,
    std::shared_ptr<Node> node,
    const rxcpp::schedulers::worker& worker,
    rmf_utils::optional<rmf_traffic::Duration> maximum_delay,
    rmf_task::State state,
    std::shared_ptr<const rmf_task::TaskPlanner> task_planner);

  std::weak_ptr<RobotCommandHandle> _command_handle;
  std::vector<rmf_traffic::agv::Plan::Start> _location;
  std::vector<rmf_traffic::agv::Plan::Start> _most_recent_valid_location;
  rmf_traffic::schedule::Participant _itinerary;
  std::shared_ptr<const Mirror> _schedule;
  SharedPlanner _planner;
  SharedPlanner _emergency_planner;
  std::shared_ptr<NavParams> _nav_params;
  rmf_task::ConstActivatorPtr _task_activator;
  rmf_task::ConstParametersPtr _task_parameters;
  std::shared_ptr<const rmf_traffic::Profile> _profile;

  std::shared_ptr<void> _negotiation_license;
  std::shared_ptr<void> _stubbornness;

  rxcpp::subjects::subject<Empty> _replan_publisher;
  rxcpp::observable<Empty> _replan_obs;

  rxcpp::subjects::subject<Empty> _charging_change_publisher;
  rxcpp::observable<Empty> _charging_change_obs;

  rxcpp::subjects::subject<GraphChange> _graph_change_publisher;
  rxcpp::observable<GraphChange> _graph_change_obs;

  std::shared_ptr<Node> _node;
  rxcpp::schedulers::worker _worker;
  rmf_utils::optional<rmf_traffic::Duration> _maximum_delay;
  std::string _requester_id;

  rmf_traffic::schedule::Negotiator* _negotiator = nullptr;

  /// Always call the current_battery_soc() setter to set a new value
  double _current_battery_soc = 1.0;
  std::size_t _charging_wp;
  /// When the robot reaches its _charging_wp, is there to wait for a charger
  /// (true) or to actually charge (false)?
  bool _waiting_for_charger = false;
  std::shared_ptr<void> _lock_charging;
  rxcpp::subjects::subject<double> _battery_soc_publisher;
  rxcpp::observable<double> _battery_soc_obs;
  rmf_task::State _current_task_end_state;
  std::optional<std::string> _current_task_id;
  std::unique_ptr<std::mutex> _current_task_id_mutex =
    std::make_unique<std::mutex>();
  std::shared_ptr<const rmf_task::TaskPlanner> _task_planner;
  std::weak_ptr<TaskManager> _task_manager;

  RobotUpdateHandle::Unstable::Watchdog _lift_watchdog;
  rmf_traffic::Duration _lift_rewait_duration = std::chrono::seconds(0);
  std::unique_ptr<std::mutex> _commission_mutex =
    std::make_unique<std::mutex>();
  RobotUpdateHandle::Commission _commission;
  bool _emergency = false;
  EasyFullControl::LocalizationRequest _localize;

  // Mode value for RobotMode message
  uint32_t _current_mode;
  std::optional<std::string> _override_status;

  RobotUpdateHandle::ActionExecutor _action_executor;
  Reporting _reporting;
  /// Keep track of a lost robot
  std::optional<Lost> _lost;

  void _check_lift_state(const rmf_lift_msgs::msg::LiftState& state);
  void _publish_lift_destination();
  std::shared_ptr<LiftDestination> _lift_destination;
  rmf_rxcpp::subscription_guard _lift_subscription;
  std::optional<std::chrono::steady_clock::time_point>
  _initial_time_idle_outside_lift;
  std::shared_ptr<void> _lift_stubbornness;
  bool _lift_arrived = false;

  void _check_door_supervisor(
    const rmf_door_msgs::msg::SupervisorHeartbeat& hb);
  std::optional<std::string> _holding_door;
  rmf_rxcpp::subscription_guard _door_subscription;

  void _check_mutex_groups(const rmf_fleet_msgs::msg::MutexGroupStates& states);
  void _retain_mutex_groups(
    const std::unordered_set<std::string>& retain,
    std::unordered_map<std::string, TimeMsg>& _groups);
  void _release_mutex_group(const MutexGroupData& data) const;
  void _publish_mutex_group_requests();
  void _handle_mutex_group_manual_release(
    const rmf_fleet_msgs::msg::MutexGroupManualRelease& msg);
  std::unordered_map<std::string, TimeMsg> _requesting_mutex_groups;
  std::unordered_map<std::string, TimeMsg> _locked_mutex_groups;
  rxcpp::subjects::subject<std::string> _mutex_group_lock_subject;
  rxcpp::observable<std::string> _mutex_group_lock_obs;
  rclcpp::TimerBase::SharedPtr _mutex_group_heartbeat;
  rmf_rxcpp::subscription_guard _mutex_group_sanity_check;
  rclcpp::Subscription<rmf_fleet_msgs::msg::MutexGroupManualRelease>::SharedPtr
    _mutex_group_manual_release_sub;
  std::chrono::steady_clock::time_point _last_active_task_time;
};

using RobotContextPtr = std::shared_ptr<RobotContext>;
using ConstRobotContextPtr = std::shared_ptr<const RobotContext>;

//==============================================================================
struct GetContext
{
  RobotContextPtr value;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__ROBOTCONTEXT_HPP
