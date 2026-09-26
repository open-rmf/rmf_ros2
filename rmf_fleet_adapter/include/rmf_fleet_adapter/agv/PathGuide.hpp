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

#ifndef RMF_FLEET_ADAPTER__AGV__PATHGUIDE_HPP
#define RMF_FLEET_ADAPTER__AGV__PATHGUIDE_HPP

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>

// rmf_fleet_adapter headers
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/Transformation.hpp>

// rmf_battery headers
#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/MechanicalSystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

// rmf_traffic headers
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

// rmf_task headers
#include <rmf_task/TaskPlanner.hpp>

#include <rmf_utils/impl_ptr.hpp>

// System headers
#include <Eigen/Geometry>

namespace rmf_fleet_adapter {
namespace agv {

/// A full_control fleet adapter that hands the downstream integrator an entire
/// path at once, instead of one destination at a time.
///
/// PathGuide is a sibling of EasyFullControl, not a subclass. It offers the same
/// feature set — traffic schedule participation and negotiation, schedule
/// overrides, lifts, docking, coordinate transformations, YAML fleet
/// configuration, responsive wait and battery-aware task planning — and differs
/// only in how navigation work reaches the integrator:
///
/// * EasyFullControl issues one `NavigationRequest` per graph vertex and waits
///   for `CommandExecution::finished()` before revealing the next one.
/// * PathGuide issues a single `PathRequest` carrying every remaining waypoint,
///   each bundled with its own `CommandExecution`. The integrator acknowledges
///   waypoints as the robot reaches them.
///
/// This lets a fleet manager smooth, spline or velocity-profile across the whole
/// route, and removes the round trip that EasyFullControl pays at every vertex.
///
/// \par Acknowledgement ordering
/// The integrator **must** acknowledge waypoints strictly in order, from the
/// waypoint PathGuide is currently tracking through to the last. A
/// `finished()` call on any other waypoint is rejected with a warning and the
/// path does not advance. If the robot physically passes a vertex without
/// noticing, acknowledge the missed waypoints in sequence before the one it
/// actually reached; several acknowledgements in one update cycle are fine.
///
/// By default the adapter will be configured to accept all tasks. To disable
/// specific tasks, call the respective consider_*_requests() method on the
/// FleetUpdateHandle that can be accessed within this adapter.
//==============================================================================
class PathGuide : public std::enable_shared_from_this<PathGuide>
{
public:

  // Aliases
  using Graph = rmf_traffic::agv::Graph;
  using VehicleTraits = rmf_traffic::agv::VehicleTraits;
  using ActionExecutor = RobotUpdateHandle::ActionExecutor;
  using ActivityIdentifier = RobotUpdateHandle::ActivityIdentifier;
  using ActivityIdentifierPtr = RobotUpdateHandle::ActivityIdentifierPtr;
  using ConstActivityIdentifierPtr =
    RobotUpdateHandle::ConstActivityIdentifierPtr;
  using Stubbornness = RobotUpdateHandle::Unstable::Stubbornness;
  using ConsiderRequest = FleetUpdateHandle::ConsiderRequest;
  using AssignmentStrategy = rmf_task::TaskPlanner::TaskAssignmentStrategy;

  // Nested class declarations
  class PathRobotUpdateHandle;
  class RobotState;
  class RobotConfiguration;
  class RobotCallbacks;
  class Destination;
  class PathWaypoint;
  class Path;
  class FleetConfiguration;
  class CommandExecution;

  /// Signature for a function that handles path requests. The request contains
  /// every waypoint the robot should visit, in order.
  ///
  /// \param[in] path
  ///   The path for the robot to follow. Acknowledge each waypoint in order by
  ///   calling `finished()` on its `CommandExecution` once the robot is inside
  ///   that waypoint's `merge_radius()`.
  ///
  /// \note A new path request supersedes any previous one. The
  /// `CommandExecution`s belonging to a superseded path stop being `okay()`.
  using PathRequest = std::function<void(Path path)>;

  /// Signature for a function to handle stop requests.
  using StopRequest = std::function<void(ConstActivityIdentifierPtr)>;

  /// Signature for a function that handles localization requests. The request
  /// will specify an approximate location for the robot.
  ///
  /// \param[in] location_estimate
  ///   An estimate for where the robot is currently located.
  ///
  /// \param[in] execution
  ///   The command execution progress updater. Use this to keep the fleet
  ///   adapter updated on the progress of localizing.
  ///
  /// \note Localization is a single pose, not a path, so this keeps the
  /// destination-shaped signature.
  using LocalizationRequest = std::function<void(
        Destination location_estimate,
        CommandExecution execution)>;

  /// Add a robot to the fleet once it is available.
  ///
  /// \param[in] name
  ///   Name of the robot. This must be unique per fleet.
  ///
  /// \param[in] initial_state
  ///   The initial state of the robot when it is added to the fleet.
  ///
  /// \param[in] configuration
  ///   The configuration of the robot.
  ///
  /// \param[in] callbacks
  ///   The callbacks that will be used to issue commands for the robot.
  ///
  /// \return a path robot update handle on success. A nullptr if an error
  /// occurred.
  std::shared_ptr<PathRobotUpdateHandle> add_robot(
    std::string name,
    RobotState initial_state,
    RobotConfiguration configuration,
    RobotCallbacks callbacks);

  /// Get the FleetUpdateHandle that this adapter will be using.
  /// This may be used to perform more specialized customizations using the
  /// base FleetUpdateHandle API.
  std::shared_ptr<FleetUpdateHandle> more();

  /// Immutable reference to the base FleetUpdateHandle API.
  std::shared_ptr<const FleetUpdateHandle> more() const;

  class Implementation;
private:
  PathGuide();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using PathGuidePtr = std::shared_ptr<PathGuide>;

/// Handle used to update information about one robot
class PathGuide::PathRobotUpdateHandle
{
public:
  /// Recommended function for updating information about a robot in a PathGuide
  /// fleet.
  ///
  /// \param[in] state
  ///   The current state of the robot
  ///
  /// \param[in] current_activity
  ///   The activity that the robot is currently executing. While following a
  ///   path this is the identifier of the waypoint currently being approached,
  ///   i.e. the first waypoint that has not been acknowledged yet.
  void update(
    RobotState state,
    ConstActivityIdentifierPtr current_activity);

  /// Get the maximum allowed merge waypoint distance for this robot.
  double max_merge_waypoint_distance() const;

  /// Modify the maximum allowed merge distance between the robot and a waypoint.
  ///
  /// \param[in] distance
  ///   The maximum merge waypoint distance for this robot.
  void set_max_merge_waypoint_distance(double distance);

  /// Get the maximum allowed merge lane distance for this robot.
  double max_merge_lane_distance() const;

  /// Modify the maximum allowed merge distance between the robot and a lane.
  ///
  /// \param[in] distance
  ///   The maximum merge lane distance for this robot.
  void set_max_merge_lane_distance(double distance);

  /// Get the minimum lane length for this robot.
  double min_lane_length() const;

  /// Modify the minimum lane length for this robot.
  ///
  /// \param[in] length
  ///   The minimum length of a lane.
  void set_min_lane_length(double length);

  /// Get more options for updating the robot's state
  std::shared_ptr<RobotUpdateHandle> more();

  /// Immutable reference to the base robot update API
  std::shared_ptr<const RobotUpdateHandle> more() const;

  class Implementation;
private:
  PathRobotUpdateHandle();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

/// The current state of a robot, passed into PathRobotUpdateHandle::update
class PathGuide::RobotState
{
public:
  /// Constructor
  ///
  /// \param[in] map_name
  ///   The name of the map the robot is currently on
  ///
  /// \param[in] position
  ///   The current position of the robot
  ///
  /// \param[in] battery_soc
  ///   the current battery level of the robot, specified by its state of
  ///   charge as a fraction of its total charge capacity, i.e. a value from 0.0
  ///   to 1.0.
  RobotState(
    std::string map_name,
    Eigen::Vector3d position,
    double battery_soc);

  /// Current map the robot is on
  const std::string& map() const;

  /// Set the current map the robot is on
  void set_map(std::string value);

  /// Current position of the robot
  Eigen::Vector3d position() const;

  /// Set the current position of the robot
  void set_position(Eigen::Vector3d value);

  /// Current state of charge of the battery, as a fraction from 0.0 to 1.0.
  double battery_state_of_charge() const;

  /// Set the state of charge of the battery, as a fraction from 0.0 to 1.0.
  void set_battery_state_of_charge(double value);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// The configuration of a robot. These are parameters that typically do not
/// change over time.
class PathGuide::RobotConfiguration
{
public:

  /// Constructor
  ///
  /// \param[in] compatible_chargers
  ///   List of chargers that this robot is compatible with
  ///
  /// \param[in] responsive_wait
  ///   Should this robot use the responsive wait behavior? true / false / fleet
  ///   default.
  ///
  /// \warning This must contain a single string value until a later release of
  /// RMF. We are using a vector for forward API compatibility. For now, make
  /// sure each robot has only one unique compatible charger to avoid charging
  /// conflicts.
  RobotConfiguration(
    std::vector<std::string> compatible_chargers,
    std::optional<bool> responsive_wait = std::nullopt,
    std::optional<double> max_merge_waypoint_distance = 1e-3,
    std::optional<double> max_merge_lane_distance = 0.3,
    std::optional<double> min_lane_length = 1e-8);

  /// List of chargers that this robot is compatible with
  const std::vector<std::string>& compatible_chargers() const;

  /// Set the list of chargers compatible with this robot.
  void set_compatible_chargers(std::vector<std::string> chargers);

  /// Should this robot use the responsive wait behavior? Responsive wait means
  /// that when the robot is idle on a point, it will report to the traffic
  /// schedule that it is waiting on that point, and it will negotiate with
  /// other robots to let them pass while ultimately remaining on the point.
  ///
  /// If std::nullopt is used, then the fleet-wide responsive wait behavior will
  /// be used.
  std::optional<bool> responsive_wait() const;

  /// Toggle responsive wait on (true), off (false), or use fleet default
  /// (std::nullopt).
  void set_responsive_wait(std::optional<bool> enable);

  /// Get the maximum merge distance between a robot and a waypoint. This refers
  /// to the maximum distance allowed to consider a robot to be on a particular
  /// waypoint.
  ///
  /// If std::nullopt is used, then the fleet-wide default merge waypoint
  /// distance will be used.
  std::optional<double> max_merge_waypoint_distance() const;

  /// Set the maximum merge distance between a robot and a waypoint.
  void set_max_merge_waypoint_distance(std::optional<double> distance);

  /// Get the maximum merge distance between a robot and a lane. This refers
  /// to the maximum distance allowed to consider a robot to be on a particular
  /// lane.
  ///
  /// If std::nullopt is used, then the fleet-wide default merge lane
  /// distance will be used.
  std::optional<double> max_merge_lane_distance() const;

  /// Set the maximum merge distance between a robot and a lane.
  void set_max_merge_lane_distance(std::optional<double> distance);

  /// Get the minimum lane length.
  ///
  /// If std::nullopt is used, then the fleet-wide default minimum lane length
  /// will be used.
  std::optional<double> min_lane_length() const;

  /// Set the minimum lane length.
  void set_min_lane_length(std::optional<double> distance);

  /// Get the idle behavior.
  ///
  /// If std::nullopt is used, then the fleet-wide default finishing request
  /// will be used.
  std::optional<rmf_task::ConstRequestFactoryPtr> finishing_request() const;

  /// Set the finishing request.
  void set_finishing_request(
    std::optional<rmf_task::ConstRequestFactoryPtr> request);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

class PathGuide::RobotCallbacks
{
public:

  /// Constructor
  ///
  /// \param[in] follow_path
  ///   A function that receives the entire path for the robot to follow.
  ///
  /// \param[in] stop
  ///   A function to stop the robot.
  ///
  /// \param[in] action_executor
  ///   The ActionExecutor callback to request the robot to perform an action.
  RobotCallbacks(
    PathRequest follow_path,
    StopRequest stop,
    ActionExecutor action_executor);

  /// Get the callback for following a path
  PathRequest follow_path() const;

  /// Get the callback for stopping
  StopRequest stop() const;

  /// Get the action executor.
  ActionExecutor action_executor() const;

  /// Give the robot a localization callback. Unlike the callbacks used by the
  /// constructor, this callback is optional.
  RobotCallbacks& with_localization(LocalizationRequest localization);

  /// Get the callback for localizing if available.
  LocalizationRequest localize() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// Used by system integrators to give feedback on the progress of reaching one
/// waypoint of a path, or of a localization request.
class PathGuide::CommandExecution
{
public:

  /// Trigger this when the robot has reached the waypoint that this execution
  /// belongs to. No other function in this CommandExecution instance will be
  /// usable after this.
  ///
  /// \warning Waypoints must be acknowledged in order. Calling this on a
  /// waypoint that PathGuide is not currently tracking logs a warning and does
  /// nothing.
  void finished();

  /// Returns false if the command has been stopped, already finished, or
  /// belongs to a path that has been superseded by a replan.
  bool okay() const;

  /// Use this to override the traffic schedule for the agent while it performs
  /// this command.
  ///
  /// If the given trajectory results in a traffic conflict then a negotiation
  /// will be triggered. Hold onto the `Stubbornness` returned by this function
  /// to ask other agents to plan around your trajectory, otherwise the
  /// negotiation may result in a replan for this agent and a new command will
  /// be issued.
  ///
  /// \note Using this will function always trigger a replan once the agent
  /// finishes the command.
  ///
  /// \warning Too many overridden/stubborn agents can cause a deadlock. It's
  ///   recommended to use this API sparingly and only over short distances or
  ///   small deviations.
  ///
  /// \param[in] map
  ///   Name of the map where the trajectory will take place
  ///
  /// \param[in] path
  ///   The path of the agent
  ///
  /// \param[in] hold
  ///   How long the agent will wait at the end of the path
  ///
  /// \return a Stubbornness handle that tells the fleet adapter to not let the
  /// overridden path be negotiated. The returned handle will stop having an
  /// effect after this command execution is finished.
  Stubbornness override_schedule(
    std::string map,
    std::vector<Eigen::Vector3d> path,
    rmf_traffic::Duration hold = rmf_traffic::Duration(0));

  /// Activity handle for this command. Pass this into
  /// PathRobotUpdateHandle::update while the robot is approaching this
  /// waypoint.
  ConstActivityIdentifierPtr identifier() const;

  class Implementation;
private:
  CommandExecution();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

class PathGuide::Destination
{
public:
  /// The name of the map where the destination is located.
  const std::string& map() const;

  /// The (x, y, yaw) position of the destination.
  Eigen::Vector3d position() const;

  /// The (x, y) position of the destination.
  Eigen::Vector2d xy() const;

  /// The intended orientation of the robot at the destination, represented in
  /// radians.
  double yaw() const;

  /// If the destination has an index in the navigation graph, you can get it
  /// from this field.
  std::optional<std::size_t> graph_index() const;

  /// The name of this destination, if it has one. Nameless destinations will
  /// give an empty string.
  std::string name() const;

  /// If there is a speed limit that should be respected while approaching the
  /// destination, this will indicate it.
  std::optional<double> speed_limit() const;

  /// If the destination should be reached by performing a dock maneuver, this
  /// will contain the name of the dock.
  std::optional<std::string> dock() const;

  /// Get whether the destination is inside of a lift, and if so get the
  /// properties of the lift.
  ///
  /// Non-null exactly when this waypoint is inside a lift cabin. **You are
  /// expected to use this.** PathGuide does not tighten `merge_radius()`
  /// inside lifts — deciding when the robot is far enough in for the doors to
  /// close safely is yours to make, because only you know your true footprint
  /// shape, your heading on arrival, and your controller's stopping accuracy.
  /// Acknowledging a lift waypoint on `merge_radius()` alone will close the
  /// doors on a robot that is still partly outside; see `merge_radius()` for
  /// the incident that demonstrates it.
  ///
  /// \warning **Coordinate frames differ within this object.** The returned
  /// LiftProperties — `location()`, `orientation()`, `dimensions()`, and the
  /// positions `is_in_lift()` expects — are all in **RMF canonical
  /// coordinates**, straight from the navigation graph. But `position()`,
  /// `xy()` on this Destination, and `merge_radius()` on the PathWaypoint
  /// that carries it, are in **robot
  /// coordinates**, having been through the fleet's configured transform.
  ///
  /// So `inside_lift()->is_in_lift(my_robot_frame_position)` is **wrong** on
  /// any fleet that configures a `transforms` entry, and wrong in the
  /// dangerous direction: it silently reports a plausible answer. It happens to
  /// work when the transform is identity, which is how it escapes notice.
  ///
  /// Either convert your position back to RMF coordinates before calling
  /// `is_in_lift()`, or apply your own transform to the cabin geometry. If you
  /// would rather not deal with frames at all, judging your distance to
  /// `xy()` against your own footprint — entirely in robot coordinates — avoids
  /// the issue, at the cost of not knowing where the cabin walls are.
  rmf_traffic::agv::Graph::LiftPropertiesPtr inside_lift() const;

  class Implementation;
private:
  Destination();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// One waypoint of a Path, bundled with the handle used to acknowledge it.
class PathGuide::PathWaypoint
{
public:
  /// Where the robot should go for this waypoint.
  const Destination& destination() const;

  /// How far off its lane RMF can still localize this robot here.
  ///
  /// **This is a tolerance, not an arrival radius.** PathGuide has no opinion
  /// about when this waypoint counts as reached — that is yours, exactly as it
  /// is under EasyFullControl, where you decide arrival outright by choosing
  /// when to call `finished()`. What this gives you is an **upper bound** on
  /// how loose your arrival test may safely be.
  ///
  /// Acknowledging a path's *final* waypoint ends the movement phase, so the
  /// robot stops wherever it happens to be. Credit it further out than this and
  /// RMF may be unable to merge the stopped robot back onto its lane — it is
  /// declared lost, replans, and a lift already summoned for it can be
  /// re-summoned to the wrong floor.
  ///
  /// It is `max(this robot's max_merge_lane_distance, the graph vertex's own
  /// merge_radius)`. A graph may be more permissive at an unusually open
  /// vertex, never stricter.
  ///
  /// \warning **In robot coordinates**, like `destination().position()`, so the
  /// two compare directly. Do not convert it again.
  ///
  /// \warning **Not lift-aware, deliberately.** Deciding when a robot is far
  /// enough inside a cabin for the doors to close safely needs your true
  /// footprint shape, arrival heading and stopping accuracy — none of which the
  /// adapter knows. Use `destination().inside_lift()` and tighten this bound
  /// yourself. Not hypothetical: a 2.7 m square cabin with a 0.9 m footprint
  /// radius leaves 0.45 m of clearance from the cabin centre, and a robot
  /// credited on 1.0 m had its centre only 0.35 m past the door plane with
  /// 0.778 m still outside when the doors closed on it.
  double merge_radius() const;

  /// This waypoint's position in the path, counting from 0.
  std::size_t index() const;

  /// The handle used to tell PathGuide that the robot has reached this
  /// waypoint. Waypoints must be acknowledged in order.
  CommandExecution execution() const;

  class Implementation;
private:
  PathWaypoint();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// An entire route for the robot to follow, handed over in one piece.
class PathGuide::Path
{
public:
  /// Every waypoint the robot should visit, in the order it should visit them.
  const std::vector<PathWaypoint>& waypoints() const;

  /// The traffic schedule plan that produced this path. This increases whenever
  /// RMF replans, so it can be used to recognise and discard acknowledgements
  /// that belong to a superseded path.
  std::size_t plan_id() const;

  /// The map that the robot is starting this path on.
  const std::string& map() const;

  class Implementation;
private:
  Path();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// The Configuration class contains parameters necessary to initialize a
/// PathGuide fleet instance and add fleets to the adapter.
class PathGuide::FleetConfiguration
{
public:

  /// Constructor
  ///
  /// \param[in] fleet_name
  ///   The name of the fleet that is being added.
  ///
  /// \param[in] transformations_to_robot_coordinates
  ///   A dictionary of transformations from RMF canonical coordinates to the
  ///   the coordinate system used by the robot. Each map should be assigned its
  ///   own transformation. If this is not nullptr, then a warning will be
  ///   logged whenever the dictionary is missing a transform for a map, and the
  ///   canonical RMF coordinates will be used.
  ///
  /// \param[in] known_robot_configurations
  ///   Configurations for the robots that are expected to join this fleet.
  ///
  /// \param[in] traits
  ///   Specify the approximate traits of the vehicles in this fleet.
  ///
  /// \param[in] graph
  ///   Specify the navigation graph used by the vehicles in this fleet.
  ///
  /// \param[in] battery_system
  ///   Specify the battery system used by the vehicles in this fleet.
  ///
  /// \param[in] motion_sink
  ///   Specify the motion sink that describes the vehicles in this fleet.
  ///
  /// \param[in] ambient_sink
  ///   Specify the device sink for ambient sensors used by the vehicles in this
  ///   fleet.
  ///
  /// \param[in] tool_sink
  ///   Specify the device sink for special tools used by the vehicles in this
  ///   fleet.
  ///
  /// \param[in] recharge_threshold
  ///   The threshold for state of charge below which robots in this fleet
  ///   will cease to operate and require recharging. A value between 0.0 and
  ///   1.0 should be specified.
  ///
  /// \param[in] recharge_soc
  ///   The state of charge to which robots in this fleet should be charged up
  ///   to by automatic recharging tasks. A value between 0.0 and 1.0 should be
  ///   specified.
  ///
  /// \param[in] account_for_battery_drain
  ///   Specify whether battery drain is to be considered while allocating
  ///   tasks. If false, battery drain will not be considered when planning for
  ///   tasks. As a consequence, charging tasks will not be automatically
  ///   assigned to vehicles in this fleet when battery levels fall below the
  ///   recharge_threshold.
  ///
  /// \param[in] retreat_to_charger_interval
  ///   Specify whether to allow automatic retreat to charger if the robot's
  ///   battery is estimated to fall below its recharge_threshold before it is
  ///   able to complete its current task. Provide a duration between checks in
  ///   seconds. If nullopt, retreat to charger would be disabled.
  ///
  /// \param[in] task_consideration
  ///   Provide callbacks for considering tasks belonging to each category.
  ///
  /// \param[in] action_consideration
  ///   List of actions that this fleet can perform. Each item represents a
  ///   category in the PerformAction description.
  ///
  /// \param[in] finishing_request
  ///   A factory for a request that should be performed by each robot in this
  ///   fleet at the end of its assignments.
  ///
  /// \param[in] server_uri
  ///   The URI for the websocket server that receives updates on tasks and
  ///   states. If nullopt, data will not be published.
  ///
  /// \param[in] max_delay
  ///   Specify the default value for how high the delay of the current
  ///   itinerary can become before it gets interrupted and replanned.
  ///
  /// \param[in] update_interval
  ///   The duration between positional state updates that are sent to
  ///   the fleet adapter.
  ///
  /// \param[in] default_responsive_wait
  ///   Should the robots in this fleet have responsive wait enabled (true) or
  ///   disabled (false) by default?
  ///
  /// \param[in] default_max_merge_waypoint_distance
  ///   The maximum merge distance between a robot position and a waypoint.
  ///
  /// \param[in] default_max_merge_lane_distance
  ///   The maximum merge distance between a robot position and a lane.
  ///
  /// \param[in] min_lane_length
  ///   The minimum length that a lane should have.
  ///
  ///
  ///   This is deliberately separate from max_merge_lane_distance, which the
  ///   two used to share. They pull in opposite directions: lane merging wants
  ///   a tight tolerance so that genuine drift is noticed, while arrival wants
  ///   one loose enough to cover however wide the robot actually rounds a
  ///   corner. Sharing one value meant loosening drift detection just to stop
  ///   paths stranding.
  ///
  ///   If left at its default of 0.0 it falls back to
  ///   default_max_merge_lane_distance, preserving the old coupled behaviour
  ///   for configurations that do not set it.
  ///
  /// \note Unlike EasyFullControl, there is no `skip_rotation_commands`
  /// parameter. PathGuide never emits a rotate-in-place waypoint, because such
  /// a waypoint is meaningless inside a path that the robot is expected to
  /// smooth over. Every destination carries the final orientation for its
  /// vertex and the robot owns its own in-place turns.
  FleetConfiguration(
    const std::string& fleet_name,
    std::optional<std::unordered_map<std::string, Transformation>>
    transformations_to_robot_coordinates,
    std::unordered_map<std::string, RobotConfiguration>
    known_robot_configurations,
    std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    rmf_battery::agv::ConstBatterySystemPtr battery_system,
    rmf_battery::ConstMotionPowerSinkPtr motion_sink,
    rmf_battery::ConstDevicePowerSinkPtr ambient_sink,
    rmf_battery::ConstDevicePowerSinkPtr tool_sink,
    double recharge_threshold,
    double recharge_soc,
    bool account_for_battery_drain,
    std::unordered_map<std::string, ConsiderRequest> task_consideration,
    std::unordered_map<std::string, ConsiderRequest> action_consideration,
    rmf_task::ConstRequestFactoryPtr finishing_request = nullptr,
    std::optional<std::string> server_uri = std::nullopt,
    rmf_traffic::Duration max_delay = rmf_traffic::time::from_seconds(10.0),
    rmf_traffic::Duration update_interval = rmf_traffic::time::from_seconds(
      0.5),
    bool default_responsive_wait = false,
    double default_max_merge_waypoint_distance = 1e-3,
    double default_max_merge_lane_distance = 0.3,
    double min_lane_length = 1e-8
  );

  /// Create a FleetConfiguration object using a set of configuration parameters
  /// imported from YAML files that follow the defined schema. This is an
  /// alternative to constructing the FleetConfiguration using the RMF objects
  /// if users do not require specific tool systems for their fleets. The
  /// FleetConfiguration object will be instantiated with instances of
  /// SimpleMotionPowerSink and SimpleDevicePowerSink.
  ///
  /// The schema is the same one EasyFullControl uses, so an existing
  /// `config.yaml` can be reused as-is. The `skip_rotation_commands` key is
  /// ignored, and a warning is logged if it is present.
  ///
  /// \param[in] config_file
  ///   The path to a configuration YAML file containing data about the fleet's
  ///   vehicle traits and task capabilities. This file needs to follow the
  ///   pre-defined config.yaml structure to successfully load the parameters
  ///   into the FleetConfiguration object.
  ///
  /// \param[in] nav_graph_path
  ///   The path to a navigation path file that includes map information
  ///   necessary to create a rmf_traffic::agv::Graph object
  ///
  /// \param[in] server_uri
  ///   The URI for the websocket server that receives updates on tasks and
  ///   states. If nullopt, data will not be published.
  ///
  /// \return A FleetConfiguration object with the essential config parameters
  /// loaded.
  static std::optional<FleetConfiguration> from_config_files(
    const std::string& config_file,
    const std::string& nav_graph_path,
    std::optional<std::string> server_uri = std::nullopt);

  /// Get the fleet name.
  const std::string& fleet_name() const;

  /// Set the fleet name.
  void set_fleet_name(std::string value);

  /// Get the transformations into robot coordinates for this fleet.
  const std::optional<std::unordered_map<std::string, Transformation>>&
  transformations_to_robot_coordinates() const;

  /// Set the transformation into robot coordinates for a map. This will replace
  /// any transformation previously set for the map. If the transformation
  /// dictionary was previously nullopt, this will initialize it with an empty
  /// value before inserting this transformation.
  void add_robot_coordinate_transformation(
    std::string map,
    Transformation transformation);

  /// Get a dictionary of known robot configurations. The key is the name of the
  /// robot belonging to this fleet. These configurations are usually parsed
  /// from a fleet configuration file.
  const std::unordered_map<std::string, RobotConfiguration>&
  known_robot_configurations() const;

  /// Get the names of all robots with known robot configurations.
  std::vector<std::string> known_robots() const;

  /// Provide a known configuration for a named robot.
  ///
  /// \param[in] robot_name
  ///   The unique name of the robot.
  ///
  /// \param[in] configuration
  ///   The configuration for the robot.
  void add_known_robot_configuration(
    std::string robot_name,
    RobotConfiguration configuration);

  /// Get a known configuration for a robot based on its name.
  std::optional<RobotConfiguration> get_known_robot_configuration(
    const std::string& robot_name) const;

  /// Get the fleet vehicle traits.
  const std::shared_ptr<const VehicleTraits>& vehicle_traits() const;

  /// Set the vehicle traits.
  void set_vehicle_traits(std::shared_ptr<const VehicleTraits> value);

  /// Get the fleet navigation graph.
  const std::shared_ptr<const Graph>& graph() const;

  /// Set the fleet navigation graph.
  void set_graph(std::shared_ptr<const Graph> value);

  /// Get the battery system.
  rmf_battery::agv::ConstBatterySystemPtr battery_system() const;

  /// Set the battery system.
  void set_battery_system(rmf_battery::agv::ConstBatterySystemPtr value);

  /// Get the motion sink.
  rmf_battery::ConstMotionPowerSinkPtr motion_sink() const;

  /// Set the motion sink.
  void set_motion_sink(rmf_battery::ConstMotionPowerSinkPtr value);

  /// Get the ambient sink.
  rmf_battery::ConstDevicePowerSinkPtr ambient_sink() const;

  /// Set the ambient sink.
  void set_ambient_sink(rmf_battery::ConstDevicePowerSinkPtr value);

  /// Get the tool sink.
  rmf_battery::ConstDevicePowerSinkPtr tool_sink() const;

  /// Set the tool sink.
  void set_tool_sink(rmf_battery::ConstDevicePowerSinkPtr value);

  /// Get the recharge threshold.
  double recharge_threshold() const;

  /// Set the recharge threshold.
  void set_recharge_threshold(double value);

  /// Get the recharge state of charge. If the robot's state of charge dips
  /// below this value then its next task will be to recharge.
  double recharge_soc() const;

  /// Set the recharge state of charge.
  void set_recharge_soc(double value);

  /// Get whether or not to account for battery drain during task planning.
  bool account_for_battery_drain() const;

  /// Set whether or not to account for battery drain during task planning.
  void set_account_for_battery_drain(bool value);

  /// Get the duration between retreat to charger checks.
  std::optional<rmf_traffic::Duration> retreat_to_charger_interval() const;

  /// Set the duration between retreat to charger checks. Passing in a nullopt
  /// will turn off these checks entirely.
  void set_retreat_to_charger_interval(
    std::optional<rmf_traffic::Duration> value);

  /// Get the task categories
  const std::unordered_map<std::string, ConsiderRequest>&
  task_consideration() const;

  /// Mutable access to the task consideration map.
  std::unordered_map<std::string, ConsiderRequest>& task_consideration();

  /// Get the action categories
  const std::unordered_map<std::string, ConsiderRequest>&
  action_consideration() const;

  /// Mutable access to the action consideration map.
  std::unordered_map<std::string, ConsiderRequest>& action_consideration();

  /// Get the finishing request.
  rmf_task::ConstRequestFactoryPtr finishing_request() const;

  /// Set the finishing request.
  void set_finishing_request(rmf_task::ConstRequestFactoryPtr value);

  /// Get the server uri.
  std::optional<std::string> server_uri() const;

  /// Set the server uri.
  void set_server_uri(std::optional<std::string> value);

  /// Get the max delay.
  rmf_traffic::Duration max_delay() const;

  /// Set the max delay.
  void set_max_delay(rmf_traffic::Duration value);

  /// Get the update interval.
  rmf_traffic::Duration update_interval() const;

  /// Set the update interval.
  void set_update_interval(rmf_traffic::Duration value);

  /// Should robots in this fleet have responsive wait enabled by default?
  bool default_responsive_wait() const;

  /// Should robots use the parking reservation system.
  bool using_parking_reservation_system() const;

  /// Set whether this fleet uses the parking reservation system.
  void use_parking_reservation_system(
    const bool use);

  /// Set whether robots in this fleet should have responsive wait enabled by
  /// default.
  void set_default_responsive_wait(bool enable);

  /// Get the maximum merge distance between a robot position and a waypoint.
  double default_max_merge_waypoint_distance() const;

  /// Set the maximum merge distance between a robot position and a waypoint.
  void set_default_max_merge_waypoint_distance(double distance);

  /// Get the maximum merge distance between a robot position and a lane.
  double default_max_merge_lane_distance() const;

  /// Set the maximum merge distance between a robot position and a lane.
  void set_default_max_merge_lane_distance(double distance);

  /// Get the minimum lane length allowed.
  double default_min_lane_length() const;

  /// Set the minimum lane length.
  void set_default_min_lane_length(double distance);

  /// Get the arrival radius that waypoints are published with.
  ///
  /// Returns the effective value: if no arrival radius was configured this is
  /// default_max_merge_lane_distance, which is what the two shared before they
  /// were separated.
 
  /// During a fire emergency, real-life lifts might be required to move to a
  /// specific level and refuse to stop or go to any other level. This function
  /// lets you provide this information to the fleet adapter so that it can
  /// produce reasonable emergency pullover plans for robots that happen to be
  /// inside of a lift when the fire alarm goes off.
  ///
  /// Internally, this will close all lanes that go into the specified lift and
  /// close all lanes exiting this lift (except on the designated level) when a
  /// fire emergency begins. Lifts that were not specified in a call to this
  /// function will not behave any differently during a fire emergency.
  ///
  /// \param[in] lift_name
  ///   The name of the lift whose behavior is being specified
  ///
  /// \param[in] emergency_level_name
  ///   The level that lift will go to when a fire emergency is happening
  void set_lift_emergency_level(
    std::string lift_name,
    std::string emergency_level_name);

  /// Get mutable access to the level that each specified lift will go to during
  /// a fire emergency.
  ///
  /// \sa set_lift_emergency_level
  std::unordered_map<std::string, std::string>& change_lift_emergency_levels();

  /// Get the level that each specified lift will go to during a fire emergency.
  ///
  /// \sa set_lift_emergency_level
  const std::unordered_map<std::string, std::string>&
  lift_emergency_levels() const;

  /// A set of lanes which must strictly be navigated from from the start to end
  /// of the lane when used. This means when replanning, the planner cannot ask
  /// a robot in the middle of one of these lanes to immediately go to the end
  /// of the lane.
  const std::unordered_set<std::size_t>& strict_lanes() const;

  /// Get a mutable reference to the set of strict lanes.
  ///
  /// \sa strict_lanes
  std::unordered_set<std::size_t>& change_strict_lanes();

  /// Get the task planner assignment strategy.
  const AssignmentStrategy& task_assignment_strategy() const;

  /// Set the task planner assignment strategy.
  void set_task_assignment_strategy(
    const AssignmentStrategy& strategy);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__PATHGUIDE_HPP
