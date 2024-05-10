/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
#define RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP

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

#include <rmf_utils/impl_ptr.hpp>

// System headers
#include <Eigen/Geometry>

namespace rmf_fleet_adapter {
namespace agv {

/// An easy to initialize full_control fleet adapter.
/// By default the adapter will be configured to accept all tasks.
/// To disable specific tasks, call the respective consider_*_requests() method
/// on the FleetUpdateHandle that can be accessed within this adapter.
//==============================================================================
class EasyFullControl : public std::enable_shared_from_this<EasyFullControl>
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

  // Nested class declarations
  class EasyRobotUpdateHandle;
  class RobotState;
  class RobotConfiguration;
  class RobotCallbacks;
  class Destination;
  class FleetConfiguration;
  class CommandExecution;

  /// Signature for a function that handles navigation requests. The request
  /// will specify a destination for the robot to go to.
  ///
  /// \param[in] destination
  ///   Where the robot should move to. Includings (x, y) coordinates, a target
  ///   yaw, a map name, and may include a graph index when one is available.
  ///
  /// \param[in] execution
  ///   The command execution progress updater. Use this to keep the fleet
  ///   adapter updated on the progress of the command.
  using NavigationRequest =
    std::function<void(
        Destination destination,
        CommandExecution execution)>;

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
  /// \return an easy robot update handle on success. A nullptr if an error
  /// occurred.
  std::shared_ptr<EasyRobotUpdateHandle> add_robot(
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
  EasyFullControl();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using EasyFullControlPtr = std::shared_ptr<EasyFullControl>;

/// Handle used to update information about one robot
class EasyFullControl::EasyRobotUpdateHandle
{
public:
  /// Recommended function for updating information about a robot in an
  /// EasyFullControl fleet.
  ///
  /// \param[in] state
  ///   The current state of the robot
  ///
  /// \param[in] current_action
  ///   The action that the robot is currently executing
  void update(
    RobotState state,
    ConstActivityIdentifierPtr current_activity);

  /// Get the maximum allowed merge waypoint distance for this robot.
  double max_merge_waypoint_distance() const;

  /// Modify the maximum allowed merge distance between the robot and a waypoint.
  ///
  /// \param[in] max_merge_waypoint_distance
  ///   The maximum merge waypoint distance for this robot.
  void set_max_merge_waypoint_distance(double distance);

  /// Get the maximum allowed merge lane distance for this robot.
  double max_merge_lane_distance() const;

  /// Modify the maximum allowed merge distance between the robot and a lane.
  ///
  /// \param[in] max_merge_lane_distance
  ///   The maximum merge lane distance for this robot.
  void set_max_merge_lane_distance(double distance);

  /// Get the minimum lane length for this robot.
  double min_lane_length() const;

  /// Modify the minimum lane length for this robot.
  ///
  /// \param[in] min_lane_length
  ///   The minimum length of a lane.
  void set_min_lane_length(double length);

  /// Get more options for updating the robot's state
  std::shared_ptr<RobotUpdateHandle> more();

  /// Immutable reference to the base robot update API
  std::shared_ptr<const RobotUpdateHandle> more() const;

  class Implementation;
private:
  EasyRobotUpdateHandle();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

/// The current state of a robot, passed into EasyRobotUpdateHandle::update
class EasyFullControl::RobotState
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
class EasyFullControl::RobotConfiguration
{
public:

  /// Constructor
  ///
  /// \param[in] compatible_chargers
  ///   List of chargers that this robot is compatible with
  ///
  /// \param[in] responsive_wait
  ///   Should this robot use the responsive wait behavior? true / false / fleet default.
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

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

class EasyFullControl::RobotCallbacks
{
public:

  /// Constructor
  ///
  /// \param[in] navigate
  ///   A function that can be used to request the robot to navigate to a location.
  ///   The function returns a handle which can be used to track the progress of the navigation.
  ///
  /// \param[in] stop
  ///   A function to stop the robot.
  ///
  /// \param[in] action_executor
  ///   The ActionExecutor callback to request the robot to perform an action.
  RobotCallbacks(
    NavigationRequest navigate,
    StopRequest stop,
    ActionExecutor action_executor);

  /// Get the callback for navigation
  NavigationRequest navigate() const;

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

/// Used by system integrators to give feedback on the progress of executing a
/// navigation or docking command.
class EasyFullControl::CommandExecution
{
public:

  /// Trigger this when the command is successfully finished. No other function
  /// in this CommandExecution instance will be usable after this.
  void finished();

  /// Returns false if the command has been stopped.
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
  /// EasyRobotUpdateHandle::update_position while executing this command.
  ConstActivityIdentifierPtr identifier() const;

  class Implementation;
private:
  CommandExecution();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

class EasyFullControl::Destination
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
  rmf_traffic::agv::Graph::LiftPropertiesPtr inside_lift() const;

  class Implementation;
private:
  Destination();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// The Configuration class contains parameters necessary to initialize an
/// EasyFullControl fleet instance and add fleets to the adapter.
class EasyFullControl::FleetConfiguration
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
  /// \param[in] traits
  ///   Specify the approximate traits of the vehicles in this fleet.
  ///
  /// \param[in] navigation_graph
  ///   Specify the navigation graph used by the vehicles in this fleet.
  ///
  /// \param[in] battery_system
  ///   Specify the battery system used by the vehicles in this fleet.
  ///
  /// \param[in] motion_sink
  ///   Specify the motion sink that describes the vehicles in this fleet.
  ///
  /// \param[in] ambient_sink
  ///   Specify the device sink for ambient sensors used by the vehicles in this fleet.
  ///
  /// \param[in] tool_sink
  ///   Specify the device sink for special tools used by the vehicles in this fleet.
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
  ///   Specify whether battery drain is to be considered while allocating tasks.
  ///   If false, battery drain will not be considered when planning for tasks.
  ///   As a consequence, charging tasks will not be automatically assigned to
  ///   vehicles in this fleet when battery levels fall below the
  ///   recharge_threshold.
  ///
  /// \param[in] retreat_to_charger_interval
  ///   Specify whether to allow automatic retreat to charger if the robot's
  ///   battery is estimated to fall below its recharge_threshold before it is
  ///   able to complete its current task. Provide a duration between checks in
  ///   seconds. If nullopt, retreat to charger would be disabled.
  ///
  /// \param[in] task_categories
  ///   Provide callbacks for considering tasks belonging to each category.
  ///
  /// \param[in] action_categories
  ///   List of actions that this fleet can perform. Each item represents a
  ///   category in the PerformAction description.
  ///
  /// \param[in] finishing_request
  ///   A factory for a request that should be performed by each robot in this
  ///   fleet at the end of its assignments.
  ///
  /// \param[in] skip_rotation_commands
  ///   If true, navigation requests which would only have the robot rotate in
  ///   place will not be sent. Instead, navigation requests will always have
  ///   the final orientation for the destination.
  ///
  /// \param[in] server_uri
  ///   The URI for the websocket server that receives updates on tasks and
  ///   states. If nullopt, data will not be published.
  ///
  /// \param[in] max_delay
  ///   Specify the default value for how high the delay of the current itinerary
  ///   can become before it gets interrupted and replanned.
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
  /// \param[in] default_min_lane_length
  ///   The minimum length that a lane should have.
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
    bool skip_rotation_commands = true,
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
  /// alternative to constructing the FleetConfiguration using the RMF objects if
  /// users do not require specific tool systems for their fleets. The
  /// FleetConfiguration object will be instantiated with instances of
  /// SimpleMotionPowerSink and SimpleDevicePowerSink.
  ///
  /// \param[in] config_file
  ///   The path to a configuration YAML file containing data about the fleet's
  ///   vehicle traits and task capabilities. This file needs to follow the pre-defined
  ///   config.yaml structure to successfully load the parameters into the FleetConfiguration
  ///   object.
  ///
  /// \param[in] nav_graph_path
  ///   The path to a navigation path file that includes map information necessary
  ///   to create a rmf_traffic::agv::Graph object
  ///
  /// \param[in] server_uri
  ///   The URI for the websocket server that receives updates on tasks and
  ///   states. If nullopt, data will not be published.
  ///
  /// \return A FleetConfiguration object with the essential config parameters loaded.
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

  /// Check whether rotation commands will be skipped.
  bool skip_rotation_commands() const;

  /// Set whether rotation commands will be skipped.
  void set_skip_rotation_commands(bool value);

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

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
