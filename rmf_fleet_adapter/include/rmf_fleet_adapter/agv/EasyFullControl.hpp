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
  using ConstActivityIdentifierPtr = RobotUpdateHandle::ConstActivityIdentifierPtr;
  using Stubbornness = RobotUpdateHandle::Unstable::Stubbornness;
  using ConsiderRequest = FleetUpdateHandle::ConsiderRequest;

  // Nested class declarations
  class EasyRobotUpdateHandle;
  class Destination;
  class Configuration;
  class InitializeRobot;
  class CommandExecution;

  /// Signature for a function that handles navigation requests. The request
  /// will specify an (x, y) location and yaw on a map.
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

  /// Signature for a function to request the robot to dock at a location.
  ///
  /// \param[in] dock_name
  ///   The name of the dock.
  ///
  /// \param[in] execution
  ///   The action execution may be used to update docking request items
  ///   as well as to obtain the robot handle for the robot to submit any
  ///   issue tickets regarding challenges with reaching the location.
  using DockRequest =
    std::function<void(
        const std::string& dock_name,
        CommandExecution execution)>;

  /// Add a robot to the fleet once it is available.
  ///
  /// \note This will internally call Planner::compute_plan_starts() using the
  ///    provieded start_state determine the StartSet of the robot. If you
  ///    have a custom method to determine the StartSet of the robot, consider
  ///    calling fleet_handle()->add_robot() instead.
  ///
  /// \param[in] start_state
  ///   The initial state of the robot when it is added to the fleet.
  ///
  /// \param[in] get_state
  ///   A function that returns the most recent state of the robot when called.
  ///
  /// \param[in] navigate
  ///   A function that can be used to request the robot to navigate to a location.
  ///   The function returns a handle which can be used to track the progress of the navigation.
  ///
  /// \param[in] stop
  ///   A function to stop the robot.
  ///
  /// \param[in] dock
  ///   A function that can be used to request the robot to dock at a location.
  ///   The function returns a handle which can be used to track the progress of the docking.
  ///
  /// \param[in] action_executor
  ///   The ActionExecutor callback to request the robot to perform an action.
  ///
  /// \return an easy robot update handle on success. A nullptr if an error
  /// occurred.
  std::shared_ptr<EasyRobotUpdateHandle> add_robot(
    InitializeRobot initial_state,
    NavigationRequest navigate,
    StopRequest stop,
    DockRequest dock,
    ActionExecutor action_executor);

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

class EasyFullControl::EasyRobotUpdateHandle
{
public:
  /// Recommended function for updating the position of a robot in an
  /// EasyFullControl fleet.
  ///
  /// \param[in] map_name
  ///   The name of the map the robot is currently on
  ///
  /// \param[in] position
  ///   The current position of the robot
  ///
  /// \param[in] current_action
  ///   The action that the robot is currently executing
  void update_position(
    const std::string& map_name,
    const Eigen::Vector3d& position,
    ConstActivityIdentifierPtr current_activity);

  /// Update the current battery level of the robot by specifying its state of
  /// charge as a fraction of its total charge capacity, i.e. a value from 0.0
  /// to 1.0.
  void update_battery_soc(const double battery_soc);

  /// Get more options for updating the robot's state
  RobotUpdateHandle& more();

  class Implementation;
private:
  EasyRobotUpdateHandle();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

///
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
  /// \param[in] trajectory
  ///   The path of the agent
  ///
  /// \return a Stubbornness handle that tells the fleet adapter to not let the
  /// overridden path be negotiated. The returned handle will stop having an
  /// effect after this command execution is finished.
  Stubbornness override_schedule(
    std::string map,
    std::vector<Eigen::Vector3d> path);

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

  class Implementation;
private:
  Destination();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// The Configuration class contains parameters necessary to initialize an
/// EasyFullControl fleet instance and add fleets to the adapter.
class EasyFullControl::Configuration
{
public:

  /// Constructor
  ///
  /// \param[in] fleet_name
  ///   The name of the fleet that is being added.
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
  Configuration(
    const std::string& fleet_name,
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
      0.5)
  );

  /// Create a Configuration object using a set of configuration parameters
  /// imported from YAML files that follow the defined schema. This is an
  /// alternative to constructing the Configuration using the RMF objects if
  /// users do not require specific tool systems for their fleets. The
  /// Configuration object will be instantiated with instances of
  /// SimpleMotionPowerSink and SimpleDevicePowerSink.
  ///
  /// \param[in] config_file
  ///   The path to a configuration YAML file containing data about the fleet's
  ///   vehicle traits and task capabilities. This file needs to follow the pre-defined
  ///   config.yaml structure to successfully load the parameters into the Configuration
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
  /// \return A Configuration object with the essential config parameters loaded.
  static std::shared_ptr<Configuration> from_config_files(
    const std::string& config_file,
    const std::string& nav_graph_path,
    std::optional<std::string> server_uri = std::nullopt);

  /// Get the fleet name.
  const std::string& fleet_name() const;

  /// Set the fleet name.
  void set_fleet_name(std::string value);

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

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// The InitializeRobot class encapsulates information about a robot in this fleet.
class EasyFullControl::InitializeRobot
{
public:
  /// Constructor
  ///
  /// \param[in] name
  ///   The name of this robot.
  ///
  /// \param[in] charger_name
  ///   The name of the charger waypoint of this robot.
  ///
  /// \param[in] map_name
  ///   The name of the map this robot is currenly on.
  ///
  /// \param[in] location
  ///   The XY position and orientation of this robot on current map.
  ///
  /// \param[in] battery_soc
  ///   The state of charge of the battery of this robot.
  ///   This should be a value between 0.0 and 1.0.
  ///
  /// \param[in] action
  ///   The state of action of this robot. True if robot is
  ///   performing an action.
  InitializeRobot(
    const std::string& name,
    const std::string& charger_name,
    const std::string& map_name,
    Eigen::Vector3d location,
    double battery_soc);

  /// Get the name.
  const std::string& name() const;

  /// Get the charger name.
  const std::string& charger_name() const;

  /// Get the map name.
  const std::string& map_name() const;

  /// Get the location.
  const Eigen::Vector3d& location() const;

  /// Get the battery_soc.
  double battery_soc() const;

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
