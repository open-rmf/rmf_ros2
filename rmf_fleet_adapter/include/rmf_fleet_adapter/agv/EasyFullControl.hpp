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

#ifndef RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
#define RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>

// rmf_fleet_adapter headers
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

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

  /// The Configuration class contains parameters necessary to initialize an
  /// EasyFullControl instance and add fleets to the adapter.
  class Configuration
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
    /// \param[in] action_categories
    ///   List of actions that this fleet can perform. Each item represents a
    ///   category in the PerfromAction description.
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
      rmf_task::ConstRequestFactoryPtr finishing_request = nullptr,
      std::optional<std::string> server_uri = std::nullopt,
      rmf_traffic::Duration max_delay = rmf_traffic::time::from_seconds(10.0),
      rmf_traffic::Duration update_interval = rmf_traffic::time::from_seconds(0.5)
    );

    /// Get the fleet name.
    const std::string& fleet_name() const;

    /// Get the fleet vehicle traits.
    const VehicleTraits& vehicle_traits() const;

    /// Get the fleet navigation graph.
    const Graph& graph() const;

    /// Get the battery system.
    std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system() const;

    /// Get the motion sink.
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink() const;

    /// Get the ambient sink.
    std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink() const;

    /// Get the tool sink.
    std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink() const;

    /// Get the recharge threshold.
    double recharge_threshold() const;

    /// Get the recharge soc.
    double recharge_soc() const;

    /// Get whether or not to account for battery drain during task planning.
    bool account_for_battery_drain() const;

    /// Get the action categories
    const std::vector<std::string>& action_categories() const;

    /// Get the finishing request.
    rmf_task::ConstRequestFactoryPtr finishing_request() const;

    /// Get the server uri.
    std::optional<std::string> server_uri() const;

    /// Get the max delay.
    rmf_traffic::Duration max_delay() const;

    /// Get the update interval.
    rmf_traffic::Duration update_interval() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The RobotState class encapsulates information about a robot in this fleet.
  class RobotState
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
    RobotState(
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

  /// Signature for a callback that returns the current RobotState of the robot.
  ///
  /// \return RobotState
  using GetStateCallback = std::function<RobotState(void)>;

  /// Signature for a function that is used to track the status of a goal.
  ///
  /// \param[in] remaining_time
  ///   A mutable reference to set the time remaining for this goal. If not
  ///   updated, the adapter will derive the time estimate by interpolating
  ///   the robot's motion to the destination location.
  ///
  /// \param[in]  request_replan
  ///   A mutable reference to request the fleet adapter to generate a new plan
  ///   for this robot due to issues with completing the goal.
  ///
  /// \return True if the robot has completed the goal.
  using GoalCompletedCallback = std::function<bool(
        rmf_traffic::Duration& remaining_time,
        bool& request_replan)>;

  /// Signature for a function to request the robot to navigate to a location.
  ///
  /// \param[in] map_name
  ///   The name of the map where the location exists.
  ///
  /// \param[in] location
  ///   The XY position and orientation of the location.
  ///
  /// \param[in] robot_handle
  ///   The robot_handle may be used to submit issue tickets if the robot
  ///   is having any challenges with reaching the location.
  ///
  /// \return A GoalCompletedCallback to query the status of this request.
  using NavigationRequest =
    std::function<GoalCompletedCallback(
        const std::string& map_name,
        const Eigen::Vector3d location,
        RobotUpdateHandlePtr robot_handle)>;

  /// Signature for a function to request the robot to stop.
  ///
  /// \return true if the robot has come to a stop.
  using StopRequest = std::function<bool(void)>;

  /// Signature for a function to request the robot to dock at a location.
  ///
  /// \param[in] dock_name
  ///   The name of the dock.
  ///
  /// \param[in] robot_handle
  ///   The robot_handle may be used to submit issue tickets if the robot
  ///   is having any challenges with reaching the location.
  ///
  /// \return A GoalCompletedCallback to query the status of this request.
  using DockRequest =
    std::function<GoalCompletedCallback(
        const std::string& dock_name,
        RobotUpdateHandlePtr robot_handle)>;

  /// Make an EasyFullControl adapter instance.
  /// \param[in] config
  ///   The Configuration for the adapter that contains parameters used by the
  ///   fleet robots.
  ///   If nullopt, the adapter will attempt to read all required parameters
  ///   via its ROS 2 parameter server.
  ///
  /// \param[in] node_options
  ///   The options that the rclcpp::Node will be constructed with.
  ///
  /// \param[in] discovery_timeout
  ///   How long we will wait to discover the Schedule Node before giving up. If
  ///   rmf_utils::nullopt is given, then this will try to use the
  ///   discovery_timeout node parameter, or it will wait 1 minute if the
  ///   discovery_timeout node parameter was not defined.
  static std::shared_ptr<EasyFullControl> make(
    std::optional<Configuration> config = std::nullopt,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
    std::optional<rmf_traffic::Duration> discovery_timeout = std::nullopt);

  /// Get the rclcpp::Node that this adapter will be using for communication.
  std::shared_ptr<rclcpp::Node> node();

  /// Get the FleetUpdateHandle that this adapter will be using.
  /// This may be used to perform more specialized customizations using the
  /// standard FleetUpdateHandle API.
  std::shared_ptr<FleetUpdateHandle> fleet_handle();

  /// Wait till the adapter is finished spinning.
  EasyFullControl& wait();

  /// Add a robot to the fleet once it is available.
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
  /// \return false if the robot was not added to the fleet.
  /// \note This will internally call Planner::compute_plan_starts() using the
  ///    provieded start_state determine the StartSet of the robot. If you
  ///    have a custom method to determine the StartSet of the robot, consider
  ///    calling fleet_handle()->add_robot() instead.
  bool add_robot(
    RobotState start_state,
    GetStateCallback get_state,
    NavigationRequest navigate,
    StopRequest stop,
    DockRequest dock,
    ActionExecutor action_executor);

  // TODO(YV): Add an overloaded API for add_robot() where users can pass in a
  // RobotCommandHandle instead of all the callbacks.

  class Implementation;
private:
  EasyFullControl();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using EasyFullControlPtr = std::shared_ptr<EasyFullControl>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
