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

  /// Forward declarations
  class Configuration;
  class RobotState;

  /// Callback definitions
  using GetStateCallback = std::function<RobotState(void)>;
  /// Set replan to true if your robot is stuck and needs a new plan.
  using GoalCompletedCallback = std::function<bool(
        rmf_traffic::Duration& remaining_time,
        bool& request_replan)>;
  using NavigationRequest =
    std::function<GoalCompletedCallback(
        const std::string& map_name,
        const Eigen::Vector3d goal,
        RobotUpdateHandlePtr robot_handle)>;

  using StopRequest = std::function<bool(void)>;

  using DockRequest =
    std::function<GoalCompletedCallback(
        const std::string& dock_name,
        RobotUpdateHandlePtr robot_handle)>;

  /// Make an EasyFullControl adapter instance.
  /// \param[in] config
  ///   The Configuration for the adapter that contains parameters used by the
  ///   fleet robots.
  ///
  /// \param[in] node_options
  ///   The options that the rclcpp::Node will be constructed with.
  ///
  /// \param[in] discovery_timeout
  ///   How long we will wait to discover the Schedule Node before giving up. If
  ///   rmf_utils::nullopt is given, then this will try to use the
  ///   discovery_timeout node paramter, or it will wait 1 minute if the
  ///   discovery_timeout node parameter was not defined.
  static std::shared_ptr<EasyFullControl> make(
    Configuration config,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
    std::optional<rmf_traffic::Duration> discovery_timeout = std::nullopt);

  /// Get the rclcpp::Node that this adapter will be using for communication.
  std::shared_ptr<rclcpp::Node> node();

  /// Get the FleetUpdateHandle that this adapter will be using.
  /// This may be used to perform more specialized customizations using the
  /// standard FleetUpdateHandle API.
  std::shared_ptr<FleetUpdateHandle> fleet_handle();

  /// Begin running the event loop for this adapter. The event loop will operate
  /// in another thread, so this function is non-blocking.
  EasyFullControl& start();

  /// Stop the event loop if it is running.
  EasyFullControl& stop();

  /// Wait until the adapter is done spinning.
  ///
  /// \sa wait_for()
  EasyFullControl& wait();

  /// Add a robot to the fleet once it is available.
  /// \param[in] navigate
  ///   The API function for navigating your robot to a pose.
  ///   Returns a ProcessCompleted callback to check status of navigation task.
  ///
  /// \param[in] dock
  ///   The API function for starting a dock process.
  ///   Returns a ProcessCompleted callback to check status of docking task.
  ///
  /// \param[in] stop
  ///   The API for command your robot to stop.
  ///   Returns a bool indicating whether stop was successful.
  ///
  /// \param[in] action_executor
  ///   The ActionExecutor callback to request the robot to perform an action.
  ///
  /// \return false if the robot was not added to the fleet.
  /// \note This will internally call Planner::compute_plan_starts() using the
  ///    provieded start_state determine the StartSet of the robot. If you
  ///    have a custom method to determine the StartSet of the robot, consider
  ///    calling flee_handle()->add_robot() instead.
  bool add_robot(
    RobotState start_state,
    GetStateCallback get_state,
    NavigationRequest handle_nav_request,
    StopRequest handle_stop,
    DockRequest handle_dock,
    ActionExecutor action_executor);

  // TODO(YV): Add an overloaded API for add_robot() where users can pass in a
  // RobotCommandHandle instead of all the callbacks.

  class Implementation;
private:
  EasyFullControl();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// The Configuration class contains parameters necessary to initialize an
/// Adapter instance and add fleets to the adapter.
class EasyFullControl::Configuration
{
public:

  /// Constructor
  ///
  /// \param[in] config_file
  ///   The config file that provides important parameters for setting up the fleet adapter.
  ///
  /// \param[in] nav_graph_path
  ///   The graph file that this fleet should use for navigation.
  ///
  /// \param[in] server_uri
  ///   The URI for the websocket server that receives updates on tasks and
  ///   states. If nullopt, data will not be published.
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

  const std::string& fleet_name() const;

  // Get the fleet vehicle traits.
  const VehicleTraits& vehicle_traits() const;

  // Get the fleet navigation graph.
  const Graph& graph() const;

  // Get a const reference to the server uri.
  std::optional<std::string> server_uri() const;

  std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system() const;
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink() const;
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink() const;
  std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink() const;
  double recharge_threshold() const;
  double recharge_soc() const;
  bool account_for_battery_drain() const;
  const std::vector<std::string>& action_categories() const;
  rmf_task::ConstRequestFactoryPtr finishing_request() const;
  rmf_traffic::Duration max_delay() const;
  rmf_traffic::Duration update_interval() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class EasyFullControl::RobotState
{
public:

  /// Constructor
  RobotState(
    const std::string& name,
    const std::string& charger_name,
    const std::string& map_name,
    Eigen::Vector3d location,
    double battery_soc);

  const std::string& name() const;
  const std::string& charger_name() const;
  const std::string& map_name() const;
  const Eigen::Vector3d& location() const;
  double battery_soc() const;

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};


using EasyFullControlPtr = std::shared_ptr<EasyFullControl>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
