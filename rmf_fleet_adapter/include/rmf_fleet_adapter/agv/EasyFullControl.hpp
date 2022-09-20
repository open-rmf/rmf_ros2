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

#include <Eigen/Geometry>
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <yaml-cpp/yaml.h>
#include <variant>

namespace rmf_fleet_adapter {
namespace agv {

using Graph = rmf_traffic::agv::Graph;
using VehicleTraits = rmf_traffic::agv::VehicleTraits;
using Planner = rmf_traffic::agv::Planner;

//==============================================================================
class EasyFullControl : public std::enable_shared_from_this<EasyFullControl>
{
public:

  /// The Configuration class contains parameters necessary to initialize an
  /// Adapter instance and add fleets to the adapter.
  class Configuration
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
      const std::string& config_file,
      const std::string& nav_graph_path,
      std::optional<std::string> server_uri = std::nullopt);

    // Get the fleet config yaml node.
    const YAML::Node fleet_config() const;

    // Get the fleet navigation graph.
    Graph graph() const;

    // Get the fleet vehicle traits.
    VehicleTraits vehicle_traits() const;

    // Get a const reference to the server uri.
    std::optional<std::string> server_uri() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  struct Target
  {
    // The command id for the current navigation or dock task.
    std::size_t cmd_id;

    // The (x, y, yaw) coordinates that the robot should navigate to.
    Eigen::Vector3d pose;

    // The map that the robot should navigate to.
    std::string map_name;

    // Speed limit that the robot should adhere to.
    std::optional<double> speed_limit;
  };

  struct Position
  {
    // Current position of the robot.
    Eigen::Vector3d position;

    // The map that the robot is currently in.
    std::string map_name;

    // Remaining battery level left in the robot.
    double battery_percent;
  };

  /// Initialize and spin an Adapter instance in order to add fleets.
  ///
  /// \param[in] adapter
  ///   The Adapter instance for your fleet to be adapted.
  ///
  /// \param[in] config
  ///   The Configuration for the adapter that contains parameters used by the
  ///   fleet robots.
  static std::shared_ptr<EasyFullControl> make(
    Configuration config, const AdapterPtr& adapter);

  using Start = std::variant<Planner::Start, Eigen::Vector3d>;
  using GetPosition = std::function<Position()>;
  using ProcessCompleted = std::function<bool(std::size_t cmd_id)>;

  /// Initialize a robot in the fleet.
  ///
  /// \param[in] robot_name
  ///   The name of the robot.
  ///
  /// \param[in] pose
  ///   The starting pose of the robot.
  ///   Accepts either a known Planner::Start or an Eigen::Vector3d pose.
  ///
  /// \param[in] get_position
  ///   The position function that returns the robot's current location and battery status.
  ///
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
  bool add_robot(
    const std::string& robot_name,
    Start pose,
    GetPosition get_position,
    std::function<ProcessCompleted(const Target target)> navigate,
    std::function<ProcessCompleted(
      const std::string& dock_name, std::size_t cmd_id)> dock,
    ProcessCompleted stop,
    RobotUpdateHandle::ActionExecutor action_executor);

  class EasyCommandHandle;

  class Implementation;
private:
  EasyFullControl();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;

};

using EasyFullControlPtr = std::shared_ptr<EasyFullControl>;
using EasyCommandHandlePtr =
  std::shared_ptr<EasyFullControl::EasyCommandHandle>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP