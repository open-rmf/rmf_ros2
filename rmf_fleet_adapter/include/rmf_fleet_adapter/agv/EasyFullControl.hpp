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
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Graph.hpp>

namespace rmf_fleet_adapter {
namespace agv {

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
    /// \param[in] node_name
    ///   The name for the rclcpp::Node that will be produced for the Adapter.
    ///
    /// \param[in] fleet_name
    ///   The name of the fleet that is being added.
    ///
    /// \param[in] graph
    ///   The graph which is being used by the fleet.
    ///
    /// \param[in] traits
    ///   The traits of the vehicle that is added to the fleet.
    ///
    /// \param[in] server_uri
    ///   The URI for the websocket server that receives updates on tasks and
    ///   states. If nullopt, data will not be published.
    Configuration(
      const std::string& node_name,
      const std::string& fleet_name,
      rmf_traffic::agv::Graph graph,
      rmf_traffic::agv::VehicleTraits traits,
      std::optional<std::string> server_uri = std::nullopt);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Initialize and spin an Adapter instance in order to add fleets.
  ///
  /// \param[in] config
  ///   The Configuration for the adapter that contains parameters used by the
  ///   fleet robots.
  static std::shared_ptr<EasyFullControl> make(Configuration config);

  using Start = std::variant<rmf_traffic::agv::Planner::Start, Eigen::Vector3d>;
  using GetPosition = std::function<Eigen::Vector3d()>;
  using ProcessCompleted = std::function<bool()>;

  /// Initialize a robot in the fleet.
  ///
  /// \param[in] name
  ///   The name of the robot.
  ///
  /// \param[in] get_position
  ///   The position function that returns the robot's current location.
  ///
  /// \param[in] navigate
  ///   The API function for navigating your robot to a pose.
  ///   Returns a ProcessCompleted callback to check status of navigation task.
  ///
  /// \param[in] action_executor
  ///   The ActionExecutor callback to request the robot to perform an action.
  bool add_robot(
    const std::string& name,
    Start pose,
    GetPosition get_position,
    std::function<ProcessCompleted(const Eigen::Vector3d pose)> navigate,
    ActionExecutor action_executor);

  class Implementation;
private:
  EasyFullControl();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using EasyFullControlPtr = std::shared_ptr<EasyFullControl>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
