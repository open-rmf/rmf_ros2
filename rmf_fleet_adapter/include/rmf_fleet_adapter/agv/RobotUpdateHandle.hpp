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

#ifndef RMF_FLEET_ADAPTER__AGV__ROBOTUPDATEHANDLE_HPP
#define RMF_FLEET_ADAPTER__AGV__ROBOTUPDATEHANDLE_HPP

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

#include <Eigen/Geometry>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// You will be given an instance of this class every time you add a new robot
/// to your fleet. Use that instance to send updates to RoMi-H about your
/// robot's state.
///
/// To remove the available of this robot, simply allow the instance to expire.
class RobotUpdateHandle
{
public:

  /// Tell the RMF schedule that the robot has been delayed.
  void add_delay(rmf_traffic::Duration duration);

  /// Tell the RMF schedule that the robot was interrupted and needs a new plan.
  void interrupted();

  /// Update the current position of the robot by specifying the waypoint that
  /// the robot is on and its orientation.
  void update_position(
      std::size_t waypoint,
      double orientation);

  /// Update the current position of the robot by specifying the x, y, yaw
  /// position of the robot and one or more lanes that the robot is occupying.
  ///
  /// \warning At least one lane must be specified. If no lane information is
  /// available, then use the update_position(std::string, Eigen::Vector3d)
  /// signature of this function.
  void update_position(
      const Eigen::Vector3d& position,
      const std::vector<std::size_t>& lanes);

  /// Update the current position of the robot by specifying the x, y, yaw
  /// position of the robot and what map the robot is on.
  ///
  /// \warning This function should only be used if the robot has diverged from
  /// the navigation graph somehow.
  void update_position(
      const std::string& map_name,
      const Eigen::Vector3d& position);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__ROBOTUPDATEHANDLE_HPP
