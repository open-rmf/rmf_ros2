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

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class EasyFullControl : public std::enable_shared_from_this<EasyFullControl>
{
public:

  using ProcessCompleted = std::function<bool(const std::string& id)>;

  /// Initialize a robot in the fleet
  ///
  /// \param[in] name
  ///   The name of the robot
  ///
  /// \param[in] get_position
  ///   The position function that returns the robot's current location
  ///
  /// \param[in] navigate
  ///   The API function for navigating your robot to a pose
  ///   Returns a ProcessCompleted callback to check status of navigation task
  ///
  /// \param[in] action_executor
  ///   The ActionExecutor callback to request the robot to perform an action

  bool add_robot(
    const std::string& name,
    const Eigen::Vector3d& pose,
    std::function<Eigen::Vector3d()> get_position,
    std::function<ProcessCompleted(const Eigen::Vector3d pose)> navigate,
    ActionExecutor action_executor);

private:
  EasyFullControl();
};

using EasyFullControlPtr = std::shared_ptr<EasyFullControl>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
