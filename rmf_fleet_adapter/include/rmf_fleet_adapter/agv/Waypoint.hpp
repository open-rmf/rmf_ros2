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

#ifndef RMF_FLEET_ADAPTER__AGV__WAYPOINT_HPP
#define RMF_FLEET_ADAPTER__AGV__WAYPOINT_HPP

#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <Eigen/Geometry>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Waypoint
{
public:

  /// Constructor
  ///
  /// \param[in] map_name
  ///   The name of the reference map that the position is on.
  ///
  /// \param[in] position
  ///   A position along the robot's path where it will (or can) have zero
  ///   instantaneous velocity. This will usually be a point where the robot
  ///   needs to turn, or a point that comes before an intersection, so the
  ///   robot can come to a stop and allow other vehicles to pass. This is a 2D
  ///   heterogeneous vector. The first two elements refer to translational
  ///   (x,y) position while the third element is a yaw value in radians.
  ///
  /// \param[in] mandatory_delay
  ///   A delay that the robot is expected to experience once it arrives at
  ///   this waypoint. Usually this will be caused by the robots needing to
  ///   wait for doors to open or lifts to arrive.
  ///
  /// \param[in] yield
  ///   Whether or not the robot can wait at this point. If yield is false, then
  ///   the planner will assume that it cannot ask the robot to wait here. If
  ///   yield is true, then the planner may request that the robot wait here to
  ///   avoid conflicts with other traffic participants.
  Waypoint(
    std::string map_name,
    Eigen::Vector3d position,
    rmf_traffic::Duration mandatory_delay = std::chrono::nanoseconds(0),
    bool yield = true);

  /// Get the map of this Waypoint.
  const std::string& map_name() const;

  /// Set the map of this Waypoint.
  Waypoint& map_name(std::string new_map_name);

  /// Get the position of this Waypoint.
  const Eigen::Vector3d& position() const;

  /// Set the position of this Waypoint.
  Waypoint& position(Eigen::Vector3d new_position);

  /// Get the mandatory delay associated with this Waypoint.
  rmf_traffic::Duration mandatory_delay() const;

  /// Set the mandatory delay associated with this Waypoint.
  Waypoint& mandatory_delay(rmf_traffic::Duration duration);

  /// Get whether the robot can yield here.
  bool yield() const;

  /// Set whether the robot can yield here.
  Waypoint& yield(bool on);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__WAYPOINT_HPP
