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

#ifndef RMF_FLEET_ADAPTER__AGV__ROBOTCOMMANDHANDLE_HPP
#define RMF_FLEET_ADAPTER__AGV__ROBOTCOMMANDHANDLE_HPP

#include <rmf_traffic/agv/Planner.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// Implement this class to receive robot commands from RMF
class RobotCommandHandle
{
public:

  using Duration = rmf_traffic::Duration;

  /// Use this callback function to update the fleet adapter on how long the
  /// robot will take to reach its next destination.
  ///
  /// \param[in] path_index
  ///   The index of the path element that the robot is currently heading
  ///   towards.
  ///
  /// \param[in] remaining_time
  ///   An estimate of how much longer the robot will take to arrive at
  ///   `path_index`.
  using ArrivalEstimator =
    std::function<void(std::size_t path_index, Duration remaining_time)>;

  /// Trigger this callback function when the follow_new_path request has been
  /// completed. It should only be triggered that one time and then discarded.
  using RequestCompleted = std::function<void()>;

  /// Have the robot follow a new path. If it was already following a path, then
  /// it should immediately switch over to this one.
  ///
  /// \param[in] waypoints
  ///   The sequence of waypoints to follow. When the robot arrives at a
  ///   waypoint in this sequence, it should wait at that waypoint until the
  ///   clock reaches the time() field of the waypoint. This is important
  ///   because the waypoint timing is used to avoid traffic conflicts with
  ///   other vehicles.
  ///
  /// \param[in] next_arrival_estimator
  ///   Use this callback to give estimates for how long the robot will take to
  ///   reach the path element of the specified index. You should still be
  ///   calling RobotUpdateHandle::update_position() even as you call this
  ///   function.
  ///
  /// \param[in] path_finished_callback
  ///   Trigger this callback when the robot is done following the new path.
  ///   You do not need to trigger waypoint_arrival_callback when triggering
  ///   this one.
  virtual void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) = 0;

  /// Have the robot come to an immediate stop.
  virtual void stop() = 0;

  /// Have the robot begin a pre-defined docking procedure. Implement this
  /// function as a no-op if your robots do not perform docking procedures.
  ///
  /// \param[in] dock_name
  ///   The predefined name of the docking procedure to use.
  ///
  /// \param[in] docking_finished_callback
  ///   Trigger this callback when the docking is finished.
  virtual void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) = 0;

  // Virtual destructor
  virtual ~RobotCommandHandle() = default;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__ROBOTCOMMANDHANDLE_HPP
