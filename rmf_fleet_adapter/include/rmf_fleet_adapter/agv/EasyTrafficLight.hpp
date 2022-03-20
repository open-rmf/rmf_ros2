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

#ifndef RMF_FLEET_ADAPTER__AGV__EASYTRAFFICLIGHT_HPP
#define RMF_FLEET_ADAPTER__AGV__EASYTRAFFICLIGHT_HPP

#include <rmf_fleet_adapter/agv/Waypoint.hpp>
#include <rmf_traffic/schedule/ParticipantDescription.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class EasyTrafficLight : public std::enable_shared_from_this<EasyTrafficLight>
{
public:

  /// Update the traffic light with a new path for your robot.
  ///
  /// \warning This function will throw an exception if there are less than 2
  /// waypoints in the path.
  void follow_new_path(const std::vector<Waypoint>& new_path);

  /// This instruction is given for moving updates. It
  enum class MovingInstruction : uint8_t
  {
    /// This indicates that your robot has not obeyed its instructions to stop.
    /// When this happens, it could mean serious problems for the overall
    /// traffic light system, including permanent deadlocks with other robots.
    /// This error may be seen if moving_from(~) is called during the time gap
    /// between the robot being instructed to pause and the feedback from the
    /// robot that it has paused.
    MovingError = 0,

    /// When the robot reaches the next checkpoint, it should continue.
    ContinueAtNextCheckpoint,

    /// When the robot reaches the next checkpoint, it must wait.
    WaitAtNextCheckpoint,

    /// The robot should pause immediately. This typically means there has been
    /// a change of plans and now the robot is scheduled to give way to another.
    PauseImmediately
  };

  /// Tell the traffic light that the robot is moving.
  ///
  /// \param[in] checkpoint
  ///   The last checkpoint which the robot passed over.
  ///
  /// \param[in] location
  ///   The current location of the robot.
  ///
  /// \return what the robot should do when it reaches its next checkpoint. This
  /// may change in between calls to this function. The results may even change
  /// from ContinueAtNextCheckpoint to WaitAtNextCheckpoint if a negotiation
  /// decided to have this robot give way to another robot. You must always use
  /// the latest value received from this function.
  ///
  /// \warning This function should only be called if the system has enough time
  /// for the robot to stop at the next checkpoint (i.e. accounting for the
  /// network latency of sending out the stop command and the maximum
  /// deceleration of the robot). The expectation is that if this function
  /// returns a WaitAtNextCheckpoint instruction, then the robot will definitely
  /// wait at the next checkpoint (until instructed otherwise). If that
  /// expectation is violated, you may get MovingError and/or WaitingError
  /// results, and the overall traffic flow may get interrupted or deadlocked.
  ///
  /// \note If your robot might not be able to stop in time to wait at the next
  /// checkpoint, then call moving_from(checkpoint+1, location) instead, even if
  /// your robot has not physically reached checkpoint+1 yet.
  [[nodiscard]]
  MovingInstruction moving_from(
    std::size_t checkpoint,
    Eigen::Vector3d location);

  enum class WaitingInstruction : uint8_t
  {
    /// This indicates that your robot has not obeyed its instructions to stop.
    /// When this happens, it could mean serious problems for the overall
    /// traffic light system, including permanent deadlocks with other robots.
    WaitingError = 0,

    /// The robot can continue along its path. It no longer needs to wait here.
    Resume,

    /// The robot must continue waiting here.
    Wait,
  };

  /// Tell the traffic light that the robot is waiting at a checkpoint.
  ///
  /// \param[in] checkpoint
  ///   The checkpoint where the robot is waiting.
  ///
  /// \return whether the robot should resume its travel or keep waiting.
  [[nodiscard]]
  WaitingInstruction waiting_at(std::size_t checkpoint);

  /// Tell the traffic light that the robot is waiting at a location in-between
  /// waypoints.
  ///
  /// \param[in] checkpoint
  ///   The last checkpoint that the robot passed.
  ///
  /// \param[in] location
  ///   The location where the robot is waiting.
  [[nodiscard]]
  WaitingInstruction waiting_after(
    std::size_t checkpoint,
    Eigen::Vector3d location);

  /// Get the last checkpoint that the traffic light knows has been reached.
  std::size_t last_reached() const;

  /// Update the location of the robot while it is idle. This means the robot is
  /// sitting somewhere without the intention of going anywhere.
  ///
  /// \param[in] map_name
  ///   The name of the reference map where the robot is located.
  ///
  /// \param[in] position
  ///   The (x, y, yaw) coordinates of the robot.
  EasyTrafficLight& update_idle_location(
    std::string map_name,
    Eigen::Vector3d position);

  /// Update the current battery level of the robot by specifying its state of
  /// charge as a fraction of its total charge capacity.
  EasyTrafficLight& update_battery_soc(double battery_soc);

  /// Tell the fleet adapter to replan. This can help to break out of deadlocks.
  EasyTrafficLight& replan();

  /// Specify a period for how often the fleet state message is published for
  /// this fleet. Passing in std::nullopt will disable the fleet state message
  /// publishing. The default value is 1s.
  EasyTrafficLight& fleet_state_publish_period(
    std::optional<rmf_traffic::Duration> value);

  /// This class will be provided to the deadlock_callback when a deadlock has
  /// occurred due to an unresolvable conflict. Human intervention may be
  /// required at this point, because the RMF traffic negotiation system does
  /// not have a high enough level of control over the conflicting participants
  /// to resolve it.
  class Blocker
  {
  public:

    /// Get the schedule participant ID of the blocker.
    rmf_traffic::schedule::ParticipantId participant_id() const;

    /// Get the description of the blocker.
    const rmf_traffic::schedule::ParticipantDescription& description() const;

    class Implementation;
  private:
    Blocker();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  class Implementation;
private:
  EasyTrafficLight();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using EasyTrafficLightPtr = std::shared_ptr<EasyTrafficLight>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EASYTRAFFICLIGHT_HPP
