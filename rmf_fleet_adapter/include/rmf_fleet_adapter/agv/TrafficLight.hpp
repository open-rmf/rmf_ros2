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

#ifndef RMF_FLEET_ADAPTER__AGV__TRAFFICLIGHT_HPP
#define RMF_FLEET_ADAPTER__AGV__TRAFFICLIGHT_HPP

#include <rmf_fleet_adapter/agv/Waypoint.hpp>
#include <rmf_traffic/schedule/ParticipantDescription.hpp>

#include <memory>
#include <vector>

#include <rclcpp/time.hpp>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class TrafficLight
{
public:

  using Duration = rmf_traffic::Duration;

  /// A class for updating a traffic light manager about the intentions and
  /// state of its robot.
  class UpdateHandle
  {
  public:

    /// Update the traffic light with a new path for your robot.
    ///
    /// The traffic light manager is an async system, which means responses for
    /// this update will come at a later time. Due to race conditions, it's
    /// possible to get responses for old path updates that you no longer care
    /// about because you are operating off of a new path now. Save the return
    /// value of this function, and ignore all calls to
    /// CommandHandle::receive_checkpoints(~) whose version number is different
    /// from the return value of your latest call to update_path(~).
    ///
    /// This function should be called any time the robot changes the path that
    /// it intends to follow.
    ///
    /// \warning This function will throw an exception if there are less than 2
    /// waypoints in the path.
    ///
    /// \param[in] new_path
    ///   Submit a new path that the robot intends to follow.
    std::size_t follow_new_path(const std::vector<Waypoint>& new_path);

    /// Update the location of the robot while it is idle. Calling this function
    /// will clear the last path that was given to follow_new_path.
    ///
    /// \param[in] map_name
    ///   The name of the reference map where the robot is located.
    ///
    /// \param[in] position
    ///   The (x, y, yaw) coordinates of the robot.
    UpdateHandle& update_idle_location(
      std::string map_name,
      Eigen::Vector3d position);

    /// Update the current battery level of the robot by specifying its state of
    /// charge as a fraction of its total charge capacity.
    UpdateHandle& update_battery_soc(double battery_soc);

    /// Specify a period for how often the fleet state message is published for
    /// this fleet. Passing in std::nullopt will disable the fleet state message
    /// publishing. The default value is 1s.
    UpdateHandle& fleet_state_publish_period(
        std::optional<rmf_traffic::Duration> value);

    class Implementation;
  private:
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };

  using UpdateHandlePtr = std::shared_ptr<UpdateHandle>;

  /// Implement this class to receive traffic light commands from RMF.
  class CommandHandle
  {
  public:

    /// After departing from a checkpoint, use this callback to keep your
    /// location up to date.
    ///
    /// \param[in] location
    ///   The current <x, y, yaw> location of your robot.
    using Departed = std::function<void(Eigen::Vector3d location)>;
    using StoppedAt = Departed;

    /// Use this function signature to indicate that your robot is waiting for
    /// its next batch of waypoints.
    using OnStandby = std::function<void()>;

    /// Use this function signature to indicate that your robot cannot comply
    /// with the given plan because it has already passed one of the waypoints
    /// that it was not supposed to.
    ///
    /// When issuing a rejection, you should have the robot halt immediately and
    /// trigger the rejection when the robot comes to a halt.
    ///
    /// Provide the last waypoint that was departed from, and the current
    /// location of the robot once
    using Reject = std::function<void(
        std::size_t last_departed,
        Eigen::Vector3d stopped_location)>;

    /// The Checkpoint struct contains information about when the robot may
    /// depart from a Waypoint that was passed into
    /// UpdateHandle::follow_new_path().
    struct Checkpoint
    {
      /// The index of the new_path element that this Checkpoint is referring to
      std::size_t waypoint_index;

      /// The earliest time at which the robot is allowed to depart from this
      /// checkpoint.
      rclcpp::Time departure_time;

      /// After the robot has departed from this checkpoint, you should
      /// periodically trigger this callback with the current location of the
      /// robot.
      Departed departed;
    };

    /// Receive checkpoints for waypoints that have been submitted. Each
    /// checkpoint refers to one of the waypoints that was provided by the
    /// new_path argument of UpdateHandle::follow_new_path().
    ///
    /// This function may get triggered multiple times per path version. Each
    /// call will contain a new continguous sequence of checkpoints. A robot
    /// must not depart from a waypoint before it receives checkpoint
    /// information for that waypoint.
    ///
    /// After traffic negotiations happen, it is possible that some checkpoint
    /// departure times might get postponed or temporarily canceled. When that
    /// happens, this function will get triggered again for the same checkpoint
    /// indices. The latest values given by the checkpoints argument of this
    /// function are the values that need to be obeyed.
    ///
    /// When receiving a repeated batch of checkpoints, all old checkpoints from
    /// the new standby_at index and higher must be discarded.
    ///
    /// \param[in] version
    ///   The version number of the path whose timing is being provided. If this
    ///   version number does not match the latest path that you submitted, then
    ///   simply ignore and discard the timing information.
    ///
    /// \param[in] checkpoints
    ///   Receive a set of checkpoints that provide information about when the
    ///   robot is allowed to depart each waypoint, and callback functions to
    ///   keep the fleet adapter up to date on the robot's progress.
    ///
    /// \param[in] standby_at
    ///   Trigger the on_standby callback and have the robot wait once it
    ///   reaches this waypoint.
    ///
    /// \param[in] on_standby
    ///   Trigger this callback when the robot has arrived at the first waypoint
    ///   that it has not received a checkpoint for, or when the robot has
    ///   arrived at the last waypoint in its path.
    ///
    /// \param[in] reject
    ///   Trigger this callback if the plan must be rejected because the robot
    ///   has already passed the standby_at checkpoint. If you trigger this
    ///   callback, you must not trigger any of the new checkpoint departure
    ///   callbacks. Vice-versa, if you trigger any of the new checkpoint
    ///   departure callbacks, then you must not trigger this reject callback.
    ///   Before triggering any of these newly received callbacks, first
    ///   determine whether this new plan can be obeyed.
    virtual void receive_checkpoints(
        std::size_t version,
        std::vector<Checkpoint> checkpoints,
        std::size_t standby_at,
        OnStandby on_standby,
        Reject reject) = 0;

    /// Immediately stop until the specified time.
    ///
    /// This command is canceled if resume() is called afterwards.
    ///
    /// If a subsequent call is made to immediately_stop_until(~), then the new
    /// time overrides the old time, even if the new time is sooner.
    ///
    /// If the specified time has already passed by the time this function is
    /// called, then there is no need to stop.
    ///
    /// \param[in] version
    ///   The version number of the path whose timing is being provided. If this
    ///   version number does not match the latest path that you submitted, then
    ///   simply ignore and discard the timing information.
    ///
    /// \param[in] time
    ///   The time to wait until
    ///
    /// \param[in] stopped_at
    ///   Trigger this callback while the robot is stopped and update the
    ///   location. As long as the robot truly comes to a stop, this only needs
    ///   to be called once.
    ///
    /// \param[in] departed
    ///   Trigger this callback when the robot departs from this stop.
    virtual void immediately_stop_until(
        std::size_t version,
        rclcpp::Time time,
        StoppedAt stopped_at,
        Departed departed) = 0;

    /// Resume travel, even if immediately_stop_until(~) was activated and the
    /// given time has not been reached yet.
    ///
    /// \param[in] version
    ///   The version number of the path whose timing is being provided. If this
    ///   version number does not match the latest path that you submitted, then
    ///   simply ignore and discard the timing information.
    virtual void resume(std::size_t version) = 0;

    /// This class is given to the deadlock() function to describe the
    /// participants that are blocking the robot and creating the deadlock.
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

    /// This function will be called when deadlock has occurred due to an
    /// unresolvable conflict. Human intervention may be required at this point,
    /// because the RMF traffic negotiation system does not have a high enough
    /// level of control over the conflicting participants to resolve it.
    virtual void deadlock(std::vector<Blocker> blockers) = 0;

    virtual ~CommandHandle() = default;
  };

  using CommandHandlePtr = std::shared_ptr<CommandHandle>;

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__TRAFFICLIGHT_HPP
