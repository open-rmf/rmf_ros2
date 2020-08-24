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
    /// CommandHandle::receive_path_timing(~) whose version number is different
    /// from the return value of your latest call to update_path(~).
    ///
    /// This function should be called any time the robot changes its path, and
    /// it may also be a good idea to call it if significant delays or
    /// interruptions have accumulated.
    ///
    /// \param[in] new_path
    ///   Submit a new path that the robot intends to follow.
    std::size_t update_path(const std::vector<Waypoint>& new_path);

    /// Set the maximum acceptable delay before the timing gets recomputed. Pass
    /// in a nullopt to prevent the timing from ever being recomputed.
    UpdateHandle& maximum_delay(
        rmf_utils::optional<rmf_traffic::Duration> value);

    /// Get the maximum acceptable delay before the timing gets recomputed.
    ///
    /// \note The setter for this field is run asynchronously, so it may take
    /// some time before the getter has the same value that was given to the
    /// setter.
    rmf_utils::optional<rmf_traffic::Duration> maximum_delay() const;

    class Implementation;
  private:
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };

  using UpdateHandlePtr = std::shared_ptr<UpdateHandle>;

  /// Implement this class to receive traffic light commands from RMF.
  class CommandHandle
  {
  public:

    /// Use this callback function to keep the fleet adapter up to date on the
    /// progress of the vehicle.
    ///
    /// \param[in] path_index
    ///   The index of the path element that the robot is currently moving
    ///   towards.
    ///
    /// \param[in] location
    ///   The current (x, y, yaw) location of the robot.
    using ProgressCallback =
        std::function<void(std::size_t path_index, Eigen::Vector3d location)>;

    /// Receive the required timing for a path that has been submitted.
    ///
    /// This function may get called multiple times for the same path version,
    /// because the timing information may need to change as the traffic
    /// schedule gets updated and conflicts occur.
    ///
    /// \param[in] version
    ///   The version number of the path whose timing is being provided. If this
    ///   version number does not match the latest path that you submitted, then
    ///   simply ignore and discard the timing information.
    ///
    /// \param[in] departure_timing
    ///   This is a vector of the earliest times that the vehicle is allowed to
    ///   leave a given waypoint. The entries of this vector have a 1:1 mapping
    ///   with the entries of the path vector that was submitted earlier. Each
    ///   entry represents the earliest time that a robot is allowed to move
    ///   past its corresponding waypoint. If the robot arrives at the waypoint
    ///   before the given time, then the robot is obligated to pause until the
    ///   given time arrives.
    ///
    ///   \warning Sometimes a vehicle may need to pause after it has already
    ///   departed one of its waypoints but before it reaches the next one. This
    ///   will be conveyed by increasing the departure time of the last waypoint
    ///   that it left. If the departure timing of the last waypoint that the
    ///   robot departed from is increased to a value greater than the current
    ///   clock time, then the robot is obligated to stop in place (wherever it
    ///   happens to be) until the time is reached.
    ///
    /// \param[in] progress_updater
    ///   Use this callback to update the traffic light on how the robot has
    ///   progressed along the path.
    virtual void receive_path_timing(
        std::size_t version,
        const std::vector<rclcpp::Time>& departure_timing,
        ProgressCallback progress_updater) = 0;

    /// This function will be called when deadlock has occurred due to an
    /// unresolvable conflict. Human intervention may be required at this point,
    /// because the RMF traffic negotiation system does not have a high enough
    /// level of control over the conflicting participants to resolve it.
    virtual void deadlock() = 0;

    virtual ~CommandHandle() = default;
  };

  using CommandHandlePtr = std::shared_ptr<CommandHandle>;

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__TRAFFICLIGHT_HPP
