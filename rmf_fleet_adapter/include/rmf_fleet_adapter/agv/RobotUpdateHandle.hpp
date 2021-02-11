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
#include <rmf_utils/optional.hpp>

#include <rmf_traffic/schedule/Participant.hpp>

#include <Eigen/Geometry>

#include <vector>
#include <memory>
#include <functional>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// You will be given an instance of this class every time you add a new robot
/// to your fleet. Use that instance to send updates to RoMi-H about your
/// robot's state.
class RobotUpdateHandle
{
public:

  /// Tell the RMF schedule that the robot was interrupted and needs a new plan.
  /// A new plan will be generated, starting from the last position that was
  /// given by update_position(). It is best to call update_position() with the
  /// latest position of the robot before calling this function.
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
  /// position of the robot and the waypoint that it is moving towards.
  ///
  /// This should be used if the robot has diverged significantly from its
  /// course but it is merging back onto a waypoint.
  void update_position(
      const Eigen::Vector3d& position,
      std::size_t target_waypoint);

  /// Update the current position of the robot by specifying the x, y, yaw
  /// position of the robot and what map the robot is on.
  ///
  /// \warning This function should only be used if the robot has diverged from
  /// the navigation graph somehow.
  ///
  /// We will attempt to merge the robot back onto the navigation graph. The
  /// parameters for this function are passed along to
  /// rmf_traffic::agv::compute_plan_starts().
  void update_position(
      const std::string& map_name,
      const Eigen::Vector3d& position,
      const double max_merge_waypoint_distance = 0.1,
      const double max_merge_lane_distance = 1.0,
      const double min_lane_length = 1e-8);

  /// Set the waypoint where the charger for this robot is located.
  /// If not specified, the nearest waypoint in the graph with the is_charger()
  /// property will be assumed as the charger for this robot.
  RobotUpdateHandle& set_charger_waypoint(const std::size_t charger_wp);

  /// Update the current battery level of the robot by specifying its state of
  /// charge as a fraction of its total charge capacity
  void update_battery_soc(const double battery_soc);

  /// Specify how high the delay of the current itinerary can become before it
  /// gets interrupted and replanned. A nullopt value will allow for an
  /// arbitrarily long delay to build up without being interrupted.
  RobotUpdateHandle& maximum_delay(
      rmf_utils::optional<rmf_traffic::Duration> value);

  /// Get the value for the maximum delay.
  ///
  /// \note The setter for the maximum_delay field is run asynchronously, so
  /// it may take some time for the return value of this getter to match the
  /// value that was given to the setter.
  rmf_utils::optional<rmf_traffic::Duration> maximum_delay() const;

  class Implementation;

  /// This API is experimental and will not be supported in the future. Users
  /// are to avoid relying on these feature for any integration.
  class Unstable
  {
  public:
    /// Get the schedule participant of this robot
    rmf_traffic::schedule::Participant* get_participant();

    enum class Decision
    {
      Undefined = 0,
      Clear = 1,
      Crowded = 2
    };

    /// A callback with this signature will be given to the watchdog when the
    /// robot is ready to enter a lift. If the watchdog passes in a true, then
    /// the robot will proceed to enter the lift. If the watchdog passes in a
    /// false, then the fleet adapter will release its session with the lift and
    /// resume later.
    using Decide = std::function<void(Decision)>;

    using Watchdog = std::function<void(const std::string&, Decide)>;

    /// Set a callback that can be used to check whether the robot is clear to
    /// enter the lift.
    void set_lift_entry_watchdog(
      Watchdog watchdog,
      rmf_traffic::Duration wait_duration = std::chrono::seconds(10));

  private:
    friend Implementation;
    Implementation* _pimpl;
  };

  /// Get a mutable reference to the unstable API extension
  Unstable& unstable();
  /// Get a const reference to the unstable API extension
  const Unstable& unstable() const;

private:
  RobotUpdateHandle();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using RobotUpdateHandlePtr = std::shared_ptr<RobotUpdateHandle>;
using ConstRobotUpdateHandlePtr = std::shared_ptr<const RobotUpdateHandle>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__ROBOTUPDATEHANDLE_HPP
