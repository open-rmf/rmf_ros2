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

#ifndef RMF_FLEET_ADAPTER__AGV__FLEETUPDATEHANDLE_HPP
#define RMF_FLEET_ADAPTER__AGV__FLEETUPDATEHANDLE_HPP

#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/DevicePowerSink.hpp>
#include <rmf_battery/MotionPowerSink.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class FleetUpdateHandle : public std::enable_shared_from_this<FleetUpdateHandle>
{
public:

  /// Add a robot to this fleet adapter.
  ///
  /// \param[in] command
  ///   A reference to a command handle for this robot.
  ///
  /// \param[in] name
  ///   The name of this robot.
  ///
  /// \param[in] profile
  ///   The profile of this robot. This profile should account for the largest
  ///   possible payload that the robot might carry.
  ///
  /// \param[in] start
  ///   The initial location of the robot, expressed as a Plan::StartSet.
  ///   Multiple Start objects might be needed if the robot is not starting
  ///   precisely on a waypoint. The function
  ///   rmf_traffic::agv::compute_plan_starts() may help with this.
  ///
  /// \param[in] handle_cb
  ///   This callback function will get triggered when the RobotUpdateHandle is
  ///   ready to be used by the Fleet API side of the Adapter. Setting up a new
  ///   robot requires communication with the Schedule Node, so there may be a
  ///   delay before the robot is ready to be used.
  ///
  /// \return a handle to give the adapter updates about the robot.
  void add_robot(
      std::shared_ptr<RobotCommandHandle> command,
      const std::string& name,
      const rmf_traffic::Profile& profile,
      rmf_traffic::agv::Plan::StartSet start,
      std::function<void(std::shared_ptr<RobotUpdateHandle> handle)> handle_cb);

  /// Specify a set of lanes that should be closed.
  void close_lanes(std::vector<std::size_t> lane_indices);

  /// Specify a set of lanes that should be open.
  void open_lanes(std::vector<std::size_t> lane_indices);

  /// Set the parameters required for task planning. Without calling this
  /// function, this fleet will not bid for and accept tasks.
  ///
  /// \param[in] battery_system
  ///   Specify the battery system used by the vehicles in this fleet.
  ///
  /// \param[in] motion_sink
  ///   Specify the motion sink that describes the vehicles in this fleet.
  ///
  /// \param[in] ambient_sink
  ///   Specify the device sink for ambient sensors used by the vehicles in this fleet.
  ///
  /// \param[in] tool_sink
  ///   Specify the device sink for special tools used by the vehicles in this fleet.
  ///
  /// \param[in] recharge_threshold
  ///   The threshold for state of charge below which robots in this fleet
  ///   will cease to operate and require recharging. A value between 0.0 and
  ///   1.0 should be specified.
  ///
  /// \param[in] recharge_soc
  ///   The state of charge to which robots in this fleet should be charged up
  ///   to by automatic recharging tasks. A value between 0.0 and 1.0 should be
  ///   specified.
  ///
  /// \param[in] account_for_battery_drain
  ///   Specify whether battery drain is to be considered while allocating tasks.
  ///   If false, battery drain will not be considered when planning for tasks.
  ///   As a consequence, charging tasks will not be automatically assigned to
  ///   vehicles in this fleet when battery levels fall below the
  ///   recharge_threshold.
  ///
  /// \return true if task planner parameters were successfully updated.
  bool set_task_planner_params(
    std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink,
    double recharge_threshold,
    double recharge_soc,
    bool account_for_battery_drain);

  /// A callback function that evaluates whether a fleet will accept a task
  /// request
  ///
  /// \param[in] request
  ///   Information about the task request that is being considered.
  ///
  /// \return true to indicate that this fleet should accept the request, false
  /// to reject the request.
  using AcceptTaskRequest =
      std::function<bool(const rmf_task_msgs::msg::TaskProfile& profile)>;

  /// Provide a callback that indicates whether this fleet will accept a
  /// BidNotice request. By default all requests will be rejected.
  ///
  /// \note The callback function that you give should ideally be non-blocking
  /// and return quickly. It's meant to check whether this fleet's vehicles are
  /// compatible with the requested payload, pickup, and dropoff behavior
  /// settings. The path planning feasibility will be taken care of by the
  /// adapter internally.
  FleetUpdateHandle& accept_task_requests(AcceptTaskRequest check);

  /// A callback function that evaluates whether a fleet will accept a delivery
  /// request.
  ///
  /// \param[in] request
  ///   Information about the delivery request that is being considered.
  ///
  /// \return true to indicate that this fleet should accept the request, false
  /// to reject the request.
  ///
  using AcceptDeliveryRequest =
      std::function<bool(const rmf_task_msgs::msg::Delivery& request)>;

  /// Provide a callback that indicates whether this fleet will accept a
  /// delivery request. By default all delivery requests will be rejected.
  ///
  /// \note The callback function that you give should ideally be non-blocking
  /// and return quickly. It's meant to check whether this fleet's vehicles are
  /// compatible with the requested payload, pickup, and dropoff behavior
  /// settings. The path planning feasibility will be taken care of by the
  /// adapter internally.
  [[deprecated("Use accept_task_requests() instead")]]
  FleetUpdateHandle& accept_delivery_requests(AcceptDeliveryRequest check);

  /// Specify the default value for how high the delay of the current itinerary
  /// can become before it gets interrupted and replanned. A nullopt value will
  /// allow for an arbitrarily long delay to build up without being interrupted.
  FleetUpdateHandle& default_maximum_delay(
      std::optional<rmf_traffic::Duration> value);

  /// Get the default value for the maximum acceptable delay.
  rmf_utils::optional<rmf_traffic::Duration> default_maximum_delay() const;

  /// Specify a period for how often the fleet state message is published for
  /// this fleet. Passing in std::nullopt will disable the fleet state message
  /// publishing. The default value is 1s.
  FleetUpdateHandle& fleet_state_publish_period(
      std::optional<rmf_traffic::Duration> value);

  class Implementation;
private:
  FleetUpdateHandle();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using FleetUpdateHandlePtr = std::shared_ptr<FleetUpdateHandle>;
using ConstFleetUpdateHandlePtr = std::shared_ptr<const FleetUpdateHandle>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__FLEETUPDATEHANDLE_HPP
