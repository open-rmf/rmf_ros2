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

#include <rmf_task/RequestFactory.hpp>

#include <nlohmann/json.hpp>
#include <rmf_task/Activator.hpp>
#include <rmf_task_sequence/Phase.hpp>
#include <rmf_task_sequence/Event.hpp>

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

  /// Confirmation is a class used by the task acceptance callbacks to decide if
  /// a task description should be accepted.
  class Confirmation
  {
  public:

    /// Constructor
    Confirmation();

    /// Call this function to decide that you want to accept the task request.
    /// If this function is never called, it will be assumed that the task is
    /// rejected.
    Confirmation& accept();

    /// Check whether
    bool is_accepted() const;

    /// Call this function to bring attention to errors related to the task
    /// request. Each call to this function will overwrite any past calls, so
    /// it is recommended to only call it once.
    Confirmation& errors(std::vector<std::string> error_messages);

    /// Call this function to add errors instead of overwriting the ones that
    /// were already there.
    Confirmation& add_errors(std::vector<std::string> error_messages);

    /// Check the errors that have been given to this confirmation.
    const std::vector<std::string>& errors() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Signature for a callback that decides whether to accept a specific
  /// category of task request.
  ///
  /// \param[in] description
  ///   A description of the task that is being considered
  ///
  /// \param[in] confirm
  ///   Use this object to decide if you want to accept the task
  using ConsiderRequest =
    std::function<void(
        const nlohmann::json& description,
        Confirmation& confirm)
    >;

  /// Allow this fleet adapter to consider delivery requests.
  ///
  /// Pass in nullptrs to disable delivery requests.
  ///
  /// By default, delivery requests are not accepted until you provide these
  /// callbacks.
  ///
  /// The FleetUpdateHandle will ensure that the requests are feasible for the
  /// robots before triggering these callbacks.
  ///
  /// \param[in] consider_pickup
  ///   Decide whether to accept a pickup request. The description will satisfy
  ///   the event_description_PickUp.json schema of rmf_fleet_adapter.
  ///
  /// \param[in] consider_dropoff
  ///   Decide whether to accept a dropoff request. The description will satisfy
  ///   the event_description_DropOff.json schema of rmf_fleet_adapter.
  FleetUpdateHandle& consider_delivery_requests(
    ConsiderRequest consider_pickup,
    ConsiderRequest consider_dropoff);

  /// Allow this fleet adapter to consider cleaning requests.
  ///
  /// Pass in a nullptr to disable cleaning requests.
  ///
  /// By default, cleaning requests are not accepted until you provide this
  /// callback.
  ///
  /// \param[in] consider
  ///   Decide whether to accept a cleaning request. The description will
  ///   satisfy the event_description_Clean.json schema of rmf_fleet_adapter.
  ///   The FleetUpdateHandle will ensure that the request is feasible for the
  ///   robots before triggering this callback.
  FleetUpdateHandle& consider_cleaning_requests(ConsiderRequest consider);

  /// Allow this fleet adapter to consider patrol requests.
  ///
  /// Pass in a nullptr to disable patrol requests.
  ///
  /// By default, patrol requests are always accepted.
  ///
  /// \param[in] consider
  ///   Decide whether to accept a patrol request. The description will satisfy
  ///   the task_description_Patrol.json schema of rmf_fleet_adapter. The
  ///   FleetUpdateHandle will ensure that the request is feasible for the
  ///   robots before triggering this callback.
  FleetUpdateHandle& consider_patrol_requests(ConsiderRequest consider);

  /// Allow this fleet adapter to consider composed requests.
  ///
  /// Pass in a nullptr to disable composed requests.
  ///
  /// By default, composed requests are always accepted, as long as the events
  /// that they are composed of are accepted.
  ///
  /// \param[in] consider
  ///   Decide whether to accept a composed request. The description will
  ///   satisfy the task_description_Compose.json schema of rmf_fleet_adapter.
  ///   The FleetUpdateHandle will ensure that the request is feasible for the
  ///   robots before triggering this callback.
  FleetUpdateHandle& consider_composed_requests(ConsiderRequest consider);

  /// Allow this fleet adapter to execute a PerformAction activity of specified
  /// category which may be present in sequence event.
  ///
  /// \param[in] category
  ///   A string that categorizes the action. This value should be used when
  ///   filling out the category field in event_description_PerformAction.json
  ///   schema.
  ///
  /// \param[in] consider
  ///   Decide whether to accept the action based on the description field in
  ///   event_description_PerformAction.json schema.
  FleetUpdateHandle& add_performable_action(
    const std::string& category,
    ConsiderRequest consider);

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
  /// \param[in] finishing_request
  ///   A factory for a request that should be performed by each robot in this
  ///   fleet at the end of its assignments.
  ///
  /// \return true if task planner parameters were successfully updated.
  bool set_task_planner_params(
    std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink,
    double recharge_threshold,
    double recharge_soc,
    bool account_for_battery_drain,
    rmf_task::ConstRequestFactoryPtr finishing_requst = nullptr);

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
  [[deprecated("Use the consider_..._requests functions instead")]]
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
  [[deprecated("Use consider_delivery_requests() instead")]]
  FleetUpdateHandle& accept_delivery_requests(AcceptDeliveryRequest check);

  /// Specify the default value for how high the delay of the current itinerary
  /// can become before it gets interrupted and replanned. A nullopt value will
  /// allow for an arbitrarily long delay to build up without being interrupted.
  FleetUpdateHandle& default_maximum_delay(
    std::optional<rmf_traffic::Duration> value);

  /// Get the default value for the maximum acceptable delay.
  std::optional<rmf_traffic::Duration> default_maximum_delay() const;

  /// The behavior is identical to fleet_state_topic_publish_period
  [[deprecated("Use fleet_state_topic_publish_period instead")]]
  FleetUpdateHandle& fleet_state_publish_period(
    std::optional<rmf_traffic::Duration> value);

  /// Specify a period for how often the fleet state message is published for
  /// this fleet. Passing in std::nullopt will disable the fleet state message
  /// publishing. The default value is 1s.
  FleetUpdateHandle& fleet_state_topic_publish_period(
    std::optional<rmf_traffic::Duration> value);

  /// Specify a period for how often the fleet state is updated in the database
  /// and to the API server. This is separate from publishing the fleet state
  /// over the ROS2 fleet state topic. Passing in std::nullopt will disable
  /// the updating, but this is not recommended unless you intend to provide the
  /// API server with the fleet states through some other means.
  ///
  /// The default value is 1s.
  FleetUpdateHandle& fleet_state_update_period(
    std::optional<rmf_traffic::Duration> value);

  /// Set a callback for listening to update messages (e.g. fleet states and
  /// task updates). This will not receive any update messages that happened
  /// before the listener was set.
  FleetUpdateHandle& set_update_listener(
    std::function<void(const nlohmann::json&)> listener);

  // Do not allow moving
  FleetUpdateHandle(FleetUpdateHandle&&) = delete;
  FleetUpdateHandle& operator=(FleetUpdateHandle&&) = delete;

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
