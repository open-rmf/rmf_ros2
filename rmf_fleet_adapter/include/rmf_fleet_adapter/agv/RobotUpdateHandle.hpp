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
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

#include <rmf_traffic/schedule/Participant.hpp>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

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

  [[deprecated("Use replan() instead")]]
  void interrupted();

  /// Tell the RMF schedule that the robot needs a new plan. A new plan will be
  /// generated, starting from the last position that was given by
  /// update_position(). It is best to call update_position() with the latest
  /// position of the robot before calling this function.
  void replan();

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

  /// Update the current position of the robot by specifying a plan start set
  /// for it.
  void update_position(rmf_traffic::agv::Plan::StartSet position);

  /// Set the waypoint where the charger for this robot is located.
  /// If not specified, the nearest waypoint in the graph with the is_charger()
  /// property will be assumed as the charger for this robot.
  RobotUpdateHandle& set_charger_waypoint(const std::size_t charger_wp);

  /// Update the current battery level of the robot by specifying its state of
  /// charge as a fraction of its total charge capacity, i.e. a value from 0.0
  /// to 1.0.
  void update_battery_soc(const double battery_soc);

  /// Use this function to override the robot status. The string provided must
  /// be a valid enum as specified in the robot_state.json schema.
  /// Pass std::nullopt to cancel the override and allow RMF to automatically
  /// update the status. The default value is std::nullopt.
  void override_status(std::optional<std::string> status);

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

  /// Get the current task ID of the robot, or an empty string if the robot
  /// is not performing any task.
  const std::string current_task_id() const;

  /// Unique identifier for an activity that the robot is performing. Used by
  /// the EasyFullControl API.
  class ActivityIdentifier
  {
  public:

    /// Compare whether two activity handles are referring to the same activity.
    bool operator==(const ActivityIdentifier&) const;

    class Implementation;
  private:
    ActivityIdentifier();
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };
  using ActivityIdentifierPtr = std::shared_ptr<ActivityIdentifier>;
  using ConstActivityIdentifierPtr = std::shared_ptr<const ActivityIdentifier>;

  /// Hold onto this class to tell the robot to behave as a "stubborn
  /// negotiator", meaning it will always refuse to accommodate the schedule
  /// of any other agent. This could be used when teleoperating a robot, to
  /// tell other robots that the agent is unable to negotiate.
  ///
  /// When the object is destroyed, the stubbornness will automatically be
  /// released.
  class Stubbornness
  {
  public:
    /// Stop being stubborn
    void release();

    class Implementation;
  private:
    Stubbornness();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The ActionExecution class should be used to manage the execution of and
  /// provide updates on ongoing actions.
  class ActionExecution
  {
  public:
    /// Update the amount of time remaining for this action.
    /// This does not need to be used for navigation requests.
    void update_remaining_time(rmf_traffic::Duration remaining_time_estimate);

    /// Set task status to underway and optionally log a message (info tier)
    void underway(std::optional<std::string> text);

    /// Set task status to error and optionally log a message (error tier)
    void error(std::optional<std::string> text);

    /// Set the task status to delayed and optionally log a message
    /// (warning tier)
    void delayed(std::optional<std::string> text);

    /// Set the task status to blocked and optionally log a message
    /// (warning tier)
    void blocked(std::optional<std::string> text);

    /// Use this to override the traffic schedule for the agent while it performs
    /// this command.
    ///
    /// If the given trajectory results in a traffic conflict then a negotiation
    /// will be triggered. Hold onto the `Stubbornness` returned by this function
    /// to ask other agents to plan around your trajectory, otherwise the
    /// negotiation may result in a replan for this agent and a new command will
    /// be issued.
    ///
    /// \note Using this will function always trigger a replan once the agent
    /// finishes the command.
    ///
    /// \warning Too many overridden/stubborn agents can cause a deadlock. It's
    ///   recommended to use this API sparingly and only over short distances or
    ///   small deviations.
    ///
    /// \param[in] map
    ///   Name of the map where the trajectory will take place
    ///
    /// \param[in] path
    ///   The path of the agent
    ///
    /// \param[in] hold
    ///   How long the agent will wait at the end of the path
    ///
    /// \return a Stubbornness handle that tells the fleet adapter to not let the
    /// overridden path be negotiated. The returned handle will stop having an
    /// effect after this command execution is finished.
    Stubbornness override_schedule(
      std::string map,
      std::vector<Eigen::Vector3d> path,
      rmf_traffic::Duration hold = rmf_traffic::Duration(0));

    /// Trigger this when the action is successfully finished.
    /// No other functions in this ActionExecution instance will
    /// be usable after this.
    void finished();

    /// Returns false if the Action has been killed or cancelled
    bool okay() const;

    /// Activity identifier for this action. Used by the EasyFullControl API.
    ConstActivityIdentifierPtr identifier() const;

    class Implementation;
  private:
    ActionExecution();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Signature for a callback to request the robot to perform an action
  ///
  /// \param[in] category
  ///   A category of the action to be performed
  ///
  /// \param[in] description
  ///   A description of the action to be performed
  ///
  /// \param[in] execution
  ///   An ActionExecution object that will be provided to the user for
  ///   updating the state of the action.
  using ActionExecutor = std::function<void(
        const std::string& category,
        const nlohmann::json& description,
        ActionExecution execution)>;

  /// Set the ActionExecutor for this robot
  void set_action_executor(ActionExecutor action_executor);

  /// Submit a direct task request to this manager
  /// \param[in] task_request
  ///   A JSON description of the task request. It should match the
  ///   task_request.json schema of rmf_api_msgs, in particular it must contain
  ///   `category` and `description` properties.
  ///
  /// \param[in] request_id
  ///   The unique ID for this task request.
  ///
  /// \param[in] receive_response
  ///   Provide a callback to receive the response. The response will be a
  ///   robot_task_response.json message from rmf_api_msgs (note: this message
  ///   is not validated before being returned).
  void submit_direct_request(
    nlohmann::json task_request,
    std::string request_id,
    std::function<void(nlohmann::json response)> receive_response);

  /// An object to maintain an interruption of the current task. When this
  /// object is destroyed, the task will resume.
  class Interruption
  {
  public:
    /// Call this function to resume the task while providing labels for
    /// resuming.
    void resume(std::vector<std::string> labels);

    class Implementation;
  private:
    Interruption();
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };

  /// Interrupt (pause) the current task, yielding control of the robot away
  /// from the fleet adapter's task manager.
  ///
  /// \param[in] labels
  ///   Labels that will be assigned to this interruption. It is recommended to
  ///   include information about why the interruption is happening.
  ///
  /// \return a handle for this interruption.
  Interruption interrupt(
    std::vector<std::string> labels,
    std::function<void()> robot_is_interrupted);

  /// Cancel a task, if it has been assigned to this robot
  ///
  /// \param[in] task_id
  ///   The ID of the task to be canceled
  ///
  /// \param[in] labels
  ///   Labels that will be assigned to this cancellation. It is recommended to
  ///   include information about why the cancellation is happening.
  ///
  /// \param[in] on_cancellation
  ///   Callback that will be triggered after the cancellation is issued.
  ///   task_was_found will be true if the task was successfully found and
  ///   issued the cancellation, false otherwise.
  void cancel_task(
    std::string task_id,
    std::vector<std::string> labels,
    std::function<void(bool task_was_found)> on_cancellation);

  /// Kill a task, if it has been assigned to this robot
  ///
  /// \param[in] task_id
  ///   The ID of the task to be canceled
  ///
  /// \param[in] labels
  ///   Labels that will be assigned to this cancellation. It is recommended to
  ///   include information about why the cancellation is happening.
  ///
  /// \param[in] on_kill
  ///   Callback that will be triggered after the cancellation is issued.
  ///   task_was_found will be true if the task was successfully found and
  ///   issued the kill, false otherwise.
  void kill_task(
    std::string task_id,
    std::vector<std::string> labels,
    std::function<void(bool task_was_found)> on_kill);

  enum class Tier
  {
    /// General status information, does not require special attention
    Info,

    /// Something unusual that might require attention
    Warning,

    /// A critical failure that requires immediate operator attention
    Error
  };

  /// An object to maintain an issue that is happening with the robot. When this
  /// object is destroyed without calling resolve(), the issue will be
  /// "dropped", which issues a warning to the log.
  class IssueTicket
  {
  public:

    /// Indicate that the issue has been resolved. The provided message will be
    /// logged for this robot and the issue will be removed from the robot
    /// state.
    void resolve(nlohmann::json msg);

    class Implementation;
  private:
    IssueTicket();
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };

  /// Create a new issue for the robot.
  ///
  /// \param[in] tier
  ///   The severity of the issue
  ///
  /// \param[in] category
  ///   A brief category to describe the issue
  ///
  /// \param[in] detail
  ///   Full details of the issue that might be relevant to an operator or
  ///   logging system.
  ///
  /// \return A ticket for this issue
  IssueTicket create_issue(
    Tier tier, std::string category, nlohmann::json detail);

  // TODO(MXG): Should we offer a "clear_all_issues" function?

  /// Add a log entry with Info severity
  void log_info(std::string text);

  /// Add a log entry with Warning severity
  void log_warning(std::string text);

  /// Add a log entry with Error severity
  void log_error(std::string text);

  /// Toggle the responsive wait behavior for this robot. When responsive wait
  /// is active, the robot will remain in the traffic schedule when it is idle
  /// and will negotiate its position with other traffic participants to
  /// potentially move out of their way.
  ///
  /// Disabling this behavior may be helpful to reduce CPU load or prevent
  /// parked robots from moving or being seen as conflicts when they are not
  /// actually at risk of creating traffic conflicts.
  ///
  /// By default this behavior is enabled.
  void enable_responsive_wait(bool value);

  /// If the robot is holding onto a session with a lift, release that session.
  void release_lift();

  /// A description of whether the robot should accept dispatched and/or direct
  /// tasks.
  class Commission
  {
  public:
    /// Construct a Commission description with all default values.
    /// - accept_dispatched_tasks: true
    /// - accept_direct_tasks: true
    /// - is_performing_idle_behavior: true
    Commission();

    /// Construct a Commission description that accepts no tasks at all.
    /// - accept_dispatch_tasks: false
    /// - accept_direct_tasks: false
    /// - is_performing_idle_behavior: false
    static Commission decommission();

    /// Set whether this commission should accept dispatched tasks.
    Commission& accept_dispatched_tasks(bool decision = true);

    /// Check whether this commission is accepting dispatched tasks.
    bool is_accepting_dispatched_tasks() const;

    /// Set whether this commission should accept direct tasks
    Commission& accept_direct_tasks(bool decision = true);

    /// Check whether this commission is accepting direct tasks.
    bool is_accepting_direct_tasks() const;

    /// Set whether this commission should perform idle behaviors (formerly
    /// referred to as "finishing tasks").
    Commission& perform_idle_behavior(bool decision = true);

    /// Check whether this commission is performing idle behaviors (formerly
    /// referred to as "finishing tasks").
    bool is_performing_idle_behavior() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Set the current commission for the robot.
  void set_commission(Commission commission);

  /// Get the current commission for the robot. If the robot has been dropped
  /// from the fleet, this will return Commission::decommission().
  Commission commission() const;

  /// Tell the fleet adapter to reassign all the tasks that have been dispatched
  /// to this robot. To prevent the tasks from being reassigned back to this
  /// robot use .set_commission(Commission::decommission())
  ///
  /// In the current implementation, tasks will only be reassigned to robots
  /// in the same fleet that the task was originally assigned to. This behavior
  /// could change in the future.
  void reassign_dispatched_tasks();

  class Implementation;

  /// This API is experimental and will not be supported in the future. Users
  /// are to avoid relying on these feature for any integration.
  class Unstable
  {
  public:
    /// True if this robot is allowed to accept new tasks. False if the robot
    /// will not accept any new tasks.
    [[deprecated("Use commission instead")]]
    bool is_commissioned() const;

    /// Stop this robot from accepting any new tasks. It will continue to
    /// perform tasks that are already in its queue. To reassign those tasks,
    /// you will need to use the task request API to cancel the tasks and
    /// re-request them.
    [[deprecated("Use set_commission instead")]]
    void decommission();

    /// Allow this robot to resume accepting new tasks.
    [[deprecated("Use set_commission instead")]]
    void recommission();

    /// Get the schedule participant of this robot
    rmf_traffic::schedule::Participant* get_participant();

    /// Change the radius of the footprint and vicinity of this participant.
    void change_participant_profile(
      double footprint_radius,
      double vicinity_radius);

    /// Override the schedule to say that the robot will be holding at a certain
    /// position. This should not be used while tasks with automatic schedule
    /// updating are running, or else the traffic schedule will have jumbled up
    /// information, which can be disruptive to the overall traffic management.
    void declare_holding(
      std::string on_map,
      Eigen::Vector3d at_position,
      rmf_traffic::Duration for_duration = std::chrono::seconds(30));

    /// Get the current Plan ID that this robot has sent to the traffic schedule
    rmf_traffic::PlanId current_plan_id() const;

    using Stubbornness = RobotUpdateHandle::Stubbornness;

    /// Tell this robot to be a stubborn negotiator.
    Stubbornness be_stubborn();

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

    /// Turn on/off a debug dump of how position updates are being processed
    void debug_positions(bool on);

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
