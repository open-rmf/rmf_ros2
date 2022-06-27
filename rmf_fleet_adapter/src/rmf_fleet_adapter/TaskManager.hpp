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

#ifndef SRC__RMF_FLEET_ADAPTER__TASKMANAGER_HPP
#define SRC__RMF_FLEET_ADAPTER__TASKMANAGER_HPP

#include "LegacyTask.hpp"
#include "agv/RobotContext.hpp"
#include <rmf_websocket/BroadcastClient.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task/TaskPlanner.hpp>
#include <rmf_task/Activator.hpp>

#include <rmf_task_sequence/Event.hpp>

#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_task_msgs/msg/task_summary.hpp>

#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>

#include <mutex>
#include <set>

namespace rmf_fleet_adapter {

//==============================================================================
/// This task manager is a first attempt at managing multiple tasks per fleet.
/// This is a simple implementation that only makes a modest attempt at being
/// optimal. A better task manager would queue tasks across the whole fleet
/// instead of queuing tasks for each robot individually. We will attempt that
/// in a later implementation.
class TaskManager : public std::enable_shared_from_this<TaskManager>
{
public:

  static std::shared_ptr<TaskManager> make(
    agv::RobotContextPtr context,
    std::optional<std::weak_ptr<rmf_websocket::BroadcastClient>> broadcast_client,
    std::weak_ptr<agv::FleetUpdateHandle> fleet_handle);

  using Start = rmf_traffic::agv::Plan::Start;
  using StartSet = rmf_traffic::agv::Plan::StartSet;
  using Assignment = rmf_task::TaskPlanner::Assignment;
  using State = rmf_task::State;
  using RobotModeMsg = rmf_fleet_msgs::msg::RobotMode;
  using TaskProfileMsg = rmf_task_msgs::msg::TaskProfile;
  using TaskProfiles = std::unordered_map<std::string, TaskProfileMsg>;
  using TaskSummaryMsg = rmf_task_msgs::msg::TaskSummary;

  struct DirectAssignment
  {
    std::size_t sequence_number;
    Assignment assignment;

    DirectAssignment(
      std::size_t sequence_number_,
      Assignment assignment_)
    : sequence_number(sequence_number_),
      assignment(std::move(assignment_))
    {
      // Do nothing
    }
  };

  struct DirectQueuePriority
  {
    bool operator()(const DirectAssignment& a, const DirectAssignment& b) const
    {
      // Sort by start time and then sequence_number if tie
      const auto start_time_b =
        b.assignment.request()->booking()->earliest_start_time();
      const auto start_time_a =
        a.assignment.request()->booking()->earliest_start_time();

      if (start_time_b == start_time_a)
      {
        return b.sequence_number < a.sequence_number;
      }

      return start_time_b < start_time_a;
    }
  };

  using DirectQueue = std::set<
    DirectAssignment,
    DirectQueuePriority>;

  const agv::RobotContextPtr& context();

  agv::ConstRobotContextPtr context() const;

  std::optional<std::weak_ptr<rmf_websocket::BroadcastClient>> broadcast_client()
  const;

  /// Set the queue for this task manager with assignments generated from the
  /// task planner
  void set_queue(const std::vector<Assignment>& assignments);

  /// Get the non-charging requests among pending tasks
  const std::vector<rmf_task::ConstRequestPtr> requests() const;

  std::optional<std::string> current_task_id() const;

  const std::vector<Assignment>& get_queue() const;

  bool cancel_task_if_present(const std::string& task_id);

  std::string robot_status() const;

  /// The state of the robot. If the direct assignment queue is not empty,
  /// This will always return the state of the robot after completing all
  /// the tasks in the direct assignment queue. Otherwise, the current state
  /// is returned
  State expected_finish_state() const;

  /// Callback for the retreat timer. Appends a charging task to the task queue
  /// when robot is idle and battery level drops below a retreat threshold.
  void retreat_to_charger();

  /// Get the list of task ids for tasks that have started execution.
  /// The list will contain upto 100 latest task ids only.
  const std::vector<std::string>& get_executed_tasks() const;

  RobotModeMsg robot_mode() const;

  /// Get a vector of task logs that are validated against the schema
  std::vector<nlohmann::json> task_log_updates() const;

  /// Submit a direct task request to this manager
  ///
  /// \param[in] task_request
  ///   A JSON description of the task request. It should match the
  ///   task_request.json schema of rmf_api_msgs, in particular it must contain
  ///   `category` and `description` properties.
  ///
  /// \param[in] request_id
  ///   The unique ID for this task request.
  ///
  /// \return A robot_task_response.json message from rmf_api_msgs (note: this
  /// message is not validated before being returned).
  nlohmann::json submit_direct_request(
    const nlohmann::json& task_request,
    const std::string& request_id);

  class Interruption
  {
  public:
    void resume(std::vector<std::string> labels);

    ~Interruption();

    std::mutex mutex;
    std::weak_ptr<TaskManager> w_mgr;

    // Map from task_id to interruption token
    std::unordered_map<std::string, std::string> token_map;

    std::atomic_bool resumed = false;
  };

  /// Fully interrupt the robot, interrupting any active task(s) including
  /// emergency actions. Also immediately interrupt any new tasks that get
  /// started. This can be used to maintain human operator control of the robot
  /// and prevent RMF from performing any automated behaviors.
  ///
  /// \param[in] interruption
  ///   This will be filled with interruption tokens for each task that gets
  ///   interrupted. The owner of this resource should let it expire when the
  ///   interruption should be released.
  ///
  /// \param[in] labels
  ///   Labels related to the interruption to inform operators about the nature
  ///   of the interruption.
  void interrupt_robot(
    std::shared_ptr<Interruption> interruption,
    std::vector<std::string> labels,
    std::function<void()> robot_is_interrupted);

  /// Cancel a task for this robot. Returns true if the task was being managed
  /// by this task manager, or false if it was not.
  bool cancel_task(
    const std::string& task_id,
    std::vector<std::string> labels);

  /// Kill a task for this robot. Returns true if the task was being managed by
  /// this task manager, or false if it was not.
  bool kill_task(
    const std::string& task_id,
    std::vector<std::string> labels);

private:

  TaskManager(
    agv::RobotContextPtr context,
    std::optional<std::weak_ptr<rmf_websocket::BroadcastClient>> broadcast_client,
    std::weak_ptr<agv::FleetUpdateHandle>);

  class ActiveTask
  {
  public:
    ActiveTask();

    static ActiveTask start(
      rmf_task::Task::ActivePtr task,
      rmf_traffic::Time time);

    const std::string& id() const;

    void publish_task_state(TaskManager& mgr);

    operator bool() const
    {
      return static_cast<bool>(_task);
    }

    /// Adds an interruption
    std::string add_interruption(
      std::vector<std::string> labels,
      rmf_traffic::Time time,
      std::function<void()> task_is_interrupted);

    bool is_interrupted() const;

    bool is_finished() const;

    // Any unknown tokens that were included will be returned
    std::vector<std::string> remove_interruption(
      std::vector<std::string> for_tokens,
      std::vector<std::string> labels,
      rmf_traffic::Time time);

    void cancel(
      std::vector<std::string> labels,
      rmf_traffic::Time time);

    void kill(
      std::vector<std::string> labels,
      rmf_traffic::Time time);

    void rewind(uint64_t phase_id);

    std::string skip(
      uint64_t phase_id,
      std::vector<std::string> labels,
      rmf_traffic::Time time);

    std::vector<std::string> remove_skips(
      const std::vector<std::string>& for_tokens,
      std::vector<std::string> labels,
      rmf_traffic::Time time);

  private:
    rmf_task::Task::ActivePtr _task;
    rmf_traffic::Time _start_time;
    nlohmann::json _state_msg;

    std::unordered_map<std::string, nlohmann::json> _active_interruptions;
    std::unordered_map<std::string, nlohmann::json> _removed_interruptions;
    std::optional<rmf_task::Task::Active::Resume> _resume_task;

    struct InterruptionHandler
      : public std::enable_shared_from_this<InterruptionHandler>
    {
      std::mutex mutex;
      std::vector<std::function<void()>> interruption_listeners;
      bool is_interrupted = false;
    };

    std::shared_ptr<InterruptionHandler> _interruption_handler;

    std::optional<nlohmann::json> _cancellation;
    std::optional<nlohmann::json> _killed;

    struct SkipInfo
    {
      std::unordered_map<std::string, nlohmann::json> active_skips;
      std::unordered_map<std::string, nlohmann::json> removed_skips;
    };

    std::unordered_map<uint64_t, SkipInfo> _skip_info_map;

    uint64_t _next_token = 0;
  };

  friend class ActiveTask;

  agv::RobotContextPtr _context;
  std::optional<std::weak_ptr<rmf_websocket::BroadcastClient>> _broadcast_client;
  std::weak_ptr<agv::FleetUpdateHandle> _fleet_handle;
  rmf_task::ConstActivatorPtr _task_activator;
  ActiveTask _active_task;
  bool _emergency_active = false;
  std::optional<std::string> _emergency_pullover_interrupt_token;
  ActiveTask _emergency_pullover;
  uint16_t _count_emergency_pullover = 0;
  // Queue for dispatched tasks
  std::vector<Assignment> _queue;
  // An ID to keep track of the FIFO order of direct tasks
  std::size_t _next_sequence_number;
  // Queue for directly assigned tasks
  DirectQueue _direct_queue;
  rmf_utils::optional<Start> _expected_finish_location;
  rxcpp::subscription _task_sub;
  rxcpp::subscription _emergency_sub;

  /// This phase will kick in automatically when no task is being executed. It
  /// will ensure that the agent continues to respond to traffic negotiations so
  /// it does not become a blocker for other traffic participants.
  ActiveTask _waiting;
  uint16_t _count_waiting = 0;

  // TODO: Eliminate the need for a mutex by redesigning the use of the task
  // manager so that modifications of shared data only happen on designated
  // rxcpp worker
  mutable std::mutex _mutex;
  rclcpp::TimerBase::SharedPtr _task_timer;
  rclcpp::TimerBase::SharedPtr _retreat_timer;
  rclcpp::TimerBase::SharedPtr _update_timer;
  bool _task_state_update_available = true;
  std::chrono::steady_clock::time_point _last_update_time;

  // Container to keep track of tasks that have been started by this TaskManager
  // Use the _register_executed_task() to populate this container.
  std::vector<std::string> _executed_task_registry;

  // TravelEstimator for caching travel estimates for automatic charging
  // retreat. TODO(YV): Expose the TaskPlanner's TravelEstimator.
  std::shared_ptr<rmf_task::TravelEstimator> _travel_estimator;

  // Map schema url to schema for validator.
  // TODO: Get this and loader from FleetUpdateHandle
  std::unordered_map<std::string, nlohmann::json> _schema_dictionary = {};

  rmf_rxcpp::subscription_guard _task_request_api_sub;

  // Constant jsons with validated schemas for internal use
  // TODO(YV): Replace these with codegen tools
  const nlohmann::json _task_log_update_msg =
  {{"type", "task_log_update"}, {"data", {}}};
  const nlohmann::json _task_log_json =
  {{"task_id", {}}, {"log", {}}, {"phases", {{"log", {}}, {"events, {}"}}}};
  const nlohmann::json _task_state_update_json =
  {{"type", "task_state_update"}, {"data", {}}};
  const nlohmann::json _task_state_json =
  {{"booking", {}}, {"category", {}}, {"detail", {}},
    {"unix_millis_start_time", {}}, {"unix_millis_finish_time", {}},
    {"estimate_millis", {}}, {"phases", {}}, {"completed", {}}, {"active", {}},
    {"pending", {}}, {"interruptions", {}}, {"cancellation", {}},
    {"killed", {}}};

  rmf_task::Log::Reader _log_reader;

  // Map task_id to task_log.json for all tasks managed by this TaskManager
  std::unordered_map<std::string, nlohmann::json> _task_logs = {};

  /// Callback for task timer which begins next task if its deployment time has passed
  void _begin_next_task();

  // Interrupts that were issued when there was no active task. They will be
  // applied when a task becomes active.
  //
  // TODO(MXG): This entire system of robot interruptions is sloppy and fragile.
  // The implementation should be refactored with more meaningful and testable
  // encapsulations.
  struct RobotInterrupt
  {
    std::weak_ptr<Interruption> interruption;
    std::vector<std::string> labels;
    std::function<void()> robot_is_interrupted;
  };
  std::vector<RobotInterrupt> _robot_interrupts;

  void _process_robot_interrupts();

  std::function<void()> _robot_interruption_callback();

  /// Begin responsively waiting for the next task
  void _begin_waiting();

  /// Make the callback for resuming
  std::function<void()> _make_resume_from_emergency();

  /// Resume whatever the task manager should be doing
  void _resume_from_emergency();

  std::function<void()> _make_resume_from_waiting();

  /// Get the current state of the robot
  rmf_task::State _get_state() const;

  /// Check whether publishing should happen
  void _consider_publishing_updates();

  /// Publish the current task state
  void _publish_task_state();

  /// Publish one of the pending tasks
  rmf_task::State _publish_pending_task(
    const Assignment& assignment,
    rmf_task::State expected_state,
    const rmf_task::Parameters& parameters);

  /// Publish the current pending task list
  void _publish_task_queue();

  void _publish_canceled_pending_task(
    const Assignment& assignment,
    std::vector<std::string> labels);

  /// Cancel a task that is in the dispatch queue. Returns false if the task
  /// was not present.
  bool _cancel_task_from_dispatch_queue(
    const std::string& task_id,
    const std::vector<std::string>& labels);

  /// Cancel a task that is in the direct queue. Returns false if the task was
  /// not present.
  bool _cancel_task_from_direct_queue(
    const std::string& task_id,
    const std::vector<std::string>& labels);

  /// Schema loader for validating jsons
  void _schema_loader(
    const nlohmann::json_uri& id, nlohmann::json& value) const;

  /// Returns true if json is valid.
  // TODO: Move this into a utils?
  bool _validate_json(
    const nlohmann::json& json,
    const nlohmann::json_schema::json_validator& validator,
    std::string& error) const;

  /// If the request message is valid this will return true. If it is not valid,
  /// this will publish an error message for this request and return false. The
  /// caller should not attempt to process this request or respond to it any
  /// further.
  bool _validate_request_message(
    const nlohmann::json& request,
    const nlohmann::json_schema::json_validator& validator,
    const std::string& request_id);

  void _send_simple_success_response(
    const std::string& request_id);

  void _send_token_success_response(
    std::string token,
    const std::string& request_id);

  /// Make a validator for the given schema
  nlohmann::json_schema::json_validator _make_validator(
    const nlohmann::json& schema) const;

  void _send_simple_error_response(
    const std::string& request_id,
    uint64_t code,
    std::string category,
    std::string detail);

  void _send_simple_error_if_queued(
    const std::string& task_id,
    const std::string& request_id,
    const std::string& type);

  /// Make an error message to return
  static nlohmann::json _make_error_response(
    uint64_t code,
    std::string category,
    std::string detail);

  /// Validate and publish a json. This can be used for task
  /// state and log updates
  void _validate_and_publish_websocket(
    const nlohmann::json& msg,
    const nlohmann::json_schema::json_validator& validator) const;

  /// Validate and publish a response message over the ROS2 API response topic
  void _validate_and_publish_api_response(
    const nlohmann::json& msg,
    const nlohmann::json_schema::json_validator& validator,
    const std::string& request_id);

  /// Callback for when the task has a significat update
  std::function<void(rmf_task::Phase::ConstSnapshotPtr)> _update_cb();

  /// Callback for when the task reaches a checkpoint
  std::function<void(rmf_task::Task::Active::Backup)> _checkpoint_cb();

  /// Callback for when a phase within a task has finished
  std::function<void(rmf_task::Phase::ConstCompletedPtr)> _phase_finished_cb();

  /// Callback for when the task has finished
  std::function<void()> _task_finished(std::string id);

  /// Function to register the task id of a task that has begun execution
  /// The input task id will be inserted into the registry such that the max
  /// size of the registry is 100.
  void _register_executed_task(const std::string& id);

  void _populate_task_summary(
    std::shared_ptr<LegacyTask> task,
    uint32_t task_summary_state,
    TaskSummaryMsg& msg);

  void _handle_request(
    const std::string& request_msg,
    const std::string& request_id);

  void _handle_direct_request(
    const nlohmann::json& request_json,
    const std::string& request_id);

  void _handle_cancel_request(
    const nlohmann::json& request_json,
    const std::string& request_id);

  void _handle_kill_request(
    const nlohmann::json& request_json,
    const std::string& request_id);

  void _handle_interrupt_request(
    const nlohmann::json& request_json,
    const std::string& request_id);

  void _handle_resume_request(
    const nlohmann::json& request_json,
    const std::string& request_id);

  void _handle_rewind_request(
    const nlohmann::json& request_json,
    const std::string& request_id);

  void _handle_skip_phase_request(
    const nlohmann::json& request_json,
    const std::string& request_id);

  void _handle_undo_skip_phase_request(
    const nlohmann::json& request_json,
    const std::string& request_id);

};

using TaskManagerPtr = std::shared_ptr<TaskManager>;

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__TASKMANAGER_HPP
