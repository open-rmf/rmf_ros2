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
#include "BroadcastClient.hpp"

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task/TaskPlanner.hpp>
#include <rmf_task/Activator.hpp>

#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_task_msgs/msg/task_summary.hpp>

#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>

#include <mutex>

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
    std::weak_ptr<BroadcastClient> broadcast_client);

  using Start = rmf_traffic::agv::Plan::Start;
  using StartSet = rmf_traffic::agv::Plan::StartSet;
  using Assignment = rmf_task::TaskPlanner::Assignment;
  using State = rmf_task::State;
  using RobotModeMsg = rmf_fleet_msgs::msg::RobotMode;
  using TaskProfileMsg = rmf_task_msgs::msg::TaskProfile;
  using TaskProfiles = std::unordered_map<std::string, TaskProfileMsg>;
  using TaskSummaryMsg = rmf_task_msgs::msg::TaskSummary;

  /// The location where we expect this robot to be at the end of its current
  /// task queue.
  StartSet expected_finish_location() const;

  const agv::RobotContextPtr& context();

  agv::ConstRobotContextPtr context() const;

  std::weak_ptr<BroadcastClient> broadcast_client() const;

  const LegacyTask* current_task() const;

  /// Set the queue for this task manager with assignments generated from the
  /// task planner
  void set_queue(
    const std::vector<Assignment>& assignments,
    const TaskProfiles& task_profiles = {});

  /// Get the non-charging requests among pending tasks
  const std::vector<rmf_task::ConstRequestPtr> requests() const;

  /// The state of the robot.
  State expected_finish_state() const;

  /// Callback for the retreat timer. Appends a charging task to the task queue
  /// when robot is idle and battery level drops below a retreat threshold.
  void retreat_to_charger();

  /// Get the list of task ids for tasks that have started execution.
  /// The list will contain upto 100 latest task ids only.
  const std::vector<std::string>& get_executed_tasks() const;

  RobotModeMsg robot_mode() const;

  /// Get a vector of task logs that are validaed against the schema
  std::vector<nlohmann::json> task_log_updates() const;

private:

  TaskManager(
    agv::RobotContextPtr context,
    std::weak_ptr<BroadcastClient> broadcast_client);

  agv::RobotContextPtr _context;
  std::weak_ptr<BroadcastClient> _broadcast_client;
  std::shared_ptr<LegacyTask> _active_task;
  std::vector<std::shared_ptr<LegacyTask>> _queue;
  rmf_utils::optional<Start> _expected_finish_location;
  rxcpp::subscription _task_sub;
  rxcpp::subscription _emergency_sub;

  /// This phase will kick in automatically when no task is being executed. It
  /// will ensure that the agent continues to respond to traffic negotiations so
  /// it does not become a blocker for other traffic participants.
  std::shared_ptr<LegacyTask::ActivePhase> _waiting;

  // TODO: Eliminate the need for a mutex by redesigning the use of the task
  // manager so that modifications of shared data only happen on designated
  // rxcpp worker
  std::mutex _mutex;
  rclcpp::TimerBase::SharedPtr _task_timer;
  rclcpp::TimerBase::SharedPtr _retreat_timer;

  // Container to keep track of tasks that have been started by this TaskManager
  // Use the _register_executed_task() to populate this container.
  std::vector<std::string> _executed_task_registry;

  // TravelEstimator for caching travel estimates for automatic charging
  // retreat. TODO(YV): Expose the TaskPlanner's TravelEstimator.
  std::shared_ptr<rmf_task::TravelEstimator> _travel_estimator;

  // The activator to generate rmf_task::Task::Active instances from requests
  std::shared_ptr<rmf_task::Activator> _activator;

  // Map schema url to schema for validator.
  // TODO: Get this and loader from FleetUpdateHandle
  std::unordered_map<std::string, nlohmann::json> _schema_dictionary = {};

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

  // The task_state.json for the active task. This should be initialized when
  // a request is activated.
  // TODO: Should this be a shared_ptr to pass down to LegacyTask::Active?
  nlohmann::json _active_task_state;

  // Map task_id to task_log.json for all tasks managed by this TaskManager
  std::unordered_map<std::string, nlohmann::json> _task_logs = {};

  /// Callback for task timer which begins next task if its deployment time has passed
  void _begin_next_task();

  /// Begin responsively waiting for the next task
  void _begin_waiting();

  /// Get the current state of the robot
  rmf_task::State _get_state() const;

  /// Schema loader for validating jsons
  void _schema_loader(
    const nlohmann::json_uri& id, nlohmann::json& value) const;

  /// Returns true if json is valid.
  // TODO: Move this into a utils?
  bool _validate_json(
    const nlohmann::json& json,
    const nlohmann::json& schema,
    std::string& error) const;

  /// Validate and publish a json. This can be used for task
  /// state and log updates
  void _validate_and_publish_json(
    const nlohmann::json& msg,
    const nlohmann::json& schema) const;

  /// Callback for when the task has a significat update
  void _update(rmf_task::Phase::ConstSnapshotPtr snapshot);

  /// Callback for when the task reaches a checkpoint
  void _checkpoint(rmf_task::Task::Active::Backup backup);

  /// Callback for when a phase within a task has finished
  void _phase_finished(rmf_task::Phase::ConstCompletedPtr completed_phase);

  /// Callback for when the task has finished
  void _task_finished();

  // TODO: Assuming each LegacyTask::Active instance stores a weak_ptr to this
  // TaskManager, the implementations of the corresponding functions in
  // LegacyTask::Active can call these methods to publish state/log updates
  // void _interrupt_active_task();
  // void _cancel_active_task();
  // void _kill_active_task();
  // void _skip_phase_in_active_task(uint64_t phase_id, bool value = true);
  // void _rewind_active_task(uint64_t phase_id);

  /// Function to register the task id of a task that has begun execution
  /// The input task id will be inserted into the registry such that the max
  /// size of the registry is 100.
  void _register_executed_task(const std::string& id);

  void _populate_task_summary(
    std::shared_ptr<LegacyTask> task,
    uint32_t task_summary_state,
    TaskSummaryMsg& msg);
};

using TaskManagerPtr = std::shared_ptr<TaskManager>;

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__TASKMANAGER_HPP
