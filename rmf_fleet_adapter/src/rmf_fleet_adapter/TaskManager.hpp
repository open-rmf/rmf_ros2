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

#include "Task.hpp"
#include "agv/RobotContext.hpp"

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task/agv/TaskPlanner.hpp>

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

  static std::shared_ptr<TaskManager> make(agv::RobotContextPtr context);

  using Start = rmf_traffic::agv::Plan::Start;
  using StartSet = rmf_traffic::agv::Plan::StartSet;
  using Assignment = rmf_task::agv::TaskPlanner::Assignment;
  using State = rmf_task::agv::State;

  /// Add a task to the queue of this manager.
  void queue_task(std::shared_ptr<Task> task, Start expected_finish);

  /// The location where we expect this robot to be at the end of its current
  /// task queue.
  StartSet expected_finish_location() const;

  const agv::RobotContextPtr& context();

  agv::ConstRobotContextPtr context() const;

  const Task* current_task() const;

  /// Set the queue for this task manager with assignments generated from the
  /// task planner
  void set_queue(const std::vector<Assignment>& assignments);

  /// Get the non-charging requests among pending tasks
  const std::vector<rmf_task::ConstRequestPtr> requests() const;

  /// The state of the robot.
  State expected_finish_state() const;

  /// Callback for the retreat timer. Appends a charging task to the task queue
  /// when robot is idle and battery level drops below a retreat threshold.
  void retreat_to_charger();

private:

  TaskManager(agv::RobotContextPtr context);

  agv::RobotContextPtr _context;
  std::shared_ptr<Task> _active_task;
  std::vector<std::shared_ptr<Task>> _queue;
  rmf_utils::optional<Start> _expected_finish_location;
  rxcpp::subscription _task_sub;
  rxcpp::subscription _emergency_sub;

  // TODO: Eliminate the need for a mutex by redesigning the use of the task
  // manager so that modifications of shared data only happen on designated
  // rxcpp worker
  std::mutex _mutex;
  rclcpp::TimerBase::SharedPtr _task_timer;
  rclcpp::TimerBase::SharedPtr _retreat_timer;

  /// Callback for task timer which begins next task if its deployment time has passed
  void _begin_next_task();
};

using TaskManagerPtr = std::shared_ptr<TaskManager>;

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__TASKMANAGER_HPP
