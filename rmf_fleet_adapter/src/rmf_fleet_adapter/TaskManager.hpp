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

namespace rmf_fleet_adapter {

//==============================================================================
/// This task manager is a first attempt at managing multiple tasks per fleet.
/// This is a simple implementation that only makes a modest attempt at being
/// optimal. A better task manager would queue tasks across the whole fleet
/// instead of queuing tasks for each robot individually. We will attempt that
/// in a later implementation.
class TaskManager
{
public:

  TaskManager(agv::RobotContextPtr context);

  using Start = rmf_traffic::agv::Plan::Start;
  using StartSet = rmf_traffic::agv::Plan::StartSet;

  /// Add a task to the queue of this manager.
  void queue_task(Task task, Start expected_finish);

  /// The location where we expect this robot to be at the end of its current
  /// task queue.
  StartSet expected_finish_location() const;

private:
  agv::RobotContextPtr _context;
  std::unique_ptr<Task> _active_task;
  std::vector<std::unique_ptr<Task>> _queue;
  rmf_utils::optional<Start> _expected_finish_location;
  rxcpp::subscription _task_sub;

  void _begin_next_task();
};

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__TASKMANAGER_HPP
