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

#include "TaskManager.hpp"

namespace rmf_fleet_adapter {

//==============================================================================
void TaskManager::queue_task(Task task, Start expected_finish)
{
  _queue.push_back(std::make_unique<Task>(std::move(task)));
  _expected_finish_location = std::move(expected_finish);

  if (!_active_task)
    _begin_next_task();
}

//==============================================================================
auto TaskManager::expected_finish_location() const -> StartSet
{
  if (_expected_finish_location)
    return {*_expected_finish_location};

  return _context->location();
}

//==============================================================================
void TaskManager::_begin_next_task()
{
  if (_queue.empty())
  {
    _task_sub.unsubscribe();
    _expected_finish_location = rmf_utils::nullopt;
    return;
  }

  _active_task = std::move(_queue.front());
  _queue.erase(_queue.begin());

  _task_sub = _active_task->observe()
      .observe_on(rxcpp::identity_same_worker(_context->worker()))
      .subscribe(
        [this, id = _active_task->id()](Task::StatusMsg msg)
  {
    msg.task_id = id;
    _context->node()->task_summary()->publish(msg);
  },
        [this, id = _active_task->id()](std::exception_ptr e)
  {
    rmf_task_msgs::msg::TaskSummary msg;
    msg.state = msg.STATE_FAILED;

    try {
      std::rethrow_exception(e);
    }
    catch(const std::exception& e) {
      msg.status = e.what();
    }

    msg.task_id = id;
    _context->node()->task_summary()->publish(msg);
    _begin_next_task();
  },
        [this]()
  {
    _begin_next_task();
  });

  _active_task->begin();
}

} // namespace rmf_fleet_adapter
