/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "SystemTimeExecutor.hpp"

#include <chrono>

namespace rmf::scheduler {

using Clock = std::chrono::system_clock;

int64_t SystemTimeExecutor::now() const
{
  return std::chrono::duration_cast<std::chrono::seconds>(
    Clock::now().time_since_epoch()).count();
}

std::shared_ptr<Task> SystemTimeExecutor::schedule_task(int64_t at,
  std::function<void()> action)
{
  std::unique_lock lk{this->_tasks_mutex};
  auto task = std::make_shared<Task>(at, action);
  this->_tasks.insert({at, task});
  this->_cv.notify_all();
  return task;
}

void SystemTimeExecutor::clear_tasks()
{
  std::unique_lock lk{this->_tasks_mutex};
  this->_tasks.clear();
  this->_cv.notify_all();
}

void SystemTimeExecutor::cancel_task(std::shared_ptr<Task> task)
{
  std::unique_lock lk{this->_tasks_mutex};
  for (auto it = this->_tasks.lower_bound(task->when);
    it != this->_tasks.upper_bound(task->when); ++it)
  {
    if (it->second == task)
    {
      this->_tasks.erase(it);
      break;
    }
  }
  this->_cv.notify_all();
}

void SystemTimeExecutor::spin()
{
  this->_spinning = true;
  while (this->_spinning)
  {
    this->_tick();
  }
}

void SystemTimeExecutor::stop()
{
  this->_spinning = false;
  this->_cv.notify_all();
}

void SystemTimeExecutor::_tick()
{
  // wait forever if there are no tasks.
  Clock::time_point next = Clock::time_point::max();
  {
    std::unique_lock lk{this->_tasks_mutex};
    if (!this->_tasks.empty())
    {
      next =
        Clock::time_point{std::chrono::seconds{this->_tasks.begin()->first}};
    }
  }

  std::unique_lock lk{this->_tasks_mutex};

  // don't need to check for spurious wakes since the loop will exit immediately
  // if it is not yet time to run a task.
  this->_cv.wait_until(lk, next);
  for (auto it = this->_tasks.begin(); it != this->_tasks.end();
    it = this->_tasks.erase(it))
  {
    auto task = it->second;
    if (task->when > this->now())
    {
      break;
    }
    task->action();
  }
}

}
