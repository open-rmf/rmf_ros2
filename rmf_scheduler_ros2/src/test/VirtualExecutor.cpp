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

#include "VirtualExecutor.hpp"

namespace rmf::scheduler {

int64_t VirtualExecutor::now() const
{
  return this->_now;
}

std::shared_ptr<Task> VirtualExecutor::schedule_task(int64_t at,
  std::function<void()> action)
{
  auto task = std::make_shared<Task>(at, action);
  this->_tasks.insert({at, task});
  return task;
}

void VirtualExecutor::clear_tasks()
{
  this->_tasks.clear();
}

void VirtualExecutor::cancel_task(std::shared_ptr<Task> task)
{
  for (auto it = this->_tasks.lower_bound(task->when);
    it != this->_tasks.upper_bound(task->when); ++it)
  {
    if (it->second == task)
    {
      this->_tasks.erase(it);
      break;
    }
  }
}

void VirtualExecutor::advance_by(int64_t by)
{
  this->advance_until(this->_now + by);
}

void VirtualExecutor::advance_until(int64_t until)
{
  for (auto it = this->_tasks.begin(); it != this->_tasks.end();
    it = this->_tasks.erase(it))
  {
    auto task = it->second;
    if (task->when > until)
    {
      break;
    }
    this->_now = task->when;
    task->action();
  }
  this->_now = until;
}

}
