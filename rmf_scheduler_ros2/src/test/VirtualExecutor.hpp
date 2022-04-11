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

#pragma once

#include "../Task.hpp"

#include <functional>
#include <map>
#include <memory>

namespace rmf::scheduler {

class VirtualExecutor
{
public:
  int64_t now() const;

  std::shared_ptr<Task> schedule_task(int64_t at, std::function<void()> action);

  void clear_tasks();

  void cancel_task(std::shared_ptr<Task> task);

  void advance_by(int64_t by);

  void advance_until(int64_t until);

private:
  std::multimap<int64_t, std::shared_ptr<Task>> _tasks;
  int64_t _now = 0;

  void _tick();
};

}
