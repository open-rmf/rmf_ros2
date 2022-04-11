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

#include "Task.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>

namespace rmf::scheduler {

/// Executor that uses the system clock to schedule tasks.
///
/// Unless specificed otherwise, all methods in this class are thread-safe.
class SystemTimeExecutor
{
public:
  int64_t now() const;

  std::shared_ptr<Task> schedule_task(int64_t at, std::function<void()> action);

  void clear_tasks();

  void cancel_task(std::shared_ptr<Task> task);

  /// Spins the executor until `stop` is called. Only one thread can spin the executor
  /// at once, calling this from multiple threads is undefined behavior.
  void spin();

  void stop();

private:
  std::multimap<int64_t, std::shared_ptr<Task>> _tasks;
  std::mutex _tasks_mutex;
  std::condition_variable _cv;
  std::atomic<bool> _spinning = false;

  void _tick();
};

}
