/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef SRC__FULL_CONTROL__TASK_HPP
#define SRC__FULL_CONTROL__TASK_HPP

#include "ScheduleManager.hpp"

#include <rclcpp/time.hpp>

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
class Task
{
public:

  Task(FleetAdapterNode* node) : schedule(node, [&](){ this->resolve(); }) { }

  virtual void next() = 0;

  virtual void interrupt() = 0;

  virtual void resume() = 0;

  virtual void resolve() = 0;

  virtual void report_status() = 0;

  virtual void critical_failure(const std::string& error) = 0;

  virtual const std::string& id() const = 0;

  virtual const rclcpp::Time& start_time() const = 0;

  virtual ~Task() = default;

  ScheduleManager schedule;
};

} // namespace full_control
} // namespace rmf_fleet_adapter

#endif // SRC__FULL_CONTROL__TASK_HPP
