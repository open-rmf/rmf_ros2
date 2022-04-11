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

#include "SqliteDataSource.hpp"
#include "Task.hpp"

#include <rmf_scheduler_msgs/msg/trigger.hpp>
#include <rmf_scheduler_msgs/msg/trigger_record.hpp>
#include <rmf_scheduler_msgs/msg/trigger_state.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>

namespace rmf::scheduler {

class Task;

template<typename Executor, typename Publisher>
class Scheduler
{
public:
  Executor executor;
  Publisher publisher;
  SqliteDataSource store;

  Scheduler(const std::string& db_file)
  : store(db_file)
  {
  }

  void schedule_trigger(const rmf_scheduler_msgs::msg::Trigger& trigger)
  {
    auto now = executor.now();

    rmf_scheduler_msgs::msg::TriggerRecord record;
    record.created_at = now;
    record.state.last_modified = now;
    record.trigger = trigger;
    record.state.last_ran = 0;
    record.state.status = rmf_scheduler_msgs::msg::TriggerState::CREATED;

    auto t = this->store.begin_transaction();
    t.create_trigger(record);
    this->store.commit_transaction();

    // cancels the previous trigger if it exists
    try
    {
      this->executor.cancel_task(this->_trigger_tasks.at(trigger.name));
      this->_trigger_tasks.erase(trigger.name);
    }
    catch (const std::out_of_range&)
    {
    }

    // capture only the name to avoid storing the payload in memory.
    this->_trigger_tasks[trigger.name] =
      executor.schedule_task(trigger.at, [this, name = trigger.name]()
        {
          auto trigger = this->store.fetch_trigger(name);
          this->publisher.publish(trigger.payload);
          rmf_scheduler_msgs::msg::TriggerState state;
          auto now = this->executor.now();
          state.last_ran = now;
          state.last_modified = now;
          state.status = rmf_scheduler_msgs::msg::TriggerState::FINISHED;
          auto t = this->store.begin_transaction();
          t.save_trigger_state(name, state);
          this->store.commit_transaction();
        });
  }

private:
  std::unordered_map<std::string, std::shared_ptr<Task>> _trigger_tasks;
};

}
