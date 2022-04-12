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

#include <sqlite3.h>

#include <croncpp.h>

#include <rmf_scheduler_msgs/msg/schedule.hpp>
#include <rmf_scheduler_msgs/msg/schedule_record.hpp>
#include <rmf_scheduler_msgs/msg/schedule_state.hpp>
#include <rmf_scheduler_msgs/msg/trigger.hpp>
#include <rmf_scheduler_msgs/msg/trigger_record.hpp>
#include <rmf_scheduler_msgs/msg/trigger_state.hpp>

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace rmf::scheduler {

class Task;

template<typename Executor, typename Publisher>
class Scheduler
{
public:
  /// Creates a scheduler by loading existing tasks from the database.
  static Scheduler<Executor, Publisher> load_from_db(SqliteDataSource& store)
  {
    Scheduler<Executor, Publisher> inst{store};

    for (const auto& name : store.fetch_running_triggers())
    {
      auto trigger = store.fetch_trigger(name);
      inst._schedule_trigger(trigger);
    }

    for (const auto& name : store.fetch_running_schedules())
    {
      auto schedule = store.fetch_schedule(name);
      auto state = store.fetch_schedule_state(name);
      inst._schedule_schedule(schedule, state);
    }

    return inst;
  }

  Executor executor;
  Publisher publisher;
  SqliteDataSource& store;

  /// Creates a new scheduler with no existing tasks.
  Scheduler(SqliteDataSource& store)
  : store(store)
  {
  }

  void create_trigger(const rmf_scheduler_msgs::msg::Trigger& trigger)
  {
    // cancels the previous trigger if it exists
    try
    {
      this->executor.cancel_task(this->_trigger_tasks.at(trigger.name));
      this->_trigger_tasks.erase(trigger.name);
    }
    catch (const std::out_of_range&)
    {
      // ignore
    }
    // no need to update state since we are overwrite with a new state anyway.

    auto now = executor.now();

    rmf_scheduler_msgs::msg::TriggerRecord record;
    record.created_at = now;
    record.trigger = trigger;
    record.state.last_modified = now;
    record.state.last_ran = 0;
    record.state.status = rmf_scheduler_msgs::msg::TriggerState::STARTED;

    auto t = this->store.begin_transaction();
    t.create_trigger(record);
    this->store.commit_transaction();

    this->_schedule_trigger(trigger);
  }

  void cancel_trigger(const std::string& trigger_name)
  {
    // cancels the previous trigger if it exists
    try
    {
      this->executor.cancel_task(this->_trigger_tasks.at(trigger_name));
      this->_trigger_tasks.erase(trigger_name);
    }
    catch (const std::out_of_range&)
    {
      // ignore
    }

    try
    {
      auto state = this->store.fetch_trigger_state(trigger_name);
      state.last_modified = this->executor.now();
      state.status = rmf_scheduler_msgs::msg::TriggerState::CANCELLED;
      auto t = this->store.begin_transaction();
      t.save_trigger_state(trigger_name, state);
      this->store.commit_transaction();
    }
    catch (const SqliteDataSource::DatabaseError& e)
    {
      if (e.code != SQLITE_DONE)
      {
        throw;
      }
    }
  }

  void create_schedule(const rmf_scheduler_msgs::msg::Schedule& schedule)
  {
    if (schedule.finish_at <= schedule.start_at)
    {
      throw std::logic_error("Finish time must be after start time");
    }

    // cancels the previous schedule if it exists
    try
    {
      this->executor.cancel_task(this->_schedule_tasks.at(schedule.name));
      this->_schedule_tasks.erase(schedule.name);
    }
    catch (const std::out_of_range&)
    {
      // ignore
    }

    auto now = this->executor.now();
    cron::cron_next(cron::make_cron(schedule.schedule), schedule.start_at);

    rmf_scheduler_msgs::msg::ScheduleRecord record;
    record.created_at = now;
    record.state.last_modified = now;
    record.state.last_ran = 0;
    record.state.next_run = this->_next_schedule_run(schedule, now);
    uint8_t status = rmf_scheduler_msgs::msg::ScheduleState::CREATED;
    if (schedule.start_at <= now)
    {
      status = rmf_scheduler_msgs::msg::ScheduleState::STARTED;
    }
    if (record.state.next_run == 0)
    {
      status = rmf_scheduler_msgs::msg::ScheduleState::FINISHED;
    }
    record.state.status = status;
    record.schedule = schedule;

    auto t = this->store.begin_transaction();
    t.create_schedule(record);
    this->store.commit_transaction();

    this->_schedule_schedule(schedule, record.state);
  }

  void cancel_schedule(const std::string& schedule_name)
  {
    // cancels the previous trigger if it exists
    try
    {
      this->executor.cancel_task(this->_schedule_tasks.at(schedule_name));
      this->_schedule_tasks.erase(schedule_name);
    }
    catch (const std::out_of_range&)
    {
      // ignore
    }

    try
    {
      auto state = this->store.fetch_schedule_state(schedule_name);
      state.last_modified = this->executor.now();
      state.next_run = 0;
      state.status = rmf_scheduler_msgs::msg::ScheduleState::CANCELLED;
      auto t = this->store.begin_transaction();
      t.save_schedule_state(schedule_name, state);
      this->store.commit_transaction();
    }
    catch (const SqliteDataSource::DatabaseError& e)
    {
      if (e.code != SQLITE_DONE)
      {
        throw;
      }
    }
  }

private:
  std::unordered_map<std::string, std::shared_ptr<Task>> _trigger_tasks;
  std::unordered_map<std::string, std::shared_ptr<Task>> _schedule_tasks;

  int64_t _next_schedule_run(
    const rmf_scheduler_msgs::msg::Schedule& schedule,
    int64_t now)
  {
    auto next_run = schedule.start_at > now ?
      cron::cron_next(cron::make_cron(schedule.schedule), schedule.start_at) :
      cron::cron_next(cron::make_cron(schedule.schedule), now);
    if (schedule.finish_at < next_run)
    {
      return 0;
    }
    return next_run;
  }

  void _schedule_trigger(const rmf_scheduler_msgs::msg::Trigger& trigger)
  {
    // capture only the name to avoid storing the payload in memory.
    this->_trigger_tasks[trigger.name] =
      this->executor.schedule_task(trigger.at, [this, name = trigger.name]()
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

  // functor to allow self referencing normally not allowd in lambdas.
  struct _run_schedule_task
  {
    using Me = Scheduler<Executor, Publisher>;
    Me* scheduler;
    std::string name;

    _run_schedule_task(Me* scheduler, std::string schedule_name)
    : scheduler(scheduler), name(std::move(schedule_name))
    {}

    void operator()() const
    {
      auto now = this->scheduler->executor.now();

      auto schedule = this->scheduler->store.fetch_schedule(name);
      this->scheduler->publisher.publish(schedule.payload);

      rmf_scheduler_msgs::msg::ScheduleState state;
      state.last_modified = now;
      state.last_ran = now;
      state.next_run = this->scheduler->_next_schedule_run(schedule, now);
      if (state.next_run == 0)
      {
        state.status = rmf_scheduler_msgs::msg::ScheduleState::FINISHED;
      }
      else
      {
        state.status = rmf_scheduler_msgs::msg::ScheduleState::STARTED;
      }

      auto t = this->scheduler->store.begin_transaction();
      t.save_schedule_state(name, state);
      this->scheduler->store.commit_transaction();

      if (state.next_run != 0)
      {
        this->scheduler->_schedule_tasks[name] =
          this->scheduler->executor.schedule_task(state.next_run,
            _run_schedule_task{this->scheduler, name});
      }
    }
  };

  void _schedule_schedule(const rmf_scheduler_msgs::msg::Schedule& schedule,
    const rmf_scheduler_msgs::msg::ScheduleState& state)
  {
    static auto start_schedule_task = [this, name = schedule.name]()
      {
        auto now = this->executor.now();
        auto state = this->store.fetch_schedule_state(name);
        state.status = rmf_scheduler_msgs::msg::ScheduleState::STARTED;
        state.last_modified = now;

        auto t = this->store.begin_transaction();
        t.save_schedule_state(name, state);
        this->store.commit_transaction();

        this->_schedule_tasks[name] =
          this->executor.schedule_task(state.next_run, _run_schedule_task{this,
              name});
      };

    switch (state.status)
    {
      case rmf_scheduler_msgs::msg::ScheduleState::CREATED:
        this->_schedule_tasks[schedule.name] =
          this->executor.schedule_task(schedule.start_at,
            start_schedule_task);
        break;
      case rmf_scheduler_msgs::msg::ScheduleState::STARTED:
        this->_schedule_tasks[schedule.name] =
          this->executor.schedule_task(state.next_run,
            _run_schedule_task{this, schedule.name});
    }
  }
};

}
