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

#include <croncpp.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rmf_scheduler_msgs/msg/schedule.hpp>
#include <rmf_scheduler_msgs/msg/schedule_state.hpp>
#include <rmf_scheduler_msgs/msg/trigger.hpp>
#include <rmf_scheduler_msgs/msg/trigger_state.hpp>

#include <sqlite3.h>

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
  static Scheduler<Executor, Publisher> load_from_db(
    Executor& executor,
    SqliteDataSource& store,
    Publisher& pub)
  {
    Scheduler<Executor, Publisher> inst{executor, store, pub};

    for (const auto& trigger : store.fetch_active_triggers())
    {
      inst._schedule_trigger(trigger);
    }

    for (const auto& schedule : store.fetch_active_schedules())
    {
      auto state = store.fetch_schedule_state(schedule.name).value();
      inst._schedule_schedule(schedule, state);
    }

    return inst;
  }

  Executor& executor;
  SqliteDataSource& store;
  Publisher& publisher;
  rclcpp::Logger logger = rclcpp::get_logger("Scheduler");

  using TriggerUpdateCb =
    std::function<void(const rmf_scheduler_msgs::msg::TriggerState&)>;
  TriggerUpdateCb on_trigger_update;
  using ScheduleUpdateCb =
    std::function<void(const rmf_scheduler_msgs::msg::ScheduleState&)>;
  ScheduleUpdateCb on_schedule_update;

  /// Creates a new scheduler with no existing tasks.
  Scheduler(Executor& executor, SqliteDataSource& store, Publisher& pub)
  : executor(executor), store(store), publisher(pub)
  {
  }

  void create_trigger(rmf_scheduler_msgs::msg::Trigger& trigger)
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

    trigger.created_at = now;
    rmf_scheduler_msgs::msg::TriggerState state;
    state.name = trigger.name;
    state.last_modified = now;
    state.last_ran = 0;
    state.status = rmf_scheduler_msgs::msg::TriggerState::STARTED;

    auto t = this->store.begin_transaction();
    t.create_trigger(trigger, state);
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
      auto state = this->store.fetch_trigger_state(trigger_name).value();
      state.last_modified = this->executor.now();
      state.status = rmf_scheduler_msgs::msg::TriggerState::CANCELLED;
      auto t = this->store.begin_transaction();
      t.save_trigger_state(state);
      this->store.commit_transaction();
    }
    catch (const std::bad_optional_access&)
    {
      // ignore
    }
  }

  void create_schedule(rmf_scheduler_msgs::msg::Schedule& schedule)
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

    schedule.created_at = now;
    rmf_scheduler_msgs::msg::ScheduleState state;
    state.name = schedule.name;
    state.last_modified = now;
    state.last_ran = 0;
    state.next_run = this->_next_schedule_run(schedule, now);
    uint8_t status = rmf_scheduler_msgs::msg::ScheduleState::CREATED;
    if (schedule.start_at <= now)
    {
      status = rmf_scheduler_msgs::msg::ScheduleState::STARTED;
    }
    if (state.next_run == 0)
    {
      status = rmf_scheduler_msgs::msg::ScheduleState::FINISHED;
    }
    state.status = status;

    auto t = this->store.begin_transaction();
    t.create_schedule(schedule, state);
    this->store.commit_transaction();

    this->_schedule_schedule(schedule, state);
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
      auto state = this->store.fetch_schedule_state(schedule_name).value();
      state.last_modified = this->executor.now();
      state.next_run = 0;
      state.status = rmf_scheduler_msgs::msg::ScheduleState::CANCELLED;
      auto t = this->store.begin_transaction();
      t.save_schedule_state(state);
      this->store.commit_transaction();
    }
    catch (const std::bad_optional_access&)
    {
      // ignore
    }
  }

  /// Cancel all triggers and schedules in group
  void cancel_all(const std::string& group)
  {
    for (auto trigger : this->store.fetch_triggers_in_group(group))
    {
      this->cancel_trigger(trigger);
    }

    for (auto schedule : this->store.fetch_schedules_in_group(group))
    {
      this->cancel_schedule(schedule);
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
    auto run = [this, name = trigger.name]()
      {
        try
        {
          auto trigger = this->store.fetch_trigger(name).value();
          this->publisher.publish(trigger.payload);
          rmf_scheduler_msgs::msg::TriggerState state;
          auto now = this->executor.now();
          state.name = name;
          state.last_ran = now;
          state.last_modified = now;
          state.status = rmf_scheduler_msgs::msg::TriggerState::FINISHED;
          auto t = this->store.begin_transaction();
          t.save_trigger_state(state);
          this->store.commit_transaction();

          if (this->on_trigger_update)
          {
            this->on_trigger_update(state);
          }
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(this->logger, "Failed to run trigger '%s': %s",
            name.c_str(), e.what());
        }
      };

    this->_trigger_tasks[trigger.name] =
      this->executor.schedule_task(trigger.at, run);
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
      try
      {
        auto now = this->scheduler->executor.now();

        auto schedule = this->scheduler->store.fetch_schedule(name).value();
        this->scheduler->publisher.publish(schedule.payload);

        rmf_scheduler_msgs::msg::ScheduleState state;
        state.name = this->name;
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
        t.save_schedule_state(state);
        this->scheduler->store.commit_transaction();

        if (scheduler->on_schedule_update)
        {
          scheduler->on_schedule_update(state);
        }

        if (state.next_run != 0)
        {
          this->scheduler->_schedule_tasks[name] =
            this->scheduler->executor.schedule_task(state.next_run,
              _run_schedule_task{this->scheduler, name});
        }
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(this->scheduler->logger, "Failed to run schedule '%s': %s",
          this->name.c_str(), e.what());
      }
    }
  };

  void _schedule_schedule(const rmf_scheduler_msgs::msg::Schedule& schedule,
    const rmf_scheduler_msgs::msg::ScheduleState& state)
  {
    auto start_schedule_task = [this, name = schedule.name]()
      {
        try
        {
          auto now = this->executor.now();
          auto state = this->store.fetch_schedule_state(name).value();
          state.status = rmf_scheduler_msgs::msg::ScheduleState::STARTED;
          state.last_modified = now;

          auto t = this->store.begin_transaction();
          t.save_schedule_state(state);
          this->store.commit_transaction();

          this->_schedule_tasks[name] =
            this->executor.schedule_task(state.next_run,
              _run_schedule_task{this,
                name});
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(this->logger, "Failed to start schedule '%s': %s",
            name.c_str(), e.what());
        }
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
        break;
    }
  }
};

}
