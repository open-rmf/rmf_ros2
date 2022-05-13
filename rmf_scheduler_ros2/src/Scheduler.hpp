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

#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace rmf::scheduler {

class Task;

template<typename Executor, typename PublisherFactory>
class Scheduler
{
public:
  static constexpr int64_t kPublisherKeepAliveSecs = 30;

  Executor& executor;
  SqliteDataSource& store;
  PublisherFactory publisher_factory;
  rclcpp::Logger logger = rclcpp::get_logger("Scheduler");

  using TriggerUpdateCb =
    std::function<void(const rmf_scheduler_msgs::msg::TriggerState&)>;
  TriggerUpdateCb on_trigger_update;
  using ScheduleUpdateCb =
    std::function<void(const rmf_scheduler_msgs::msg::ScheduleState&)>;
  ScheduleUpdateCb on_schedule_update;
  using PublisherCleanupCb =
    std::function<void(const typename PublisherFactory::Publisher&)>;
  PublisherCleanupCb on_publisher_cleanup;

  /// Creates a new scheduler with no existing tasks.
  Scheduler(Executor& executor, SqliteDataSource& store,
    PublisherFactory pub_factory,
    bool load_from_db = false)
  : executor(executor), store(store), publisher_factory(pub_factory)
  {
    if (load_from_db)
    {
      for (const auto& trigger : store.fetch_active_triggers())
      {
        this->_schedule_trigger(trigger);
      }

      std::unordered_map<std::string,
        rmf_scheduler_msgs::msg::Schedule> schedules;
      for (auto schedule : store.fetch_active_schedules())
      {
        schedules[schedule.name] = schedule;
      }
      std::unordered_map<std::string,
        rmf_scheduler_msgs::msg::ScheduleState> states;
      for (auto state : store.fetch_active_schedule_states())
      {
        states[state.name] = state;
      }

      for (auto& [name, schedule] : schedules)
      {
        auto state = states[name];
        this->_schedule_schedule(schedule, state);
      }
    }
  }

  void create_trigger(rmf_scheduler_msgs::msg::Trigger& trigger)
  {
    auto now = executor.now();

    trigger.created_at = now;
    rmf_scheduler_msgs::msg::TriggerState state;
    state.name = trigger.name;
    state.last_modified = now;
    state.last_ran = 0;
    state.status = rmf_scheduler_msgs::msg::TriggerState::STARTED;

    this->store.create_trigger(trigger, state);

    this->_schedule_trigger(trigger);
  }

  void cancel_trigger(const std::string& trigger_name)
  {
    // cancels the previous trigger if it exists
    try
    {
      std::unique_lock lk{this->_tasks_mutex};
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
      this->store.save_trigger_state(state);
    }
    catch (const std::bad_optional_access&)
    {
      // ignore
    }
  }

  void create_schedule(rmf_scheduler_msgs::msg::Schedule& schedule)
  {
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

    this->store.create_schedule(schedule, state);

    this->_schedule_schedule(schedule, state);
  }

  void cancel_schedule(const std::string& schedule_name)
  {
    // cancels the previous trigger if it exists
    try
    {
      std::unique_lock lk{this->_tasks_mutex};
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
      this->store.save_schedule_state(state);
    }
    catch (const std::bad_optional_access&)
    {
      // ignore
    }
  }

  /// Cancel all triggers and schedules in group
  void cancel_all(const std::string& group)
  {
    try
    {
      auto t = this->store.begin_transaction();
      for (auto trigger : this->store.fetch_triggers_in_group(group))
      {
        this->cancel_trigger(trigger);
      }

      for (auto schedule : this->store.fetch_schedules_in_group(group))
      {
        this->cancel_schedule(schedule);
      }
      t.commit();
    }
    catch (const std::exception&)
    {
      for (auto name : this->store.fetch_triggers_in_group(group))
      {
        auto trigger = this->store.fetch_trigger(name);
        if (trigger)
        {
          this->_schedule_trigger(*trigger);
        }
        else
        {
          RCLCPP_ERROR(this->logger,
            "Unable to fetch trigger '%s' when trying to restore failed cancel",
            name.c_str());
        }
      }

      for (auto name : this->store.fetch_schedules_in_group(group))
      {
        auto schedule = this->store.fetch_schedule(name);
        auto state = this->store.fetch_schedule_state(name);
        if (schedule && state)
        {
          this->_schedule_schedule(*schedule, *state);
        }
        else
        {
          RCLCPP_ERROR(this->logger,
            "Unable to fetch schedule '%s' when trying to restore failed cancel",
            name.c_str());
        }
      }
      throw;
    }
  }

private:
  using _Me = Scheduler<Executor, PublisherFactory>;

  std::unordered_map<std::string, std::shared_ptr<Task>> _trigger_tasks;
  std::unordered_map<std::string, std::shared_ptr<Task>> _schedule_tasks;
  std::recursive_mutex _tasks_mutex;

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
    // cancels the previous trigger if it exists
    try
    {
      std::unique_lock lk{this->_tasks_mutex};
      this->executor.cancel_task(this->_trigger_tasks.at(trigger.name));
      this->_trigger_tasks.erase(trigger.name);
    }
    catch (const std::out_of_range&)
    {
      // ignore
    }

    // capture only the name to avoid storing the payload in memory.
    auto run = [this, name = trigger.name]()
      {
        try
        {
          {
            std::unique_lock lk{this->_tasks_mutex};
            this->_trigger_tasks.erase(name);
          }
          auto trigger = this->store.fetch_trigger(name).value();
          this->_get_publisher(trigger.payload).publish(
            trigger.payload.type,
            trigger.payload.data);
          rmf_scheduler_msgs::msg::TriggerState state;
          auto now = this->executor.now();
          state.name = name;
          state.last_ran = now;
          state.last_modified = now;
          state.status = rmf_scheduler_msgs::msg::TriggerState::FINISHED;
          this->store.save_trigger_state(state);

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

    {
      std::unique_lock lk{this->_tasks_mutex};
      this->_trigger_tasks[trigger.name] =
        this->executor.schedule_task(trigger.at, run);
    }
  }

  // functor to allow self referencing normally not allowd in lambdas.
  struct _run_schedule_task
  {
    using Me = Scheduler<Executor, PublisherFactory>;
    Me* scheduler;
    std::string name;

    _run_schedule_task(Me* scheduler, std::string schedule_name)
    : scheduler(scheduler), name(std::move(schedule_name))
    {}

    void operator()() const
    {
      try
      {
        {
          std::unique_lock lk{this->scheduler->_tasks_mutex};
          this->scheduler->_schedule_tasks.erase(name);
        }
        auto now = this->scheduler->executor.now();

        auto schedule = this->scheduler->store.fetch_schedule(name).value();
        this->scheduler->_get_publisher(schedule.payload).publish(
          schedule.payload.type,
          schedule.payload.data);

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

        this->scheduler->store.save_schedule_state(state);

        if (scheduler->on_schedule_update)
        {
          scheduler->on_schedule_update(state);
        }

        if (state.next_run != 0)
        {
          std::unique_lock lk{this->scheduler->_tasks_mutex};
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
    if (schedule.finish_at <= schedule.start_at)
    {
      throw std::logic_error("Finish time must be after start time");
    }

    // cancels the previous schedule if it exists
    try
    {
      std::unique_lock lk{this->_tasks_mutex};
      this->executor.cancel_task(this->_schedule_tasks.at(schedule.name));
      this->_schedule_tasks.erase(schedule.name);
    }
    catch (const std::out_of_range&)
    {
      // ignore
    }

    auto start_schedule_task = [this, name = schedule.name]()
      {
        try
        {
          {
            std::unique_lock lk{this->_tasks_mutex};
            this->_schedule_tasks.erase(name);
          }
          auto now = this->executor.now();
          auto state = this->store.fetch_schedule_state(name).value();
          state.status = rmf_scheduler_msgs::msg::ScheduleState::STARTED;
          state.last_modified = now;

          this->store.save_schedule_state(state);

          {
            std::unique_lock lk{this->_tasks_mutex};
            this->_schedule_tasks[name] =
              this->executor.schedule_task(state.next_run,
                _run_schedule_task{this,
                  name});
          }
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(this->logger, "Failed to start schedule '%s': %s",
            name.c_str(), e.what());
        }
      };

    {
      std::unique_lock lk{this->_tasks_mutex};
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
  }

  std::unordered_map<std::string,
    typename PublisherFactory::Publisher> _publishers;
  std::unordered_map<std::string,
    std::shared_ptr<Task>> _publisher_cleanup_tasks;

  typename PublisherFactory::Publisher& _get_publisher(
    const rmf_scheduler_msgs::msg::Payload& payload)
  {
    auto& topic = payload.topic;

    if (this->_publishers.count(topic) == 0)
    {
      this->_publishers.emplace(topic, this->publisher_factory(topic,
        payload.message_type));
    }

    // schedule the publisher to be cleaned up.
    try
    {
      // remove previous task
      this->executor.cancel_task(this->_publisher_cleanup_tasks.at(topic));
    }
    catch (const std::out_of_range&)
    {
      // no need to do anything
    }
    int64_t cleanup_time = this->executor.now() + kPublisherKeepAliveSecs;
    this->_publisher_cleanup_tasks.emplace(topic,
      this->executor.schedule_task(cleanup_time, [this, topic]()
      {
        this->_publisher_cleanup_tasks.erase(topic);
        if (this->on_publisher_cleanup)
        {
          this->on_publisher_cleanup(this->_publishers.at(topic));
        }
        this->_publishers.erase(topic);
      }));

    return this->_publishers.at(topic);
  }
};

}
