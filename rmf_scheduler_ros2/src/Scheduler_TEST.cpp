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

#define CATCH_CONFIG_RUNNER
#include <rmf_utils/catch.hpp>

#include "test/MockPublisher.hpp"
#include "test/VirtualExecutor.hpp"

#include "Scheduler.hpp"
#include "SqliteDataSource.hpp"

#include <rmf_scheduler_msgs/msg/schedule.hpp>
#include <rmf_scheduler_msgs/msg/trigger.hpp>

#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace rmf::scheduler::test {

static std::string db_file = "Scheduler_TEST.sqlite3";

static Scheduler<VirtualExecutor, MockPublisher> make_scheduler()
{
  static SqliteDataSource store{db_file};
  return Scheduler<VirtualExecutor, MockPublisher>{store};
}

static std::string unique_name()
{
  return "test_" + std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
}

TEST_CASE("Schedule simple", "[Scheduler]") {
  auto scheduler = make_scheduler();

  auto& executor = scheduler.executor;
  auto& publisher = scheduler.publisher;
  auto& store = scheduler.store;
  executor.advance_until(10);

  rmf_scheduler_msgs::msg::Schedule schedule;
  schedule.name = unique_name();
  schedule.schedule = "* * * * * *"; // every sec
  schedule.start_at = 20;
  schedule.finish_at = 22;
  schedule.payload.type = 0;
  schedule.payload.data.push_back(1);
  scheduler.create_schedule(schedule);

  // check that the schedule is created
  {
    auto s = store.fetch_schedule(schedule.name);
    REQUIRE(s.name == schedule.name);
    REQUIRE(s.start_at == 20);
    REQUIRE(s.finish_at == 22);

    auto state = store.fetch_schedule_state(schedule.name);
    REQUIRE(state.status == rmf_scheduler_msgs::msg::ScheduleState::CREATED);
    REQUIRE(state.last_modified == 10);
    REQUIRE(state.next_run == 21);
  }

  // check that schedule is ran
  executor.advance_until(21);
  REQUIRE(publisher.publishes.size() == 1);
  REQUIRE(publisher.publishes[0].data[0] == 1);

  // check that the schedule state is updated
  {
    auto state = store.fetch_schedule_state(schedule.name);
    REQUIRE(state.status == rmf_scheduler_msgs::msg::ScheduleState::STARTED);
    REQUIRE(state.last_modified == 21);
    REQUIRE(state.last_ran == 21);
    REQUIRE(state.next_run == 22);
  }

  // should run again next sec.
  executor.advance_until(22);
  REQUIRE(publisher.publishes.size() == 2);
  REQUIRE(publisher.publishes[1].data[0] == 1);

  // state should now be finished
  {
    auto state = store.fetch_schedule_state(schedule.name);
    REQUIRE(state.status == rmf_scheduler_msgs::msg::ScheduleState::FINISHED);
    REQUIRE(state.last_modified == 22);
    REQUIRE(state.last_ran == 22);
    REQUIRE(state.next_run == 0);
  }
}

TEST_CASE("Schedule already started", "[Scheduler]")
{
  auto scheduler = make_scheduler();
  auto& store = scheduler.store;
  scheduler.executor.advance_until(10);

  rmf_scheduler_msgs::msg::Schedule schedule;
  schedule.name = unique_name();
  schedule.schedule = "* * * * * *"; // every sec
  schedule.start_at = 0;
  schedule.finish_at = 11;
  schedule.payload.type = 0;
  schedule.payload.data.push_back(1);
  scheduler.create_schedule(schedule);

  {
    auto state = store.fetch_schedule_state(schedule.name);
    REQUIRE(state.status == rmf_scheduler_msgs::msg::ScheduleState::STARTED);
    REQUIRE(state.next_run == 11);
  }

  scheduler.executor.advance_until(11);

  {
    REQUIRE(scheduler.publisher.publishes.size() == 1);

    auto state = store.fetch_schedule_state(schedule.name);
    REQUIRE(state.status == rmf_scheduler_msgs::msg::ScheduleState::FINISHED);
    REQUIRE(state.next_run == 0);
  }
}

TEST_CASE("Schedule already finished", "[Scheduler]")
{
  auto scheduler = make_scheduler();
  auto& store = scheduler.store;
  scheduler.executor.advance_until(10);

  rmf_scheduler_msgs::msg::Schedule schedule;
  schedule.name = unique_name();
  schedule.schedule = "* * * * * *"; // every sec
  schedule.start_at = 0;
  schedule.finish_at = 10;
  schedule.payload.type = 0;
  schedule.payload.data.push_back(1);
  scheduler.create_schedule(schedule);

  {
    auto state = store.fetch_schedule_state(schedule.name);
    REQUIRE(state.status == rmf_scheduler_msgs::msg::ScheduleState::FINISHED);
    REQUIRE(state.next_run == 0);
  }

  scheduler.executor.advance_until(11);

  REQUIRE(scheduler.publisher.publishes.size() == 0);
}

TEST_CASE("Duplicated schedules are replaced", "[Scheduler]")
{
  auto scheduler = make_scheduler();
  auto& store = scheduler.store;

  auto name = unique_name();
  {
    rmf_scheduler_msgs::msg::Schedule schedule;
    schedule.name = name;
    schedule.schedule = "* * * * * *"; // every sec
    schedule.payload.type = 0;
    schedule.payload.data.push_back(1);
    scheduler.create_schedule(schedule);
  }

  {
    rmf_scheduler_msgs::msg::Schedule schedule;
    schedule.name = name;
    schedule.schedule = "* * * * * *"; // every sec
    schedule.payload.type = 0;
    schedule.payload.data.push_back(2);
    scheduler.create_schedule(schedule);
  }

  auto schedule = store.fetch_schedule(name);
  REQUIRE(schedule.payload.data[0] == 2);

  // check that old schedule is not ran
  scheduler.executor.advance_until(1);
  REQUIRE(scheduler.publisher.publishes.size() == 1);
  REQUIRE(scheduler.publisher.publishes[0].data[0] == 2);
}

TEST_CASE("Schedule error when finish >= start", "[Scheduler]")
{
  auto scheduler = make_scheduler();

  rmf_scheduler_msgs::msg::Schedule schedule;
  schedule.name = unique_name();
  schedule.schedule = "* * * * * *"; // every sec
  schedule.start_at = 10;
  schedule.finish_at = 10;
  schedule.payload.type = 0;
  schedule.payload.data.push_back(1);

  REQUIRE_THROWS_AS(scheduler.create_schedule(schedule), std::logic_error);
}

TEST_CASE("Cancel schedule", "[Scheduler]")
{
  auto scheduler = make_scheduler();
  scheduler.executor.advance_until(10);
  auto name = unique_name();

  rmf_scheduler_msgs::msg::Schedule schedule;
  schedule.name = name;
  schedule.schedule = "* * * * * *";
  schedule.start_at = 15;
  schedule.payload.type = 0;
  schedule.payload.data.push_back(1);
  scheduler.create_schedule(schedule);

  scheduler.cancel_schedule(name);
  scheduler.executor.advance_until(20);
  REQUIRE(scheduler.publisher.publishes.size() == 0);
  auto state = scheduler.store.fetch_schedule_state(name);
  REQUIRE(state.status == rmf_scheduler_msgs::msg::ScheduleState::CANCELLED);
  REQUIRE(state.next_run == 0);
}

TEST_CASE("Cancelling non existent schedule should be noop", "[Scheduler]")
{
  auto scheduler = make_scheduler();
  REQUIRE_NOTHROW(scheduler.cancel_schedule("hello"));
}

TEST_CASE("Trigger simple", "[Scheduler]") {
  auto scheduler = make_scheduler();

  auto& executor = scheduler.executor;
  auto& publisher = scheduler.publisher;
  auto& store = scheduler.store;
  executor.advance_until(10);

  rmf_scheduler_msgs::msg::Trigger trigger;
  trigger.name = unique_name();
  trigger.at = 20;
  trigger.payload.type = 0;
  trigger.payload.data.push_back(1);
  scheduler.create_trigger(trigger);

  // check that the trigger is created
  {
    auto t = store.fetch_trigger(trigger.name);
    REQUIRE(t.name == trigger.name);
    REQUIRE(t.at == 20);

    auto state = store.fetch_trigger_state(trigger.name);
    REQUIRE(state.status == rmf_scheduler_msgs::msg::TriggerState::STARTED);
    REQUIRE(state.last_modified == 10);
  }

  // check that triggers are ran
  executor.advance_until(20);
  REQUIRE(publisher.publishes.size() == 1);
  REQUIRE(publisher.publishes[0].data[0] == 1);

  // check that the trigger state is updated
  {
    auto state = store.fetch_trigger_state(trigger.name);
    REQUIRE(state.status == rmf_scheduler_msgs::msg::TriggerState::FINISHED);
    REQUIRE(state.last_modified == 20);
    REQUIRE(state.last_ran == 20);
  }
}

TEST_CASE("Duplicated triggers are replaced", "[Scheduler]")
{
  auto scheduler = make_scheduler();
  auto& store = scheduler.store;

  auto name = unique_name();
  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = name;
    trigger.at = 10;
    trigger.payload.type = 0;
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);
  }

  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = name;
    trigger.at = 20;
    trigger.payload.type = 0;
    trigger.payload.data.push_back(2);
    scheduler.create_trigger(trigger);
  }

  auto trigger = store.fetch_trigger(name);
  REQUIRE(trigger.at == 20);
  REQUIRE(trigger.payload.data[0] == 2);

  // check that old trigger is not ran
  scheduler.executor.advance_until(20);
  REQUIRE(scheduler.publisher.publishes.size() == 1);
  REQUIRE(scheduler.publisher.publishes[0].data[0] == 2);
}

TEST_CASE("Late triggers are executed", "[Scheduler]")
{
  auto scheduler = make_scheduler();
  scheduler.executor.advance_until(10);
  auto name = unique_name();

  rmf_scheduler_msgs::msg::Trigger trigger;
  trigger.name = name;
  trigger.at = 1;
  trigger.payload.type = 0;
  trigger.payload.data.push_back(1);
  scheduler.create_trigger(trigger);

  scheduler.executor.advance_until(11);
  REQUIRE(scheduler.publisher.publishes.size() == 1);
  auto state = scheduler.store.fetch_trigger_state(name);
  REQUIRE(state.status == rmf_scheduler_msgs::msg::TriggerState::FINISHED);
}

TEST_CASE("Cancel trigger", "[Scheduler]")
{
  auto scheduler = make_scheduler();
  scheduler.executor.advance_until(10);
  auto name = unique_name();

  rmf_scheduler_msgs::msg::Trigger trigger;
  trigger.name = name;
  trigger.at = 20;
  trigger.payload.type = 0;
  trigger.payload.data.push_back(1);
  scheduler.create_trigger(trigger);

  scheduler.cancel_trigger(name);
  scheduler.executor.advance_until(20);
  REQUIRE(scheduler.publisher.publishes.size() == 0);
  auto state = scheduler.store.fetch_trigger_state(name);
  REQUIRE(state.status == rmf_scheduler_msgs::msg::TriggerState::CANCELLED);
}

TEST_CASE("Cancelling non existent trigger should be noop", "[Scheduler]")
{
  auto scheduler = make_scheduler();
  REQUIRE_NOTHROW(scheduler.cancel_trigger("hello"));
}

TEST_CASE("Load from database", "[Scheduler]")
{
  using TestScheduler = Scheduler<VirtualExecutor, MockPublisher>;
  std::string db_file = "load_from_db.sqlite3";
  std::filesystem::remove(db_file);

  {
    SqliteDataSource store{db_file};
    TestScheduler scheduler{store};

    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = unique_name();
    trigger.at = 20;
    trigger.payload.type = 0;
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);

    rmf_scheduler_msgs::msg::Schedule schedule;
    schedule.name = unique_name();
    schedule.schedule = "* * * * * *"; // every sec
    schedule.start_at = 20;
    schedule.finish_at = 22;
    schedule.payload.type = 0;
    schedule.payload.data.push_back(1);
    scheduler.create_schedule(schedule);
  }

  {
    SqliteDataSource store{db_file};
    auto loaded = TestScheduler::load_from_db(store);
    loaded.executor.advance_until(21);
    REQUIRE(loaded.publisher.publishes.size() == 2);
  }
}

}

int main(int argc, char* argv[])
{
  std::filesystem::remove(rmf::scheduler::test::db_file);
  return Catch::Session().run(argc, argv);
}