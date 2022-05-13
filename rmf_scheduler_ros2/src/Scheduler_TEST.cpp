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

#include "test/utils.hpp"
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

using TestScheduler = Scheduler<VirtualExecutor, MockPublisherFactory>;

class SchedulerFixture
{
public:
  MockPublisher mock_publisher;
  VirtualExecutor executor;
  SqliteDataSource store{db_file};
  TestScheduler scheduler;

  SchedulerFixture()
  : scheduler{executor, store, MockPublisherFactory{mock_publisher}} {}
};

TEST_CASE_METHOD(SchedulerFixture, "Schedule simple", "[Schedule]") {
  auto& executor = scheduler.executor;
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
    auto s = store.fetch_schedule(schedule.name).value();
    CHECK(s.name == schedule.name);
    CHECK(s.start_at == 20);
    CHECK(s.finish_at == 22);

    auto state = store.fetch_schedule_state(schedule.name).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::ScheduleState::CREATED);
    CHECK(state.last_modified == 10);
    CHECK(state.next_run == 21);
  }

  // check that schedule is ran
  executor.advance_until(21);
  REQUIRE(this->mock_publisher.publishes.size() == 1);
  CHECK(this->mock_publisher.publishes[0][0] == 1);

  // check that the schedule state is updated
  {
    auto state = store.fetch_schedule_state(schedule.name).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::ScheduleState::STARTED);
    CHECK(state.last_modified == 21);
    CHECK(state.last_ran == 21);
    CHECK(state.next_run == 22);
  }

  // should run again next sec.
  executor.advance_until(22);
  REQUIRE(this->mock_publisher.publishes.size() == 2);
  CHECK(this->mock_publisher.publishes[1][0] == 1);

  // state should now be finished
  {
    auto state = store.fetch_schedule_state(schedule.name).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::ScheduleState::FINISHED);
    CHECK(state.last_modified == 22);
    CHECK(state.last_ran == 22);
    CHECK(state.next_run == 0);
  }
}

TEST_CASE_METHOD(SchedulerFixture, "Schedule already started", "[Schedule]")
{
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
    auto state = store.fetch_schedule_state(schedule.name).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::ScheduleState::STARTED);
    CHECK(state.next_run == 11);
  }

  scheduler.executor.advance_until(11);

  {
    CHECK(this->mock_publisher.publishes.size() == 1);

    auto state = store.fetch_schedule_state(schedule.name).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::ScheduleState::FINISHED);
    CHECK(state.next_run == 0);
  }
}

TEST_CASE_METHOD(SchedulerFixture, "Schedule already finished", "[Schedule]")
{
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
    auto state = store.fetch_schedule_state(schedule.name).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::ScheduleState::FINISHED);
    CHECK(state.next_run == 0);
  }

  scheduler.executor.advance_until(11);

  CHECK(this->mock_publisher.publishes.size() == 0);
}

TEST_CASE_METHOD(SchedulerFixture, "Duplicated schedules are replaced",
  "[Schedule]")
{
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

  auto schedule = store.fetch_schedule(name).value();
  CHECK(schedule.payload.data[0] == 2);

  // check that old schedule is not ran
  scheduler.executor.advance_until(1);
  REQUIRE(this->mock_publisher.publishes.size() == 1);
  CHECK(this->mock_publisher.publishes[0][0] == 2);
}

TEST_CASE_METHOD(SchedulerFixture, "Schedule error when finish >= start",
  "[Schedule]")
{
  rmf_scheduler_msgs::msg::Schedule schedule;
  schedule.name = unique_name();
  schedule.schedule = "* * * * * *"; // every sec
  schedule.start_at = 10;
  schedule.finish_at = 10;
  schedule.payload.type = 0;
  schedule.payload.data.push_back(1);

  REQUIRE_THROWS_AS(scheduler.create_schedule(schedule), std::logic_error);
}

TEST_CASE_METHOD(SchedulerFixture, "Cancel schedule", "[Schedule]")
{
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
  CHECK(this->mock_publisher.publishes.size() == 0);
  auto state = scheduler.store.fetch_schedule_state(name).value();
  CHECK(state.status == rmf_scheduler_msgs::msg::ScheduleState::CANCELLED);
  CHECK(state.next_run == 0);
}

TEST_CASE_METHOD(SchedulerFixture,
  "Cancelling non existent schedule should be noop",
  "[Schedule]")
{
  REQUIRE_NOTHROW(scheduler.cancel_schedule("hello"));
}

TEST_CASE_METHOD(SchedulerFixture, "Trigger simple", "[Trigger]") {
  auto& executor = scheduler.executor;
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
    auto t = store.fetch_trigger(trigger.name).value();
    CHECK(t.name == trigger.name);
    CHECK(t.at == 20);

    auto state = store.fetch_trigger_state(trigger.name).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::TriggerState::STARTED);
    CHECK(state.last_modified == 10);
  }

  // check that triggers are ran
  executor.advance_until(20);
  REQUIRE(this->mock_publisher.publishes.size() == 1);
  CHECK(this->mock_publisher.publishes[0][0] == 1);

  // check that the trigger state is updated
  {
    auto state = store.fetch_trigger_state(trigger.name).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::TriggerState::FINISHED);
    CHECK(state.last_modified == 20);
    CHECK(state.last_ran == 20);
  }
}

TEST_CASE_METHOD(SchedulerFixture, "Duplicated triggers are replaced",
  "[Trigger]")
{
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

  auto trigger = store.fetch_trigger(name).value();
  CHECK(trigger.at == 20);
  CHECK(trigger.payload.data[0] == 2);

  // check that old trigger is not ran
  scheduler.executor.advance_until(20);
  REQUIRE(this->mock_publisher.publishes.size() == 1);
  CHECK(this->mock_publisher.publishes[0][0] == 2);
}

TEST_CASE_METHOD(SchedulerFixture, "Late triggers are executed", "[Trigger]")
{
  scheduler.executor.advance_until(10);
  auto name = unique_name();

  rmf_scheduler_msgs::msg::Trigger trigger;
  trigger.name = name;
  trigger.at = 1;
  trigger.payload.type = 0;
  trigger.payload.data.push_back(1);
  scheduler.create_trigger(trigger);

  scheduler.executor.advance_until(11);
  CHECK(this->mock_publisher.publishes.size() == 1);
  auto state = scheduler.store.fetch_trigger_state(name).value();
  CHECK(state.status == rmf_scheduler_msgs::msg::TriggerState::FINISHED);
}

TEST_CASE_METHOD(SchedulerFixture, "Cancel trigger", "[Trigger]")
{
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
  CHECK(this->mock_publisher.publishes.size() == 0);
  auto state = scheduler.store.fetch_trigger_state(name).value();
  CHECK(state.status == rmf_scheduler_msgs::msg::TriggerState::CANCELLED);
}

TEST_CASE_METHOD(SchedulerFixture,
  "Cancelling non existent trigger should be noop", "[Trigger]")
{
  REQUIRE_NOTHROW(scheduler.cancel_trigger("hello"));
}

TEST_CASE_METHOD(SchedulerFixture, "Load from database")
{
  std::string db_file = "load_from_db.sqlite3";
  std::filesystem::remove(db_file);

  {
    MockPublisher mock_publisher;
    VirtualExecutor executor;
    SqliteDataSource store{db_file};
    TestScheduler scheduler{executor, store,
      MockPublisherFactory{mock_publisher}};

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
    MockPublisher mock_publisher;
    VirtualExecutor executor;
    SqliteDataSource store{db_file};
    auto loaded =
      TestScheduler{executor, store, MockPublisherFactory{mock_publisher},
      true};
    loaded.executor.advance_until(21);
    CHECK(mock_publisher.publishes.size() == 2);
  }
}

TEST_CASE_METHOD(SchedulerFixture, "cancel all")
{
  std::string trigger_group1 = unique_name();
  std::string trigger_group2 = unique_name();
  std::string schedule_group1 = unique_name();
  std::string schedule_group2 = unique_name();


  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = trigger_group1;
    trigger.group = "group1";
    trigger.payload.type = 0;
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);
  }

  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = trigger_group2;
    trigger.group = "group2";
    trigger.payload.type = 0;
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);
  }

  {
    rmf_scheduler_msgs::msg::Schedule schedule;
    schedule.name = schedule_group1;
    schedule.group = "group1";
    schedule.schedule = "* * * * * *"; // every sec
    schedule.payload.type = 0;
    schedule.payload.data.push_back(1);
    scheduler.create_schedule(schedule);
  }

  {
    rmf_scheduler_msgs::msg::Schedule schedule;
    schedule.name = schedule_group2;
    schedule.group = "group2";
    schedule.schedule = "* * * * * *"; // every sec
    schedule.payload.type = 0;
    schedule.payload.data.push_back(1);
    scheduler.create_schedule(schedule);
  }

  scheduler.cancel_all("group1");

  {
    auto state = scheduler.store.fetch_trigger_state(trigger_group1).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::TriggerState::CANCELLED);
  }
  {
    auto state = scheduler.store.fetch_trigger_state(trigger_group2).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::TriggerState::STARTED);
  }
  {
    auto state = scheduler.store.fetch_schedule_state(schedule_group1).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::ScheduleState::CANCELLED);
  }
  {
    auto state = scheduler.store.fetch_schedule_state(schedule_group2).value();
    CHECK(state.status == rmf_scheduler_msgs::msg::ScheduleState::STARTED);
  }
}

TEST_CASE("cancel all is atomic")
{
  class BadExecutor : public VirtualExecutor
  {
  public:
    size_t bad_count = 0;
    size_t good_count = 0;

    void cancel_task(std::shared_ptr<Task> task)
    {
      if (task->when == 20 && this->bad_count == 0)
      {
        ++this->bad_count;
        throw std::runtime_error("bad");
      }
      else
      {
        ++this->good_count;
        VirtualExecutor::cancel_task(task);
      }
    }
  };

  using TestScheduler = Scheduler<BadExecutor, MockPublisherFactory>;
  MockPublisher mock_publisher;
  BadExecutor executor;
  SqliteDataSource store{":memory:"};
  TestScheduler scheduler{executor, store,
    MockPublisherFactory{mock_publisher}};

  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = "trigger1";
    trigger.at = 10;
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);
  }

  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = "trigger2";
    trigger.at = 20;
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);
  }

  CHECK_THROWS(scheduler.cancel_all("default"));
  CHECK(executor.bad_count == 1);
  CHECK(executor.good_count > 0);
  executor.advance_until(20);
  CHECK(mock_publisher.publishes.size() == 2);
}

TEST_CASE_METHOD(SchedulerFixture,
  "publisher gets cleaned up after publishing") {
  auto& executor = scheduler.executor;

  int cleanup_count = 0;
  this->scheduler.on_publisher_cleanup = [&cleanup_count](auto)
    {
      ++cleanup_count;
    };

  rmf_scheduler_msgs::msg::Trigger trigger;
  trigger.name = unique_name();
  trigger.payload.data.push_back(1);
  scheduler.create_trigger(trigger);

  executor.advance_until(TestScheduler::kPublisherKeepAliveSecs - 1);
  CHECK(cleanup_count == 0);
  executor.advance_until(TestScheduler::kPublisherKeepAliveSecs);
  CHECK(cleanup_count == 1);
}

TEST_CASE_METHOD(SchedulerFixture,
  "using publisher again keeps it alive") {
  auto& executor = scheduler.executor;

  int cleanup_count = 0;
  this->scheduler.on_publisher_cleanup = [&cleanup_count](auto)
    {
      ++cleanup_count;
    };

  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = unique_name();
    trigger.at = 0;
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);
  }
  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = unique_name();
    trigger.at = 10;
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);
  }

  executor.advance_until(TestScheduler::kPublisherKeepAliveSecs);
  CHECK(cleanup_count == 0);
  executor.advance_until(TestScheduler::kPublisherKeepAliveSecs + 9);
  CHECK(cleanup_count == 0);
  executor.advance_until(TestScheduler::kPublisherKeepAliveSecs + 10);
  CHECK(cleanup_count == 1);
}

TEST_CASE_METHOD(SchedulerFixture,
  "publishers are tracked by payload topic") {
  auto& executor = scheduler.executor;

  int cleanup_count = 0;
  this->scheduler.on_publisher_cleanup = [&cleanup_count](auto)
    {
      ++cleanup_count;
    };

  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = unique_name();
    trigger.at = 0;
    trigger.payload.topic = "topic1";
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);
  }
  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = unique_name();
    trigger.at = 10;
    trigger.payload.topic = "topic2";
    trigger.payload.data.push_back(1);
    scheduler.create_trigger(trigger);
  }

  executor.advance_until(TestScheduler::kPublisherKeepAliveSecs - 1);
  CHECK(cleanup_count == 0);
  executor.advance_until(TestScheduler::kPublisherKeepAliveSecs);
  CHECK(cleanup_count == 1);
  executor.advance_until(TestScheduler::kPublisherKeepAliveSecs + 10);
  CHECK(cleanup_count == 2);
}

}

int main(int argc, char* argv[])
{
  std::filesystem::remove(rmf::scheduler::test::db_file);
  return Catch::Session().run(argc, argv);
}
