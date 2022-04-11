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

#define CATCH_CONFIG_MAIN
#include <rmf_utils/catch.hpp>

#include "test/MockPublisher.hpp"
#include "test/VirtualExecutor.hpp"

#include "Scheduler.hpp"
#include "SqliteDataSource.hpp"

#include <rmf_scheduler_msgs/msg/serialized_message.hpp>
#include <rmf_scheduler_msgs/msg/trigger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace rmf::scheduler::test {

static Scheduler<VirtualExecutor, MockPublisher> make_scheduler()
{
  std::string db_file = Catch::getResultCapture().getCurrentTestName() +
    ".sqlite3";
  std::filesystem::remove(db_file);
  return Scheduler<VirtualExecutor, MockPublisher>{db_file};
}

TEST_CASE("Trigger") {
  auto scheduler = make_scheduler();

  auto& executor = scheduler.executor;
  auto& publisher = scheduler.publisher;
  auto& store = scheduler.store;
  executor.advance_until(10);

  rmf_scheduler_msgs::msg::Trigger trigger;
  trigger.name = "test_trigger";
  trigger.at = 20;
  trigger.payload.type = 0;
  trigger.payload.data.push_back(1);
  scheduler.schedule_trigger(trigger);

  // check that the trigger is created
  {
    auto trigger = store.fetch_trigger("test_trigger");
    REQUIRE(trigger.name == "test_trigger");
    REQUIRE(trigger.at == 20);

    auto state = store.fetch_trigger_state("test_trigger");
    REQUIRE(state.status == rmf_scheduler_msgs::msg::TriggerState::CREATED);
    REQUIRE(state.last_modified == 10);
  }

  // check that triggers are ran
  executor.advance_until(20);
  REQUIRE(publisher.publishes.size() == 1);
  REQUIRE(publisher.publishes[0].data[0] == 1);

  // check that the trigger state is updated
  {
    auto state = store.fetch_trigger_state("test_trigger");
    REQUIRE(state.status == rmf_scheduler_msgs::msg::TriggerState::FINISHED);
    REQUIRE(state.last_modified == 20);
    REQUIRE(state.last_ran == 20);
  }
}

TEST_CASE("Duplicated triggers are replaced")
{
  auto scheduler = make_scheduler();
  auto& store = scheduler.store;
  auto& publisher = scheduler.publisher;

  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = "test_trigger";
    trigger.at = 10;
    trigger.payload.type = 0;
    trigger.payload.data.push_back(1);
    scheduler.schedule_trigger(trigger);
  }

  {
    rmf_scheduler_msgs::msg::Trigger trigger;
    trigger.name = "test_trigger";
    trigger.at = 20;
    trigger.payload.type = 0;
    trigger.payload.data.push_back(2);
    scheduler.schedule_trigger(trigger);
  }

  auto trigger = store.fetch_trigger("test_trigger");
  REQUIRE(trigger.at == 20);
  REQUIRE(trigger.payload.data[0] == 2);

  // check that old trigger is not ran
  scheduler.executor.advance_until(20);
  REQUIRE(scheduler.publisher.publishes.size() == 1);
  REQUIRE(scheduler.publisher.publishes[0].data[0] == 2);
}

}
