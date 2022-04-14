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

/**
 * Most of these are simple smoke tests because most tests are done at the server, for
 * better or worse.
 */

#define CATCH_CONFIG_RUNNER
#include <rmf_utils/catch.hpp>

#include <SchedulePayload.hpp>

#include <rmf_scheduler_msgs/srv/cancel_schedule.hpp>
#include <rmf_scheduler_msgs/srv/cancel_trigger.hpp>
#include <rmf_scheduler_msgs/srv/create_schedule.hpp>
#include <rmf_scheduler_msgs/srv/create_trigger.hpp>
#include <rmf_scheduler_msgs/srv/list_schedules.hpp>
#include <rmf_scheduler_msgs/srv/list_schedule_states.hpp>
#include <rmf_scheduler_msgs/srv/list_triggers.hpp>
#include <rmf_scheduler_msgs/srv/list_trigger_states.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <boost/process.hpp>

#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <signal.h>
#include <sys/types.h>
#include <thread>

#include <cstdlib>
#include <optional>
#include <iostream>

namespace bp = boost::process;
using namespace std::chrono_literals;

namespace rmf::scheduler::test {

static bp::group processes;
static rclcpp::Node::SharedPtr node;

static std::string unique_name()
{
  return "test_" + std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
}

template<typename RespFut>
void check_response(RespFut&& resp_fut)
{
  if (rclcpp::spin_until_future_complete(node, resp_fut,
    1s) == rclcpp::FutureReturnCode::TIMEOUT)
  {
    FAIL("Timed out waiting for response");
  }
  auto resp = resp_fut.get();
  if (!resp->success)
  {
    FAIL(resp->message);
  }
}

TEST_CASE("create trigger")
{
  auto client = node->create_client<rmf_scheduler_msgs::srv::CreateTrigger>(
    "create_trigger");
  if (!client->wait_for_service(5s))
  {
    FAIL("Timed out waiting for service");
  }

  std::promise<void> got_trigger;
  auto got_trigger_fut = got_trigger.get_future();
  auto new_trigger_sub =
    node->create_subscription<rmf_scheduler_msgs::msg::Trigger>(
    "new_trigger",
    rclcpp::SystemDefaultsQoS{},
    [&got_trigger](rmf_scheduler_msgs::msg::Trigger::SharedPtr)
    {
      got_trigger.set_value();
    });

  std::promise<void> got_state;
  auto got_state_fut = got_state.get_future();
  auto state_sub =
    node->create_subscription<rmf_scheduler_msgs::msg::TriggerState>(
    "trigger_update",
    rclcpp::SystemDefaultsQoS{},
    [&got_state](rmf_scheduler_msgs::msg::TriggerState::SharedPtr)
    {
      got_state.set_value();
    });

  std_msgs::msg::String msg;
  msg.data = "hello world";

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::CreateTrigger::Request>();
  req->trigger.name = unique_name();
  req->trigger.at = 0;
  req->trigger.payload = make_serialized_message("std_msgs/String",
      "test_topic", msg);

  check_response(client->async_send_request(req));

  if (rclcpp::spin_until_future_complete(node, got_trigger_fut,
    1s) == rclcpp::FutureReturnCode::TIMEOUT)
  {
    FAIL("Timed out waiting for new trigger to be published");
  }

  if (rclcpp::spin_until_future_complete(node, got_state_fut,
    1s) == rclcpp::FutureReturnCode::TIMEOUT)
  {
    FAIL("Timed out waiting for new trigger to be executed");
  }
}

TEST_CASE("cancel_trigger")
{
  auto client = node->create_client<rmf_scheduler_msgs::srv::CancelTrigger>(
    "cancel_trigger");
  if (!client->wait_for_service(5s))
  {
    FAIL("Timed out waiting for service");
  }

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::CancelTrigger::Request>();
  check_response(client->async_send_request(req));
}

TEST_CASE("list triggers")
{
  auto client = node->create_client<rmf_scheduler_msgs::srv::ListTriggers>(
    "list_triggers");
  if (!client->wait_for_service(5s))
  {
    FAIL("Timed out waiting for service");
  }

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::ListTriggers::Request>();
  req->created_after = 10;

  check_response(client->async_send_request(req));
}

TEST_CASE("list trigger states")
{
  auto client =
    node->create_client<rmf_scheduler_msgs::srv::ListTriggerStates>(
    "list_trigger_states");
  if (!client->wait_for_service(5s))
  {
    FAIL("Timed out waiting for service");
  }

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::ListTriggerStates::Request>();
  req->modified_after = 10;

  check_response(client->async_send_request(req));
}

TEST_CASE("create schedule")
{
  auto client = node->create_client<rmf_scheduler_msgs::srv::CreateSchedule>(
    "create_schedule");
  if (!client->wait_for_service(5s))
  {
    FAIL("Timed out waiting for service");
  }

  std::promise<void> got_schedule;
  auto got_schedule_fut = got_schedule.get_future();
  auto new_schedule_sub =
    node->create_subscription<rmf_scheduler_msgs::msg::Schedule>(
    "new_schedule",
    rclcpp::SystemDefaultsQoS{},
    [&got_schedule](rmf_scheduler_msgs::msg::Schedule::SharedPtr)
    {
      got_schedule.set_value();
    });

  std::promise<void> got_state;
  auto got_state_fut = got_state.get_future();
  auto state_sub =
    node->create_subscription<rmf_scheduler_msgs::msg::ScheduleState>(
    "schedule_update",
    rclcpp::SystemDefaultsQoS{},
    [&got_state](rmf_scheduler_msgs::msg::ScheduleState::SharedPtr)
    {
      got_state.set_value();
    });

  std_msgs::msg::String msg;
  msg.data = "hello world";

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::CreateSchedule::Request>();
  req->schedule.name = unique_name();
  req->schedule.schedule = "* * * * * *";
  req->schedule.payload = make_serialized_message("std_msgs/String",
      "test_topic", msg);

  check_response(client->async_send_request(req));

  if (rclcpp::spin_until_future_complete(node, got_schedule_fut,
    1s) == rclcpp::FutureReturnCode::TIMEOUT)
  {
    FAIL("Timed out waiting for new schedule to be published");
  }

  if (rclcpp::spin_until_future_complete(node, got_state_fut,
    3s) == rclcpp::FutureReturnCode::TIMEOUT)
  {
    FAIL("Timed out waiting for new schedule to be executed");
  }
}

TEST_CASE("cancel_schedule")
{
  auto client = node->create_client<rmf_scheduler_msgs::srv::CancelSchedule>(
    "cancel_schedule");
  if (!client->wait_for_service(5s))
  {
    FAIL("Timed out waiting for service");
  }

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::CancelSchedule::Request>();
  check_response(client->async_send_request(req));
}

TEST_CASE("list schedules")
{
  auto client = node->create_client<rmf_scheduler_msgs::srv::ListSchedules>(
    "list_schedules");
  if (!client->wait_for_service(5s))
  {
    FAIL("Timed out waiting for service");
  }

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::ListSchedules::Request>();
  req->created_after = 10;

  check_response(client->async_send_request(req));
}

TEST_CASE("list schedule states")
{
  auto client =
    node->create_client<rmf_scheduler_msgs::srv::ListScheduleStates>(
    "list_schedule_states");
  if (!client->wait_for_service(5s))
  {
    FAIL("Timed out waiting for service");
  }

  auto req =
    std::make_shared<rmf_scheduler_msgs::srv::ListScheduleStates::Request>();
  req->modified_after = 10;

  check_response(client->async_send_request(req));
}

}

int main(int argc, char* argv[])
{
  using namespace rmf::scheduler::test;

  std::atexit([]()
    {
      killpg(processes.native_handle(), SIGTERM);
      processes.join();
    });

  rclcpp::init(0, nullptr);
  node = rclcpp::Node::make_shared("test_node");

  std::filesystem::remove("e2e.sqlite3");
  auto server = bp::child{
    bp::search_path("ros2"),
    "run", "rmf_scheduler_ros2", "rmf_scheduler_ros2", "--ros-args", "-p",
    "db_file:=e2e.sqlite3",
    bp::std_out > bp::null,
    bp::std_in<bp::null,
      bp::std_err> bp::null,
    processes
  };

  int result = Catch::Session().run(argc, argv);
  return result;
}
