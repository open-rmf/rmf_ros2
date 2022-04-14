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

#include <SchedulePayload.hpp>

#include <rmf_scheduler_msgs/srv/create_trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <boost/process.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <signal.h>
#include <sys/types.h>
#include <thread>

#include <cstdlib>
#include <optional>

namespace bp = boost::process;
using namespace std::chrono_literals;

namespace rmf::scheduler::test {

static std::optional<bp::child> server;
static rclcpp::Node::SharedPtr node;
static std::optional<std::thread> ros_thread;

static std::string unique_name()
{
  return "test_" + std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
}

TEST_CASE("trigger flow")
{
  auto client = node->create_client<rmf_scheduler_msgs::srv::CreateTrigger>(
    "create_trigger");
  client->wait_for_service(5s);

  std_msgs::msg::String msg;
  msg.data = "hello world";

  {
    auto req =
      std::make_shared<rmf_scheduler_msgs::srv::CreateTrigger::Request>();
    req->trigger.name = unique_name();
    req->trigger.at = 0;
    req->trigger.payload = make_serialized_message("std_msgs/String",
        "test_topic", msg);
    auto resp_fut = client->async_send_request(req);
    if (resp_fut.wait_for(5s) != std::future_status::ready)
    {
      FAIL("Timed out waiting for response");
    }
    auto resp = resp_fut.get();

    if (!resp->success)
    {
      FAIL(resp->message);
    }
  }
}

}

int main(int argc, char* argv[])
{
  using namespace rmf::scheduler::test;

  std::atexit([]()
    {
      if (server)
      {
        kill(server->id(), SIGTERM);
        server->join();
      }
    });

  rclcpp::init(0, nullptr);
  node = rclcpp::Node::make_shared("test_node");
  ros_thread = std::thread{[]()
    {
      rclcpp::spin(node);
    }};

  server = bp::child{
    bp::search_path("ros2"),
    "run", "rmf_scheduler_ros2", "rmf_scheduler_ros2", "--ros-args", "-p",
    "db_file:=e2e.sqlite3",
    bp::std_out > bp::null,
    bp::std_in<bp::null,
      bp::std_err> bp::null
  };

  int result = Catch::Session().run(argc, argv);
  rclcpp::contexts::get_global_default_context()->shutdown("done");
  ros_thread->join();
  return result;
}
