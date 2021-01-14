/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <rmf_utils/optional.hpp>
#include "../../src/rmf_task_ros2/action/Client.hpp"
#include "../../src/rmf_task_ros2/action/Server.hpp"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {
namespace action {

//==============================================================================
SCENARIO("Action communication with client and server", "[ActionInterface]")
{
  TaskProfile task_profile1;
  task_profile1.task_id = "task1";
  task_profile1.description.task_type.type =
    rmf_task_msgs::msg::TaskType::TYPE_STATION;

  TaskProfile task_profile2;
  task_profile2.task_id = "task2";
  task_profile2.description.task_type.type =
    rmf_task_msgs::msg::TaskType::TYPE_STATION;

  //============================================================================

  // received task to test
  std::optional<TaskProfile> test_add_task;
  std::optional<TaskProfile> test_cancel_task;

  // Creating 1 auctioneer and 1 bidder
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_ActionInferface");
  auto action_server = Server::make(node, "test_server");
  auto action_client = Client::make(node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // received test request msg from client
  action_server->register_callbacks(
    // add task
    [&test_add_task](const TaskProfile& task_profile)
    {
      test_add_task = task_profile;
      return true;
    },
    // cancel task
    [&test_cancel_task](const TaskProfile& task_profile)
    {
      test_cancel_task = task_profile;
      return true;
    }
  );

  // ROS Spin: forever incompleted future
  std::promise<void> ready_promise;
  std::shared_future<void> ready_future(ready_promise.get_future());

  WHEN("Add Task")
  {
    // Add invalid Task!
    TaskStatusPtr status_ptr(new TaskStatus);

    action_client->add_task("wrong_server", task_profile1, status_ptr);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(0.5));

    // should not receive cuz incorrect serverid
    REQUIRE(status_ptr->state == TaskStatus::State::Pending);

    action_client->add_task("test_server", task_profile1, status_ptr);
    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(0.5));

    // check status
    REQUIRE(status_ptr->state == TaskStatus::State::Queued);

    // status ptr is destroyed, should not have anymore tracking
    status_ptr.reset();
    REQUIRE(action_client->size() == 0);
  }

  WHEN("Cancel Task")
  {
    // send valid task
    TaskStatusPtr status_ptr(new TaskStatus);
    action_client->add_task("test_server", task_profile2, status_ptr);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(0.5));

    // Invalid Cancel Task!
    bool cancel_success = action_client->cancel_task(task_profile1);
    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(0.5));
    REQUIRE(!test_cancel_task);
    REQUIRE(cancel_success == false); // cancel failed
    REQUIRE(action_client->size() == 1);

    // Valid Cancel task
    action_client->cancel_task(task_profile2);
    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(0.5));

    REQUIRE(test_cancel_task);
    REQUIRE(*test_cancel_task == task_profile2);
    REQUIRE(status_ptr->is_terminated());
    REQUIRE(action_client->size() == 0);
  }

  //============================================================================

  std::optional<TaskStatus> test_task_onchange;
  std::optional<TaskStatus> test_task_onterminate;

  // received status update from server
  action_client->on_change(
    [&test_task_onchange](const TaskStatusPtr status)
    {
      test_task_onchange = *status;
    }
  );
  action_client->on_terminate(
    [&test_task_onterminate](const TaskStatusPtr status)
    {
      test_task_onterminate = *status;
    }
  );

  WHEN("On Change and On Terminate Task")
  {
    TaskStatusPtr status_ptr(new TaskStatus);
    action_client->add_task("test_server", task_profile1, status_ptr);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(0.5));

    REQUIRE(test_task_onchange);
    REQUIRE(test_task_onchange->state == TaskStatus::State::Queued);

    TaskStatus server_task;
    server_task.task_profile = task_profile1;
    server_task.state = TaskStatus::State::Executing;

    // Update it as executing
    action_server->update_status(server_task);
    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(1.5));

    REQUIRE(test_task_onchange->state == TaskStatus::State::Executing);
    REQUIRE(!test_task_onterminate); // havnt terminated yet

    // completion
    server_task.state = TaskStatus::State::Completed;
    // Update it as executing
    action_server->update_status(server_task);
    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(0.5));

    REQUIRE(test_task_onterminate);
    REQUIRE(test_task_onterminate->state == TaskStatus::State::Completed);
  }
  rclcpp::shutdown();
}

} // namespace action
} // namespace rmf_task_ros2
