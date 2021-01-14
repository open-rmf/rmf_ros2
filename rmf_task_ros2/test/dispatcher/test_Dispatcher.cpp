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

#include <rclcpp/rclcpp.hpp>
#include <rmf_task_ros2/Dispatcher.hpp>

// mock Fleet Adapter to test dispatcher
#include <rmf_task_ros2/bidding/MinimalBidder.hpp>
#include "../../src/rmf_task_ros2/action/Server.hpp"

#include <rmf_task_msgs/srv/submit_task.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <rmf_task_msgs/srv/get_task_list.hpp>

#include <chrono>
#include <thread>
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {

//==============================================================================
SCENARIO("Dispatcehr API Test", "[Dispatcher]")
{
  Dispatcher::TaskDescription task_desc1;
  Dispatcher::TaskDescription task_desc2;
  task_desc1.task_type.type = rmf_task_msgs::msg::TaskType::TYPE_STATION;
  task_desc2.task_type.type = rmf_task_msgs::msg::TaskType::TYPE_CLEAN;

  //============================================================================
  auto dispatcher = Dispatcher::init_and_make_node("test_dispatcher_node");

  auto spin_thread = std::thread(
    [&dispatcher]()
    {
      dispatcher->spin();
    });
  spin_thread.detach();


  WHEN("Check service interfaces")
  {
    using SubmitTaskSrv = rmf_task_msgs::srv::SubmitTask;
    using CancelTaskSrv = rmf_task_msgs::srv::CancelTask;
    using GetTaskListSrv = rmf_task_msgs::srv::GetTaskList;

    auto submit_client = dispatcher->node()->create_client<SubmitTaskSrv>(
      rmf_task_ros2::SubmitTaskSrvName);
    REQUIRE(submit_client->wait_for_service(std::chrono::milliseconds(0)));
    auto cancel_client = dispatcher->node()->create_client<CancelTaskSrv>(
      rmf_task_ros2::CancelTaskSrvName);
    REQUIRE(cancel_client->wait_for_service(std::chrono::milliseconds(0)));
    auto get_tasks_client = dispatcher->node()->create_client<GetTaskListSrv>(
      rmf_task_ros2::GetTaskListSrvName);
    REQUIRE(get_tasks_client->wait_for_service(std::chrono::milliseconds(0)));
  }

  WHEN("Add 1 and cancel task")
  {
    // add task
    const auto id = dispatcher->submit_task(task_desc1);
    REQUIRE(dispatcher->active_tasks().size() == 1);
    REQUIRE(dispatcher->terminated_tasks().size() == 0);
    REQUIRE(dispatcher->get_task_state(*id) == TaskStatus::State::Pending);

    // cancel task
    REQUIRE(dispatcher->cancel_task(*id));
    REQUIRE(dispatcher->active_tasks().size() == 0);
    REQUIRE(dispatcher->terminated_tasks().size() == 1);

    // check random id
    REQUIRE(!(dispatcher->get_task_state("non_existence_id")));

    // add an invalid task
    task_desc2.task_type.type = 10; // this is invalid
    REQUIRE(dispatcher->submit_task(task_desc2) == std::nullopt);
  }

  //============================================================================
  // test on change fn callback
  int change_times = 0;
  TaskProfile test_taskprofile;
  dispatcher->on_change(
    [&change_times, &test_taskprofile](const TaskStatusPtr status)
    {
      test_taskprofile = status->task_profile;
      change_times++;
    }
  );

  WHEN("Track Task till Bidding Failed")
  {
    // Submit first task and wait for bidding
    auto id = dispatcher->submit_task(task_desc1);
    REQUIRE(dispatcher->active_tasks().size() == 1);
    REQUIRE(dispatcher->get_task_state(*id) == TaskStatus::State::Pending);

    // Default 2s timeout, wait 3s for timetout, should fail here
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Failed);
    REQUIRE(dispatcher->terminated_tasks().size() == 1);
    REQUIRE(test_taskprofile.task_id == id);
    CHECK(change_times == 2); // add and failed

    // Submit another task
    id = dispatcher->submit_task(task_desc2);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    REQUIRE(dispatcher->terminated_tasks().size() == 2);
    REQUIRE(test_taskprofile.task_id == *id);
    CHECK(change_times == 4); // add and failed x2
  }

  //============================================================================
  // Setup Mock Fleetadapter: mock bidder to test
  using TaskType = bidding::MinimalBidder::TaskType;
  auto node = dispatcher->node();
  auto bidder = bidding::MinimalBidder::make(
    node,
    "dummy_fleet",
    { TaskType::Station, TaskType::Clean },
    [](const bidding::BidNotice&)
    {
      // Provide a best estimate
      bidding::Submission best_robot_estimate;
      best_robot_estimate.new_cost = 13.5;
      return best_robot_estimate;
    }
  );

  //============================================================================
  // Setup Mock Fleetadapter: action server to test
  auto action_server = action::Server::make(node, "dummy_fleet");

  bool task_canceled_flag = false;

  action_server->register_callbacks(
    // Add Task callback
    [&action_server, &task_canceled_flag](const TaskProfile& task_profile)
    {
      // Start action task
      auto t = std::thread(
        [&action_server, &task_canceled_flag](auto profile)
        {
          TaskStatus status;
          status.task_profile = profile;
          status.robot_name = "dumbot";
          std::this_thread::sleep_for(std::chrono::seconds(2));

          if (task_canceled_flag)
          {
            // std::cout << "[task impl] Cancelled!" << std::endl;
            return;
          }

          // Executing
          status.state = TaskStatus::State::Executing;
          action_server->update_status(status);
          std::this_thread::sleep_for(std::chrono::seconds(1));

          // Completed
          status.state = TaskStatus::State::Completed;
          action_server->update_status(status);
        }, task_profile
      );
      t.detach();
      return true; //successs (send State::Queued)
    },
    // Cancel Task callback
    [&action_server, &task_canceled_flag](const TaskProfile&)
    {
      task_canceled_flag = true;
      return true; //success ,send State::Canceled when dispatcher->cancel_task
    }
  );

  //============================================================================
  WHEN("Full Dispatch cycle")
  {
    const auto id = dispatcher->submit_task(task_desc1);
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Pending);
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));

    // now should queue the task
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Queued);
    REQUIRE(dispatcher->terminated_tasks().size() == 0);
    CHECK(change_times == 2); // Pending and Queued

    std::this_thread::sleep_for(std::chrono::seconds(3));
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Completed);
    REQUIRE(dispatcher->active_tasks().size() == 0);
    REQUIRE(dispatcher->terminated_tasks().size() == 1);
    CHECK(change_times == 4); // Pending > Queued > Executing > Completed

    // Add auto generated ChargeBattery Task from fleet adapter
    TaskStatus status;
    status.task_profile.task_id = "ChargeBattery10";
    status.state = TaskStatus::State::Queued;
    status.task_profile.description.task_type.type =
      rmf_task_msgs::msg::TaskType::TYPE_CHARGE_BATTERY;
    action_server->update_status(status);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    CHECK(change_times == 5); // new stray charge task
    REQUIRE(dispatcher->active_tasks().size() == 1);
  }

  WHEN("Half way cancel Dispatch cycle")
  {
    const auto id = dispatcher->submit_task(task_desc2);
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Pending);
    REQUIRE(dispatcher->active_tasks().size() == 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // cancel the task after QUEUED State
    REQUIRE(dispatcher->cancel_task(*id));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    REQUIRE(dispatcher->active_tasks().size() == 0);
    REQUIRE(dispatcher->terminated_tasks().size() == 1);
    REQUIRE(dispatcher->terminated_tasks().begin()->first == *id);
    auto status = dispatcher->terminated_tasks().begin()->second;
    CHECK(status->state == TaskStatus::State::Canceled);
    CHECK(change_times == 3); // Pending -> Queued -> Canceled
  }

  rclcpp::shutdown();
}

} // namespace rmf_task_ros2
