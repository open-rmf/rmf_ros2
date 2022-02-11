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
#include <rmf_task_ros2/bidding/AsyncBidder.hpp>
#include <rmf_task_ros2/StandardNames.hpp>

#include <rmf_task_msgs/srv/submit_task.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <rmf_task_msgs/srv/get_dispatch_states.hpp>

#include <rmf_task_msgs/srv/submit_task.hpp>

#include <chrono>
#include <thread>
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {

//==============================================================================
SCENARIO("Dispatcher API Test", "[Dispatcher][.flaky]")
{
  rmf_task_msgs::msg::TaskDescription task_desc1;
  rmf_task_msgs::msg::TaskDescription task_desc2;
  task_desc1.task_type.type = rmf_task_msgs::msg::TaskType::TYPE_STATION;
  task_desc2.task_type.type = rmf_task_msgs::msg::TaskType::TYPE_CLEAN;

  //============================================================================
  const auto rcl_context = std::make_shared<rclcpp::Context>();
  rcl_context->init(0, nullptr);

  const auto node = std::make_shared<rclcpp::Node>(
    "test_dispatcher_node", rclcpp::NodeOptions().context(rcl_context));

  const auto dispatcher = Dispatcher::make(node);
  auto dispatcher_spin_thread = std::thread(
    [dispatcher]()
    {
      dispatcher->spin();
    });

  WHEN("Check service interfaces")
  {
    using SubmitTaskSrv = rmf_task_msgs::srv::SubmitTask;
    using CancelTaskSrv = rmf_task_msgs::srv::CancelTask;
    using GetDispatchStatesSrv = rmf_task_msgs::srv::GetDispatchStates;

    auto submit_client = dispatcher->node()->create_client<SubmitTaskSrv>(
      rmf_task_ros2::SubmitTaskSrvName);
    REQUIRE(submit_client->wait_for_service(std::chrono::milliseconds(0)));
    auto cancel_client = dispatcher->node()->create_client<CancelTaskSrv>(
      rmf_task_ros2::CancelTaskSrvName);
    REQUIRE(cancel_client->wait_for_service(std::chrono::milliseconds(0)));
    auto get_tasks_client =
      dispatcher->node()->create_client<GetDispatchStatesSrv>(
      rmf_task_ros2::GetDispatchStatesSrvName);
    REQUIRE(get_tasks_client->wait_for_service(std::chrono::milliseconds(0)));
  }

  WHEN("Add 1 and cancel task")
  {
    // add task
    const auto id = dispatcher->submit_task(task_desc1);
    REQUIRE(id.has_value());
    CHECK(dispatcher->active_dispatches().size() == 1);
    CHECK(dispatcher->finished_dispatches().size() == 0);
    const auto state = dispatcher->get_dispatch_state(*id);
    REQUIRE(state.has_value());
    REQUIRE(state->status == DispatchState::Status::Queued);

    // cancel task
    REQUIRE(dispatcher->cancel_task(*id));
    REQUIRE(dispatcher->active_dispatches().size() == 0);
    REQUIRE(dispatcher->finished_dispatches().size() == 1);

    // check nonsense id
    REQUIRE(!(dispatcher->get_dispatch_state("non_existent_id")));

    // Try sleeping for a moment here to mitigate race conditions.
    // TODO(MXG): We should rework these tests so that we don't need to put in
    // arbitrary waits, and so that we can ensure that the correct behavior is
    // happening, exactly as intended.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // add an invalid task
    task_desc2.task_type.type = 10; // this is invalid
    REQUIRE(dispatcher->submit_task(task_desc2) == std::nullopt);
  }

  //============================================================================
  // test on change fn callback
  const auto change_times = std::make_shared<int>(0);
  const auto test_id = std::make_shared<std::string>();
  dispatcher->on_change(
    [change_times, test_id](const auto& state)
    {
      *test_id = state.task_id;
      (*change_times)++;
    }
  );

  WHEN("Track Task till Bidding Failed")
  {
    // Submit first task and wait for bidding
    auto id = dispatcher->submit_task(task_desc1);
    REQUIRE(dispatcher->active_dispatches().size() == 1);
    const auto state = dispatcher->get_dispatch_state(*id);
    REQUIRE(state.has_value());
    REQUIRE(state->status == DispatchState::Status::Queued);

    // Default 2s timeout, wait 3s for timetout, should fail here
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));
    CHECK(dispatcher->get_dispatch_state(*id)->status ==
      DispatchState::Status::FailedToAssign);

    CHECK(dispatcher->finished_dispatches().size() == 1);
    // TODO(MXG): Flake out after previous line: SIGABRT
    CHECK(*test_id == id);
    CHECK(*change_times == 2); // add and failed

    // Submit another task
    id = dispatcher->submit_task(task_desc2);
    REQUIRE(id.has_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    REQUIRE(dispatcher->finished_dispatches().size() == 2);
    REQUIRE(*test_id == *id);
    CHECK(*change_times == 4); // add and failed x2
  }

  //============================================================================
  // Setup Mock Fleetadapter: mock bidder to test
  auto bidder = bidding::AsyncBidder::make(
    node,
    [](const bidding::BidNoticeMsg& notice, auto respond)
    {
      const auto request = nlohmann::json::parse(notice.request);
      if (request["category"] != "patrol")
        return respond(bidding::Response{std::nullopt, {}});

      // Provide a best estimate
      bidding::Response::Proposal best_robot_estimate;
      best_robot_estimate.fleet_name = "dummy_fleet";
      best_robot_estimate.new_cost = 13.5;
      respond(bidding::Response{best_robot_estimate, {}});
    }
  );

  rclcpp::shutdown(rcl_context);
  dispatcher_spin_thread.join();
}

} // namespace rmf_task_ros2
