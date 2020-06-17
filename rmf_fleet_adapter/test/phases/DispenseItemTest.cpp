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

#include "MockAdapterFixture.hpp"

#include <phases/DispenseItem.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

using rmf_dispenser_msgs::msg::DispenserResult;
using rmf_dispenser_msgs::msg::DispenserRequest;
using rmf_dispenser_msgs::msg::DispenserRequestItem;
using rmf_dispenser_msgs::msg::DispenserState;

SCENARIO_METHOD(MockAdapterFixture, "dispense item phase", "[phases]")
{
  std::mutex m;
  std::condition_variable received_requests_cv;
  std::list<DispenserRequest> received_requests;
  auto rcl_subscription = adapter->node()->create_subscription<DispenserRequest>(
    DispenserRequestTopicName,
    10,
    [&](DispenserRequest::UniquePtr dispenser_request)
    {
      std::unique_lock<std::mutex> lk(m);
      received_requests.emplace_back(*dispenser_request);
      received_requests_cv.notify_all();
    });

  std::string request_guid = "test_guid";
  std::string target = "test_dispenser";
  std::string transporter_type = "test_type";
  std::vector<DispenserRequestItem> items;
  DispenserRequestItem item;
  item.type_guid = "test_item_type";
  item.compartment_name = "test_compartment";
  item.quantity = 1;
  items.emplace_back(std::move(item));


  const auto info = add_robot();
  const auto& context = info.context;

  auto dispenser_request_pub = adapter->node()->create_publisher<DispenserRequest>(
    DispenserRequestTopicName, 10);
  auto pending_phase = std::make_shared<DispenseItem::PendingPhase>(
    context,
    request_guid,
    target,
    transporter_type,
    items
  );
  auto active_phase = pending_phase->begin();

  WHEN("it is started")
  {
    std::condition_variable status_updates_cv;
    std::list<Task::StatusMsg> status_updates;
    auto sub = active_phase->observe().subscribe(
      [&](const auto& status)
      {
        std::unique_lock<std::mutex> lk(m);
        status_updates.emplace_back(status);
        status_updates_cv.notify_all();
      });

    THEN("it should send dispense item request")
    {
      std::unique_lock<std::mutex> lk(m);
      if (received_requests.empty())
        received_requests_cv.wait(lk, [&]() { return !received_requests.empty(); });
      REQUIRE(received_requests.size() == 1);
    }

    THEN("it should continuously send dispense item request")
    {
      std::unique_lock<std::mutex> lk(m);
      received_requests_cv.wait(lk, [&]() { return received_requests.size() >= 3; });
    }

    THEN("cancelled, it should not do anything")
    {
      active_phase->cancel();
      std::unique_lock<std::mutex> lk(m);

      bool completed = status_updates_cv.wait_for(lk, std::chrono::seconds(3), [&]()
      {
        for (const auto& status : status_updates)
        {
          if (status.state == Task::StatusMsg::STATE_COMPLETED)
            return true;
        }
        status_updates.clear();
        return false;
      });
      CHECK(!completed);
    }

    AND_WHEN("dispenser result is success")
    {
      auto result_pub = ros_node->create_publisher<DispenserResult>(DispenserResultTopicName, 10);
      auto state_pub = ros_node->create_publisher<DispenserState>(DispenserStateTopicName, 10);
      auto timer = ros_node->create_wall_timer(std::chrono::milliseconds(100), [&]()
      {
        std::unique_lock<std::mutex> lk(m);
        DispenserResult result;
        result.request_guid = request_guid;
        result.status = DispenserResult::SUCCESS;
        result.time = ros_node->now();
        result_pub->publish(result);

        DispenserState state;
        state.guid = request_guid;
        state.mode = DispenserState::IDLE;
        state.time = ros_node->now();
        state_pub->publish(state);
      });

      THEN("it is completed")
      {
        std::unique_lock<std::mutex> lk(m);
        bool completed = status_updates_cv.wait_for(lk, std::chrono::milliseconds(1000), [&]()
        {
          return status_updates.back().state == Task::StatusMsg::STATE_COMPLETED;
        });
        CHECK(completed);
      }

      timer.reset();
    }

    AND_WHEN("dispenser result is failed")
    {
      auto result_pub = ros_node->create_publisher<DispenserResult>(DispenserResultTopicName, 10);
      auto state_pub = ros_node->create_publisher<DispenserState>(DispenserStateTopicName, 10);
      auto timer = ros_node->create_wall_timer(std::chrono::milliseconds(100), [&]()
      {
        std::unique_lock<std::mutex> lk(m);
        DispenserResult result;
        result.request_guid = request_guid;
        result.status = DispenserResult::FAILED;
        result.time = ros_node->now();
        result_pub->publish(result);

        DispenserState state;
        state.guid = request_guid;
        state.mode = DispenserState::IDLE;
        state.time = ros_node->now();
        state_pub->publish(state);
      });

      THEN("it is failed")
      {
        std::unique_lock<std::mutex> lk(m);
        bool failed = status_updates_cv.wait_for(lk, std::chrono::milliseconds(1000), [&]()
        {
          return status_updates.back().state == Task::StatusMsg::STATE_FAILED;
        });
        CHECK(failed);
      }

      timer.reset();
    }

    AND_WHEN("request is acknowledged and request is no longer in queue")
    {
      auto result_pub = ros_node->create_publisher<DispenserResult>(DispenserResultTopicName, 10);
      auto state_pub = ros_node->create_publisher<DispenserState>(DispenserStateTopicName, 10);
      auto interval = rxcpp::observable<>::interval(std::chrono::milliseconds(100))
        .subscribe_on(rxcpp::observe_on_new_thread())
        .subscribe([&](const auto&)
        {
          DispenserResult result;
          result.request_guid = request_guid;
          result.status = DispenserResult::ACKNOWLEDGED;
          result.time = ros_node->now();
          result_pub->publish(result);

          DispenserState state;
          state.time = ros_node->now();
          state.guid = target;
          state.mode = DispenserState::BUSY;
          state_pub->publish(state);
        });

      THEN("it is completed")
      {
        std::unique_lock<std::mutex> lk(m);
        bool completed = status_updates_cv.wait_for(lk, std::chrono::milliseconds(1000), [&]()
        {
          return status_updates.back().state == Task::StatusMsg::STATE_COMPLETED;
        });
        CHECK(completed);
      }

      interval.unsubscribe();
    }

    AND_WHEN("request acknowledged result arrives before request state in queue")
    {
      auto result_pub = ros_node->create_publisher<DispenserResult>(DispenserResultTopicName, 10);
      auto state_pub = ros_node->create_publisher<DispenserState>(DispenserStateTopicName, 10);
      auto interval = rxcpp::observable<>::interval(std::chrono::milliseconds(100))
        .subscribe_on(rxcpp::observe_on_new_thread())
        .subscribe([&](const auto&)
        {
          DispenserResult result;
          result.request_guid = request_guid;
          result.status = DispenserResult::ACKNOWLEDGED;
          result.time = ros_node->now();
          result_pub->publish(result);

          DispenserState state;
          // simulate state arriving late
          state.time.sec = 0;
          state.time.nanosec = 0;
          state.guid = target;
          state.mode = DispenserState::BUSY;
          state_pub->publish(state);
        });

      THEN("it is not completed")
      {
        std::unique_lock<std::mutex> lk(m);
        bool completed = status_updates_cv.wait_for(lk, std::chrono::milliseconds(1000), [&]()
        {
          return status_updates.back().state == Task::StatusMsg::STATE_COMPLETED;
        });
        CHECK(!completed);
      }

      interval.unsubscribe();
    }

    sub.unsubscribe();
  }
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter
