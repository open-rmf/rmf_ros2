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

#include "TransportFixture.hpp"

#include <phases/DispenseItem.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

using rmf_dispenser_msgs::msg::DispenserResult;
using rmf_dispenser_msgs::msg::DispenserRequest;
using rmf_dispenser_msgs::msg::DispenserRequestItem;

SCENARIO_METHOD(TransportFixture, "dispense item phase", "[phases]")
{
  std::mutex m;
  std::condition_variable received_requests_cv;
  std::list<DispenserRequest> received_requests;
  std::string request_guid;
  auto rcl_subscription = transport->create_subscription<DispenserRequest>(
    DispenserRequestTopicName,
    10,
    [&](DispenserRequest::UniquePtr dispenser_request)
    {
      std::unique_lock<std::mutex> lk(m);
      request_guid = dispenser_request->request_guid;
      received_requests.emplace_back(*dispenser_request);
      received_requests_cv.notify_all();
    });

  GIVEN("a dispense item phase")
  {
    std::string target = "test_dispenser";
    std::string transporter_type = "test_type";
    std::vector<DispenserRequestItem> items;
    DispenserRequestItem item;
    item.type_guid = "test_item_type";
    item.compartment_name = "test_compartment";
    item.quantity = 1;
    items.emplace_back(std::move(item));

    auto dispenser_result_obs = transport->create_observable<DispenserResult>(DispenserResultTopicName, 10);
    auto pending_phase = std::make_shared<DispenseItem::PendingPhase>(
      transport,
      target,
      transporter_type,
      items,
      dispenser_result_obs
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
        REQUIRE(!completed);
      }

      AND_WHEN("dispenser result is success")
      {
        auto dispenser_result_pub = transport->create_publisher<DispenserResult>(DispenserResultTopicName, 10);
        rclcpp::TimerBase::SharedPtr timer;
        std::function<void()> publish_result = [&]()
        {
          std::unique_lock<std::mutex> lk(m);
          DispenserResult result;
          result.request_guid = request_guid;
          result.status = DispenserResult::SUCCESS;
          result.time = transport->now();
          dispenser_result_pub->publish(result);
          timer = transport->create_wall_timer(std::chrono::milliseconds(100), publish_result);
        };
        publish_result();

        THEN("it is completed")
        {
          std::unique_lock<std::mutex> lk(m);
          bool completed = status_updates_cv.wait_for(lk, std::chrono::milliseconds(1000), [&]()
          {
            return status_updates.back().state == Task::StatusMsg::STATE_COMPLETED;
          });
          REQUIRE(completed);
        }

        timer.reset();
      }

      AND_WHEN("dispenser result is failed")
      {
        auto dispenser_result_pub = transport->create_publisher<DispenserResult>(DispenserResultTopicName, 10);
        rclcpp::TimerBase::SharedPtr timer;
        std::function<void()> publish_result = [&]()
        {
          std::unique_lock<std::mutex> lk(m);
          DispenserResult result;
          result.request_guid = request_guid;
          result.status = DispenserResult::FAILED;
          result.time = transport->now();
          dispenser_result_pub->publish(result);
          timer = transport->create_wall_timer(std::chrono::milliseconds(100), publish_result);
        };
        publish_result();

        THEN("it is failed")
        {
          std::unique_lock<std::mutex> lk(m);
          bool completed = status_updates_cv.wait_for(lk, std::chrono::milliseconds(1000), [&]()
          {
            return status_updates.back().state == Task::StatusMsg::STATE_FAILED;
          });
          REQUIRE(completed);
        }

        timer.reset();
      }

      sub.unsubscribe();
    }
  }
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter