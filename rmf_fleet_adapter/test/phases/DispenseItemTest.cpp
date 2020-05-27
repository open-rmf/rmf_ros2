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

struct DispenseItemFixture : TransportFixture
{
  rxcpp::observable<DispenserResult::SharedPtr> dispenser_result_obs;
  std::shared_ptr<Task::PendingPhase> pending_phase;
  std::shared_ptr<Task::ActivePhase> active_phase;
  std::string target = "test_dispenser";
  std::string transporter_type = "test_type";
  std::vector<DispenserRequestItem> items;

  DispenseItemFixture() : TransportFixture()
  {
    DispenserRequestItem item;
    item.type_guid = "test_item_type";
    item.compartment_name = "test_compartment";
    item.quantity = 1;
    items.emplace_back(std::move(item));

    dispenser_result_obs = transport->create_observable<DispenserResult>(DispenserResultTopicName, 10);
    pending_phase = std::make_shared<DispenseItem::PendingPhase>(
      transport,
      target,
      transporter_type,
      items,
      dispenser_result_obs
    );
    active_phase = pending_phase->begin();
  }
};

TEST_CASE_METHOD(DispenseItemFixture, "publishes dispenser request", "[DispenseItem]")
{
  bool received = false;
  rxcpp::composite_subscription rx_subscription;
  auto rcl_subscription = transport->create_subscription<DispenserRequest>(
    DispenserRequestTopicName,
    10,
    [&](DispenserRequest::UniquePtr)
    {
      received = true;
      rx_subscription.unsubscribe();
    });
  active_phase->observe().as_blocking().subscribe(rx_subscription);
  REQUIRE(received);
}

TEST_CASE_METHOD(DispenseItemFixture, "continuously send dispenser requests", "[DispenseItem]")
{
  int received = 0;
  rxcpp::composite_subscription rx_subscription;
  auto rcl_subscription = transport->create_subscription<DispenserRequest>(
    DispenserRequestTopicName,
    10,
    [&](DispenserRequest::UniquePtr)
    {
      received++;
      if (received >= 2)
        rx_subscription.unsubscribe();
    });
  active_phase->observe().as_blocking().subscribe(rx_subscription);
  REQUIRE(received >= 2);
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter