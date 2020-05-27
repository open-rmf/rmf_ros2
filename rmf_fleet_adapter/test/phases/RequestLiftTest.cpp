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

#include <phases/RequestLift.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

using rmf_lift_msgs::msg::LiftState;
using rmf_lift_msgs::msg::LiftRequest;

struct RequestLiftFixture : TransportFixture
{
  rxcpp::observable<LiftState::SharedPtr> lift_state_obs;
  std::shared_ptr<Task::PendingPhase> pending_phase;
  std::shared_ptr<Task::ActivePhase> active_phase;
  std::string lift_name = "test_lift";
  std::string destination = "test_floor";

  RequestLiftFixture() : TransportFixture()
  {
    lift_state_obs = transport->create_observable<LiftState>(LiftStateTopicName, 10);
    pending_phase = std::make_shared<RequestLift::PendingPhase>(
      transport,
      lift_name,
      destination,
      lift_state_obs
    );
    active_phase = pending_phase->begin();
  }
};

TEST_CASE_METHOD(RequestLiftFixture, "publishes lift request", "[RequestLift]")
{
  bool received = false;
  rxcpp::composite_subscription rx_subscription;
  auto rcl_subscription = transport->create_subscription<LiftRequest>(
    AdapterLiftRequestTopicName,
    10,
    [&](LiftRequest::UniquePtr)
    {
      received = true;
      rx_subscription.unsubscribe();
    });
  active_phase->observe().as_blocking().subscribe(rx_subscription);
  REQUIRE(received);
}

TEST_CASE_METHOD(RequestLiftFixture, "continuously send lift requests", "[RequestLift]")
{
  int received = 0;
  rxcpp::composite_subscription rx_subscription;
  auto rcl_subscription = transport->create_subscription<LiftRequest>(
    AdapterLiftRequestTopicName,
    10,
    [&](LiftRequest::UniquePtr)
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