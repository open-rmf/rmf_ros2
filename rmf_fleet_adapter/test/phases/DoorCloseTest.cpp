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

#include <phases/DoorClose.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

using rmf_door_msgs::msg::DoorRequest;
using rmf_door_msgs::msg::DoorMode;
using rmf_door_msgs::msg::DoorState;
using rmf_door_msgs::msg::SupervisorHeartbeat;
using rmf_door_msgs::msg::DoorSessions;
using rmf_door_msgs::msg::Session;

struct DoorCloseFixture : TransportFixture
{
  rxcpp::observable<DoorState::SharedPtr> door_state_obs;
  rxcpp::observable<SupervisorHeartbeat::SharedPtr> heartbeat_obs;
  std::shared_ptr<Task::PendingPhase> pending_phase;
  std::shared_ptr<Task::ActivePhase> active_phase;
  std::string door_name = "test_door";

  DoorCloseFixture() : TransportFixture()
  {
    door_state_obs = transport->create_observable<DoorState>(DoorStateTopicName, 10);
    heartbeat_obs = transport->create_observable<SupervisorHeartbeat>(DoorSupervisorHeartbeatTopicName, 10);
    pending_phase = std::make_shared<DoorClose::PendingPhase>(
      door_name,
      transport,
      door_state_obs,
      heartbeat_obs
    );
    active_phase = pending_phase->begin();
  }
};

TEST_CASE_METHOD(DoorCloseFixture, "sends door close request", "[DoorClose]")
{
  bool received = false;
  rxcpp::composite_subscription rx_subscription;
  auto rcl_subscription = transport->create_subscription<DoorRequest>(
    AdapterDoorRequestTopicName,
    10,
    [&](DoorRequest::UniquePtr)
    {
      received = true;
      rx_subscription.unsubscribe();
    });
  active_phase->observe().as_blocking().subscribe(rx_subscription);
  REQUIRE(received);
}

/**
 * see DoorOpenTest "waits for supervisor heartbeat"
 */
TEST_CASE_METHOD(DoorCloseFixture, "door close waits for supervisor heartbeat", "[DoorClose]")
{
  auto door_state_pub = transport->create_publisher<DoorState>(DoorStateTopicName, 10);
  auto heartbeat_pub = transport->create_publisher<SupervisorHeartbeat>(DoorSupervisorHeartbeatTopicName, 10);
  auto subscription = transport->create_subscription<DoorRequest>(
    AdapterDoorRequestTopicName,
    10,
    [&](DoorRequest::UniquePtr door_request)
    {
      DoorState door_state;
      door_state.door_name = door_name;
      door_state.door_time = transport->now();
      door_state.current_mode.value = door_request->requested_mode.value;
      door_state_pub->publish(door_state);

      Session session;
      session.requester_id = door_request->requester_id;
      session.request_time = door_request->request_time;
      DoorSessions door_sessions;
      door_sessions.door_name = door_request->door_name;
      door_sessions.sessions.emplace_back(std::move(session));
      SupervisorHeartbeat heartbeat;
      heartbeat.all_sessions.emplace_back(std::move(door_sessions));
      heartbeat_pub->publish(heartbeat);

      heartbeat_pub->publish(SupervisorHeartbeat());
    });

  int call_count = 0;
  active_phase->observe().as_blocking().subscribe([&](const auto&)
  {
    call_count++;
  });
  REQUIRE(call_count == 2);
}

TEST_CASE_METHOD(DoorCloseFixture, "continuously send door close requests", "[DoorClose]")
{
  int received = 0;
  rxcpp::composite_subscription rx_subscription;
  auto rcl_subscription = transport->create_subscription<DoorRequest>(
    AdapterDoorRequestTopicName,
    10,
    [&](DoorRequest::UniquePtr)
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