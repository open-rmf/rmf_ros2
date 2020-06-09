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

SCENARIO_METHOD(TransportFixture, "door close phase", "[phases]")
{
  std::mutex m;
  std::condition_variable received_requests_cv;
  std::list<DoorRequest::UniquePtr> received_requests;
  auto rcl_subscription = transport->create_subscription<DoorRequest>(
    AdapterDoorRequestTopicName,
    10,
    [&](DoorRequest::UniquePtr door_request)
    {
      std::unique_lock<std::mutex> lk(m);
      received_requests.emplace_back(std::move(door_request));
      received_requests_cv.notify_all();
    });

  GIVEN("a door close phase")
  {
    std::string door_name = "test_door";
    std::string request_id = "test_id";
    auto heartbeat_obs = transport->create_observable<SupervisorHeartbeat>(DoorSupervisorHeartbeatTopicName, 10);
    auto pending_phase = std::make_shared<DoorClose::PendingPhase>(
      door_name,
      request_id,
      transport,
      heartbeat_obs
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

      THEN("it should send door close request")
      {
        std::unique_lock<std::mutex> lk(m);
        if (received_requests.empty())
          received_requests_cv.wait(lk, [&]() { return !received_requests.empty(); });
        REQUIRE(received_requests.size() == 1);
        REQUIRE(received_requests.front()->requested_mode.value == DoorMode::MODE_CLOSED);
      }

      THEN("it should continuously send door close requests")
      {
        std::unique_lock<std::mutex> lk(m);
        received_requests_cv.wait(lk, [&]() { return received_requests.size() >= 3; });
        for (const auto& door_request : received_requests)
        {
          REQUIRE(door_request->requested_mode.value == DoorMode::MODE_CLOSED);
        }
      }

      THEN("it is completed after door supervisor releases session, regardless of the door state")
      {
        auto door_state_pub = transport->create_publisher<DoorState>(DoorStateTopicName, 10);
        auto heartbeat_pub = transport->create_publisher<SupervisorHeartbeat>(DoorSupervisorHeartbeatTopicName, 10);
        auto rcl_subscription2 = transport->create_subscription<DoorRequest>(
          AdapterDoorRequestTopicName,
          10,
          [&](DoorRequest::UniquePtr door_request)
          {
            DoorState door_state;
            door_state.door_name = door_name;
            door_state.door_time = transport->now();
            door_state.current_mode.value = DoorMode::MODE_OPEN;
            door_state_pub->publish(door_state);

            // supervisor has session
            Session session;
            session.requester_id = door_request->requester_id;
            session.request_time = door_request->request_time;
            DoorSessions door_sessions;
            door_sessions.door_name = door_request->door_name;
            door_sessions.sessions.emplace_back(std::move(session));
            SupervisorHeartbeat heartbeat;
            heartbeat.all_sessions.emplace_back(std::move(door_sessions));
            heartbeat_pub->publish(heartbeat);
          });

        // phase should not complete yet as supervisor haven't release the session
        {
          std::unique_lock<std::mutex> lk(m);
          bool completed = status_updates_cv.wait_for(lk, std::chrono::seconds(1), [&]()
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

        // supervisor released session, door state still opened
        heartbeat_pub->publish(SupervisorHeartbeat());

        // phase should now complete
        {
          std::unique_lock<std::mutex> lk(m);
          bool completed = status_updates_cv.wait_for(lk, std::chrono::seconds(1), [&]()
          {
            for (const auto& status : status_updates)
            {
              if (status.state == Task::StatusMsg::STATE_COMPLETED)
                return true;
            }
            status_updates.clear();
            return false;
          });
          REQUIRE(completed);
        }
      }

      THEN("cancelled, it should not do anything")
      {
        active_phase->cancel();
        std::unique_lock<std::mutex> lk(m);

        bool completed = status_updates_cv.wait_for(lk, std::chrono::seconds(1), [&]()
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

      sub.unsubscribe();
    }
  }
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter