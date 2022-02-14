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

#include <phases/DoorOpen.hpp>
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


namespace {
struct TestData
{
  std::mutex m;
  std::condition_variable received_requests_cv;
  std::list<DoorRequest::UniquePtr> received_requests;

  std::condition_variable status_updates_cv;
  std::list<LegacyTask::StatusMsg> status_updates;

  std::optional<uint32_t> last_state_value() const
  {
    if (status_updates.empty())
      return std::nullopt;

    return status_updates.back().state;
  }
};
} // anonymous namespace

SCENARIO_METHOD(MockAdapterFixture, "door open phase", "[phases]")
{
  const auto test = std::make_shared<TestData>();
  auto w_test = std::weak_ptr<TestData>(test);
  auto rcl_subscription =
    data->adapter->node()->create_subscription<DoorRequest>(
    AdapterDoorRequestTopicName,
    10,
    [w_test](DoorRequest::UniquePtr door_request)
    {
      auto test = w_test.lock();
      if (!test)
        return;

      std::unique_lock<std::mutex> lk(test->m);
      test->received_requests.emplace_back(std::move(door_request));
      test->received_requests_cv.notify_all();
    });

  const auto info = add_robot();
  const auto& context = info.context;

  std::string door_name = "test_door";
  std::string request_id = "test_id";
  auto pending_phase = std::make_shared<DoorOpen::PendingPhase>(
    context,
    door_name,
    request_id,
    context->now()
  );
  auto active_phase = pending_phase->begin();

  WHEN("it is cancelled before its started")
  {
    active_phase->cancel();

    THEN("it should not send door open requests")
    {
      bool received_open = false;
      rxcpp::composite_subscription rx_sub;
      auto subscription =
        data->adapter->node()->create_subscription<DoorRequest>(
        AdapterDoorRequestTopicName,
        10,
        [&rx_sub, &received_open](DoorRequest::UniquePtr door_request)
        {
          if (door_request->requested_mode.value == DoorMode::MODE_OPEN)
            received_open = true;
          else if (door_request->requested_mode.value == DoorMode::MODE_CLOSED)
            rx_sub.unsubscribe();
        });
      auto obs = active_phase->observe();
      obs.as_blocking().subscribe(rx_sub);
      CHECK(!received_open);
    }
  }

  WHEN("it is started")
  {
    rmf_rxcpp::subscription_guard sub = active_phase->observe().subscribe(
      [w_test](const auto& status)
      {
        auto test = w_test.lock();
        if (!test)
          return;

        std::unique_lock<std::mutex> lk(test->m);
        test->status_updates.emplace_back(status);
        test->status_updates_cv.notify_all();
      });

    THEN("it should send door open request")
    {
      std::unique_lock<std::mutex> lk(test->m);
      if (test->received_requests.empty())
        test->received_requests_cv.wait(lk, [test]()
          {
            return !test->received_requests.empty();
          });
      CHECK(test->received_requests.size() == 1);
      CHECK(
        test->received_requests.front()->requested_mode.value ==
        DoorMode::MODE_OPEN);
    }

    THEN("it should continuously send door open requests")
    {
      std::unique_lock<std::mutex> lk(test->m);
      test->received_requests_cv.wait(lk, [test]()
        {
          return test->received_requests.size() >= 3;
        });
      for (const auto& door_request : test->received_requests)
      {
        CHECK(door_request->requested_mode.value == DoorMode::MODE_OPEN);
      }
    }

    auto door_state_pub =
      data->adapter->node()->create_publisher<DoorState>(
      DoorStateTopicName, 10);
    auto w_door_state_pub =
      std::weak_ptr<rclcpp::Publisher<DoorState>>(door_state_pub);
    auto heartbeat_pub =
      data->adapter->node()->create_publisher<SupervisorHeartbeat>(
      DoorSupervisorHeartbeatTopicName, 10);
    auto w_heartbeat_pub =
      std::weak_ptr<rclcpp::Publisher<SupervisorHeartbeat>>(heartbeat_pub);

    auto publish_door_state =
      [w_node = std::weak_ptr<rclcpp::Node>(data->adapter->node()),
        w_door_state_pub, door_name](uint32_t mode)
      {
        auto door_state_pub = w_door_state_pub.lock();
        if (!door_state_pub)
          return;

        auto node = w_node.lock();
        if (!node)
          return;

        DoorState door_state;
        door_state.door_name = door_name;
        door_state.door_time = node->now();
        door_state.current_mode.value = mode;
        door_state_pub->publish(door_state);
      };

    auto publish_heartbeat_with_session =
      [request_id, door_name, w_heartbeat_pub]()
      {
        auto heartbeat_pub = w_heartbeat_pub.lock();
        if (!heartbeat_pub)
          return;

        Session session;
        session.requester_id = request_id;
        DoorSessions door_sessions;
        door_sessions.door_name = door_name;
        door_sessions.sessions.emplace_back(std::move(session));
        SupervisorHeartbeat heartbeat;
        heartbeat.all_sessions.emplace_back(std::move(door_sessions));
        heartbeat_pub->publish(heartbeat);
      };

    auto publish_empty_heartbeat = [w_heartbeat_pub]()
      {
        auto heartbeat_pub = w_heartbeat_pub.lock();
        if (!heartbeat_pub)
          return;

        heartbeat_pub->publish(SupervisorHeartbeat());
      };

    AND_WHEN("door state is open and supervisor has session")
    {
      rmf_rxcpp::subscription_guard sub2 =
        rxcpp::observable<>::interval(std::chrono::milliseconds(100))
        .subscribe_on(rxcpp::observe_on_new_thread())
        .subscribe(
        [publish_door_state, publish_heartbeat_with_session](const auto&)
        {
          publish_door_state(DoorMode::MODE_OPEN);
          publish_heartbeat_with_session();
        });

      THEN("it is completed")
      {
        std::unique_lock<std::mutex> lk(test->m);
        bool completed = test->status_updates_cv.wait_for(
          lk, std::chrono::milliseconds(1000),
          [test]()
          {
            return test->last_state_value() == LegacyTask::StatusMsg::STATE_COMPLETED;
          });
        CHECK(completed);
      }
    }

    AND_WHEN("door state is open and supervisor do not have session")
    {
      rmf_rxcpp::subscription_guard sub2 =
        rxcpp::observable<>::interval(std::chrono::milliseconds(100))
        .subscribe_on(rxcpp::observe_on_new_thread())
        .subscribe([publish_door_state, publish_empty_heartbeat](const auto&)
          {
            publish_door_state(DoorMode::MODE_OPEN);
            publish_empty_heartbeat();
          });

      THEN("it is not completed")
      {
        std::unique_lock<std::mutex> lk(test->m);
        bool completed = test->status_updates_cv.wait_for(
          lk, std::chrono::milliseconds(1000),
          [test]()
          {
            return test->last_state_value() == LegacyTask::StatusMsg::STATE_COMPLETED;
          });
        CHECK(!completed);
      }
    }

    AND_WHEN("door state is closed and supervisor has session")
    {
      rmf_rxcpp::subscription_guard sub2 =
        rxcpp::observable<>::interval(std::chrono::milliseconds(100))
        .subscribe_on(rxcpp::observe_on_new_thread())
        .subscribe([publish_door_state, publish_empty_heartbeat](const auto&)
          {
            publish_door_state(DoorMode::MODE_CLOSED);
            publish_empty_heartbeat();
          });

      THEN("it is not completed")
      {
        std::unique_lock<std::mutex> lk(test->m);
        bool completed = test->status_updates_cv.wait_for(
          lk, std::chrono::milliseconds(1000),
          [test]()
          {
            return test->last_state_value() == LegacyTask::StatusMsg::STATE_COMPLETED;
          });
        CHECK(!completed);
      }
    }

    AND_WHEN("it is cancelled")
    {
      {
        std::unique_lock<std::mutex> lk(test->m);
        test->received_requests_cv.wait(lk, [test]()
          {
            return !test->received_requests.empty();
          });
        active_phase->cancel();
      }

      THEN("it should send door close request")
      {
        std::unique_lock<std::mutex> lk(test->m);
        test->received_requests_cv.wait(lk, [test]()
          {
            return test->received_requests.back()->requested_mode.value == DoorMode::MODE_CLOSED;
          });
      }
    }
  }
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter
