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

#include <phases/RequestLift.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

using rmf_lift_msgs::msg::LiftState;
using rmf_lift_msgs::msg::LiftRequest;

namespace {
struct TestData
{
  std::mutex m;
  std::condition_variable received_requests_cv;
  std::list<LiftRequest> received_requests;
  std::string session_id;

  std::condition_variable status_updates_cv;
  std::list<Task::StatusMsg> status_updates;
};
} // anonymous namespace

SCENARIO_METHOD(MockAdapterFixture, "request lift phase", "[phases]")
{
  const auto test = std::make_shared<TestData>();
  auto rcl_subscription = data->ros_node->create_subscription<LiftRequest>(
    AdapterLiftRequestTopicName,
    10,
    [test](LiftRequest::UniquePtr lift_request)
    {
      std::unique_lock<std::mutex> lk(test->m);
      test->session_id = lift_request->session_id;
      test->received_requests.emplace_back(*lift_request);
      test->received_requests_cv.notify_all();
    });

  const auto info = add_robot();
  const auto& context = info.context;

  std::string lift_name = "test_lift";
  std::string destination = "test_floor";
  auto pending_phase = std::make_shared<RequestLift::PendingPhase>(
    context,
    lift_name,
    destination,
    context->now() + std::chrono::seconds(5),
    RequestLift::Located::Outside
  );
  auto active_phase = pending_phase->begin();

  WHEN("it is cancelled before its started")
  {
    THEN("it should not send lift requests")
    {
      bool received_open = false;
      rxcpp::composite_subscription rx_sub;
      auto subscription =
        data->adapter->node()->create_subscription<LiftRequest>(
        AdapterLiftRequestTopicName, 10,
        [&](LiftRequest::UniquePtr lift_request)
        {
          if (lift_request->request_type != LiftRequest::REQUEST_END_SESSION)
            received_open = true;
          else
            rx_sub.unsubscribe();
        });

      auto obs = active_phase->observe();
      active_phase->cancel();

      // TODO(MXG): Put an explicit timeout here so this line doesn't hang
      // forever in the event of a failure.
      obs.as_blocking().subscribe(rx_sub);
      CHECK(!received_open);
    }
  }

  WHEN("it is started")
  {
    rmf_rxcpp::subscription_guard sub = active_phase->observe().subscribe(
      [test](const auto& status)
      {
        std::unique_lock<std::mutex> lk(test->m);
        test->status_updates.emplace_back(status);
        test->status_updates_cv.notify_all();
      });

    THEN("it should send lift request")
    {
      std::unique_lock<std::mutex> lk(test->m);
      if (test->received_requests.empty())
      {
        test->received_requests_cv.wait(lk, [&]()
          {
            return !test->received_requests.empty();
          });
      }
      CHECK(test->received_requests.size() == 1);
      CHECK(test->received_requests.front().destination_floor == destination);
    }

    THEN("it should continuously send lift requests")
    {
      std::unique_lock<std::mutex> lk(test->m);
      test->received_requests_cv.wait(lk, [test]()
        {
          return test->received_requests.size() >= 3;
        });
      for (const auto& lift_request : test->received_requests)
      {
        CHECK(lift_request.destination_floor == destination);
      }
    }

    AND_WHEN("lift is on destination floor")
    {
      auto lift_state_pub = data->ros_node->create_publisher<LiftState>(
        LiftStateTopicName, 10);
      rclcpp::TimerBase::SharedPtr timer = data->node->try_create_wall_timer(
        std::chrono::milliseconds(100),
        [test, node = data->ros_node, lift_name, destination, lift_state_pub]()
        {
          std::unique_lock<std::mutex> lk(test->m);
          LiftState lift_state;
          lift_state.lift_name = lift_name;
          lift_state.lift_time = node->now();
          lift_state.motion_state = LiftState::MOTION_STOPPED;
          lift_state.destination_floor = destination;
          lift_state.current_floor = destination;
          lift_state.session_id = test->session_id;
          lift_state.door_state = LiftState::DOOR_OPEN;
          lift_state.current_mode = LiftState::MODE_AGV;
          lift_state_pub->publish(lift_state);
        });

      THEN("it is completed")
      {
        std::unique_lock<std::mutex> lk(test->m);
        bool completed = test->status_updates_cv.wait_for(
          lk, std::chrono::milliseconds(1000),
          [&]()
          {
            if (test->status_updates.empty())
              return false;
            const auto& state = test->status_updates.back().state;
            return state == Task::StatusMsg::STATE_COMPLETED;
          });
        CHECK(completed);
      }

      timer.reset();
    }

    AND_WHEN("it is cancelled")
    {
      {
        std::unique_lock<std::mutex> lk(test->m);
        test->received_requests_cv.wait(lk, [&]()
          {
            return !test->received_requests.empty();
          });
        active_phase->cancel();
      }

      THEN("it should send END_SESSION request")
      {
        std::unique_lock<std::mutex> lk(test->m);
        test->received_requests_cv.wait(lk, [&]()
          {
            if (test->received_requests.empty())
              return false;
            return test->received_requests.back().request_type == LiftRequest::REQUEST_END_SESSION;
          });
      }
    }
  }
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter
