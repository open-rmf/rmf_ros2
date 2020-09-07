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

SCENARIO_METHOD(MockAdapterFixture, "request lift phase", "[phases]")
{
  std::mutex m;
  std::condition_variable received_requests_cv;
  std::list<LiftRequest> received_requests;
  std::string session_id;
  auto rcl_subscription = ros_node->create_subscription<LiftRequest>(
    AdapterLiftRequestTopicName,
    10,
    [&](LiftRequest::UniquePtr lift_request)
    {
      std::unique_lock<std::mutex> lk(m);
      session_id = lift_request->session_id;
      received_requests.emplace_back(*lift_request);
      received_requests_cv.notify_all();
    });

  const auto info = add_robot();
  const auto& context = info.context;

  std::string lift_name = "test_lift";
  std::string destination = "test_floor";
  auto pending_phase = std::make_shared<RequestLift::PendingPhase>(
    context,
    lift_name,
    destination,
    rmf_traffic::Time(),
    RequestLift::Located::Outside
  );
  auto active_phase = pending_phase->begin();

  WHEN("it is cancelled before its started")
  {
    active_phase->cancel();

    THEN("it should not send lift requests")
    {
      bool received_open = false;
      rxcpp::composite_subscription rx_sub;
      auto subscription = adapter->node()->create_subscription<LiftRequest>(
        AdapterLiftRequestTopicName,
        10,
        [&](LiftRequest::UniquePtr lift_request)
        {
          if (lift_request->request_type != LiftRequest::REQUEST_END_SESSION)
            received_open = true;
          else if (lift_request->request_type == LiftRequest::REQUEST_END_SESSION)
            rx_sub.unsubscribe();
        });
      auto obs = active_phase->observe();
      obs.as_blocking().subscribe(rx_sub);
      CHECK(!received_open);
    }
  }

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

    THEN("it should send lift request")
    {
      std::unique_lock<std::mutex> lk(m);
      if (received_requests.empty())
        received_requests_cv.wait(lk, [&]() { return !received_requests.empty(); });
      CHECK(received_requests.size() == 1);
      CHECK(received_requests.front().destination_floor == destination);
    }

    THEN("it should continuously send lift requests")
    {
      std::unique_lock<std::mutex> lk(m);
      received_requests_cv.wait(lk, [&]() { return received_requests.size() >= 3; });
      for (const auto& lift_request : received_requests)
      {
        CHECK(lift_request.destination_floor == destination);
      }
    }

    AND_WHEN("lift is on destination floor")
    {
      auto lift_state_pub = ros_node->create_publisher<LiftState>(LiftStateTopicName, 10);
      rclcpp::TimerBase::SharedPtr timer;
      std::function<void()> publish_lift_state = [&]()
      {
        std::unique_lock<std::mutex> lk(m);
        LiftState lift_state;
        lift_state.lift_name = lift_name;
        lift_state.lift_time = ros_node->now();
        lift_state.motion_state = LiftState::MOTION_STOPPED;
        lift_state.destination_floor = destination;
        lift_state.current_floor = destination;
        lift_state.session_id = session_id;
        lift_state.door_state = LiftState::DOOR_OPEN;
        lift_state.current_mode = LiftState::MODE_AGV;
        lift_state_pub->publish(lift_state);
        timer = ros_node->create_wall_timer(std::chrono::milliseconds(100), publish_lift_state);
      };
      publish_lift_state();

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

    AND_WHEN("it is cancelled")
    {
      {
        std::unique_lock<std::mutex> lk(m);
        received_requests_cv.wait(lk, [&]()
        {
          return !received_requests.empty();
        });
        active_phase->cancel();
      }

      THEN("it should send END_SESSION request")
      {
        std::unique_lock<std::mutex> lk(m);
        received_requests_cv.wait(lk, [&]()
        {
          return received_requests.back().request_type == LiftRequest::REQUEST_END_SESSION;
        });
      }
    }

    sub.unsubscribe();
  }
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter
