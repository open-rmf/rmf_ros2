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

#include <phases/ReadyToCharge.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

using rmf_charger_msgs::msg::ChargerState;
using rmf_charger_msgs::msg::ChargerRequest;

SCENARIO_METHOD(MockAdapterFixture, "Charger Request Phase", "[phases]")
{
  std::mutex m;
  std::condition_variable received_requests_cv;
  std::list<ChargerRequest> received_requests;


  const auto info = add_robot();
  const auto& context = info.context;

  auto charger_state_pub = data->adapter->node()->create_publisher<ChargerState>(
    ChargerStateTopicName, 10);

  auto request_guid = "test/charger";
  auto pending_phase = ReadyToCharge::make(
    context,
    request_guid,
    "charger1"
  );
  auto active_phase = pending_phase->begin();


  /*WHEN("it is started")
  {
    auto rcl_subscription = data->adapter->node()->create_subscription<ChargerRequest>(
    ChargerRequestTopicName,
    10,
    [&](ChargerRequest::UniquePtr ingestor_request)
    {
      std::cout << "received requests" << std::endl;
      std::unique_lock<std::mutex> lk(m);
      received_requests.emplace_back(*ingestor_request);
      received_requests_cv.notify_all();
    });

    THEN("it should continuously request a response")
    {
      std::unique_lock<std::mutex> lk(m);
      received_requests_cv.wait(lk, [&]() {
        return received_requests.size() >= 3;
     });
    }
  }*/

  WHEN("a response is sent")
  {
    auto rcl_subscription =
      data->adapter->node()->create_subscription<ChargerRequest>(
        ChargerRequestTopicName,
        10,
        [&](ChargerRequest::UniquePtr request)
        {
          ChargerState state;
          state.state = ChargerState::CHARGER_ASSIGNED;
          state.robot_name = request->robot_name;
          state.robot_fleet = request->fleet_name; 
          state.request_id = request->request_id;
          state.charger_name = request->charger_name;
          charger_state_pub->publish(state);
        });

    auto active_phase = pending_phase->begin();
    std::condition_variable status_updates_cv;
    std::list<Task::StatusMsg> status_updates;
    auto sub = active_phase->observe().subscribe(
      [&](const auto& status)
      {
        std::unique_lock<std::mutex> lk(m);
        status_updates.emplace_back(status);
        if(status_updates.begin()->state == Task::StatusMsg::STATE_COMPLETED)
        status_updates_cv.notify_all();
      });

    THEN("it should send a OK ")
    {
      std::unique_lock<std::mutex> lk(m);
      status_updates_cv.wait(lk, [&]() { return !status_updates.empty(); });
      REQUIRE(status_updates.begin()->state == Task::StatusMsg::STATE_COMPLETED);
    }
    sub.unsubscribe();
  }
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter
