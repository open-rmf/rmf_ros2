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

#include <services/FindEmergencyPullover.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/DetectConflict.hpp>

#include <services/FindEmergencyPullover.hpp>

#include <rmf_utils/catch.hpp>

#include "../thread_cooldown.hpp"

SCENARIO("Emergency Pullover")
{
  rmf_fleet_adapter_test::thread_cooldown = true;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  auto p0 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 1",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  auto p1 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 2",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0}); // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}); // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}).set_parking_spot(true); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10
  graph.add_waypoint(test_map_name, {5.0, 10.0}).set_parking_spot(true); // 11

  /*
   *                  10     11(P)
   *                   |      |
   *                   |      |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6------7(P)
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(1, 5);
  add_bidir_lane(2, 6);
  add_bidir_lane(3, 4);
  add_bidir_lane(4, 5);
  add_bidir_lane(5, 6);
  add_bidir_lane(6, 7);
  add_bidir_lane(5, 8);
  add_bidir_lane(6, 9);
  add_bidir_lane(8, 9);
  add_bidir_lane(8, 10);
  add_bidir_lane(9, 11);

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  rmf_traffic::agv::Planner::Configuration configuration{graph, traits};
  const auto planner = std::make_shared<rmf_traffic::agv::Planner>(
    configuration,
    rmf_traffic::agv::Planner::Options{nullptr}
  );

  const auto now = std::chrono::steady_clock::now();
  using namespace std::chrono_literals;

  WHEN("Parking two vehicles with no obstacles")
  {
    const auto start_0 = rmf_traffic::agv::Plan::Start(now, 4, 0.0);
    const auto start_1 = rmf_traffic::agv::Plan::Start(now, 3, 0.0);

    auto pullover_service = std::make_shared<
      rmf_fleet_adapter::services::FindEmergencyPullover>(
      planner, rmf_traffic::agv::Plan::StartSet({start_0}),
      database->snapshot(), p0.id(),
      std::make_shared<rmf_traffic::Profile>(p0.description().profile()));

    std::promise<rmf_traffic::agv::Plan::Result> result_0_promise;
    auto result_0_future = result_0_promise.get_future();
    auto pullover_sub =
      rmf_rxcpp::make_job<
      rmf_fleet_adapter::services::FindEmergencyPullover::Result>(
      pullover_service)
      .observe_on(rxcpp::observe_on_event_loop())
      .subscribe(
      [&result_0_promise](const auto& result)
      {
        result_0_promise.set_value(result);
      });

    const auto status_0 = result_0_future.wait_for(10s);
    REQUIRE(std::future_status::ready == status_0);
    const auto result_0 = result_0_future.get();
    REQUIRE(result_0.success());

    auto itinerary_0 = result_0->get_itinerary();
    rmf_traffic::Trajectory end_0;
    end_0.insert(itinerary_0.back().trajectory().back());
    end_0.insert(
      end_0.back().time() + 1h,
      end_0.back().position(),
      Eigen::Vector3d::Zero());
    itinerary_0.push_back({itinerary_0.back().map(), end_0});


    // First we will test that a conflict happens when p0 does not put its
    // itinerary in the database. The conflict happens because both vehicles
    // want to park at waypoint 7 in the absence of any other conflicts.
    pullover_service = std::make_shared<
      rmf_fleet_adapter::services::FindEmergencyPullover>(
      planner, rmf_traffic::agv::Plan::StartSet({start_1}),
      database->snapshot(), p1.id(),
      std::make_shared<rmf_traffic::Profile>(p1.description().profile()));

    std::promise<rmf_traffic::agv::Plan::Result> pre_result_1_promise;
    auto pre_result_1_future = pre_result_1_promise.get_future();
    pullover_sub =
      rmf_rxcpp::make_job<
      rmf_fleet_adapter::services::FindEmergencyPullover::Result>(
      pullover_service)
      .observe_on(rxcpp::observe_on_event_loop())
      .subscribe(
      [&pre_result_1_promise](const auto& result)
      {
        pre_result_1_promise.set_value(result);
      });

    const auto pre_status_1 = pre_result_1_future.wait_for(1s);
    REQUIRE(std::future_status::ready == pre_status_1);
    const auto pre_result_1 = pre_result_1_future.get();
    REQUIRE(pre_result_1.success());

    bool at_least_one_conflict = false;
    for (const auto& t0 : itinerary_0)
    {
      for (const auto& t1 : pre_result_1->get_itinerary())
      {
        at_least_one_conflict |= rmf_traffic::DetectConflict::between(
          p0.description().profile(), t0.trajectory(), nullptr,
          p1.description().profile(), t1.trajectory(), nullptr).has_value();
      }
    }

    CHECK(at_least_one_conflict);

    // Now we perform FindPath again for p1, but with p0's itinerary
    // in the schedule
    p0.set(p0.assign_plan_id(), itinerary_0);

    pullover_service = std::make_shared<
      rmf_fleet_adapter::services::FindEmergencyPullover>(
      planner, rmf_traffic::agv::Plan::StartSet({start_1}),
      database->snapshot(), p1.id(),
      std::make_shared<rmf_traffic::Profile>(p1.description().profile()));

    std::promise<rmf_traffic::agv::Plan::Result> result_1_promise;
    auto result_1_future = result_1_promise.get_future();
    pullover_sub =
      rmf_rxcpp::make_job<
      rmf_fleet_adapter::services::FindEmergencyPullover::Result>(
      pullover_service)
      .observe_on(rxcpp::observe_on_event_loop())
      .subscribe(
      [&result_1_promise](const auto& result)
      {
        result_1_promise.set_value(result);
      });

    const auto status_1 = result_1_future.wait_for(1s);
    REQUIRE(std::future_status::ready == status_1);
    const auto result_1 = result_1_future.get();
    REQUIRE(result_1.success());

    for (const auto& t0 : itinerary_0)
    {
      for (const auto& t1 : result_1->get_itinerary())
      {
        CHECK_FALSE(rmf_traffic::DetectConflict::between(
            p0.description().profile(), t0.trajectory(), nullptr,
            p1.description().profile(), t1.trajectory(), nullptr));
      }
    }
  }

  WHEN("Parking a vehicle with an obstacle in the way")
  {
    const auto l0 = graph.get_waypoint(5).get_location();

    const auto start_1 = rmf_traffic::agv::Plan::Start(now, 3, 0.0);

    rmf_traffic::Trajectory blocking_traj;
    blocking_traj.insert(
      now,
      {l0[0], l0[1], 0.0},
      Eigen::Vector3d::Zero());
    blocking_traj.insert(
      now + 1h,
      {l0[0], l0[1], 0.0},
      Eigen::Vector3d::Zero());

    rmf_traffic::Route blocking_route(
      graph.get_waypoint(5).get_map_name(), blocking_traj);

    p0.set(p0.assign_plan_id(), {blocking_route});

    auto pullover_service = std::make_shared<
      rmf_fleet_adapter::services::FindEmergencyPullover>(
      planner, rmf_traffic::agv::Plan::StartSet({start_1}),
      database->snapshot(), p1.id(),
      std::make_shared<rmf_traffic::Profile>(p1.description().profile()));

    std::promise<rmf_traffic::agv::Plan::Result> result_1_promise;
    auto result_1_future = result_1_promise.get_future();
    auto pullover_sub =
      rmf_rxcpp::make_job<
      rmf_fleet_adapter::services::FindEmergencyPullover::Result>(
      pullover_service)
      .observe_on(rxcpp::observe_on_event_loop())
      .subscribe(
      [&result_1_promise](const auto& result)
      {
        result_1_promise.set_value(result);
      });

    const auto status_1 = result_1_future.wait_for(10s);
    REQUIRE(std::future_status::ready == status_1);
    const auto result_1 = result_1_future.get();
    REQUIRE(result_1.success());

    bool at_least_one_conflict = false;
    for (const auto& t0 : p0.itinerary())
    {
      for (const auto& t1 : result_1->get_itinerary())
      {
        at_least_one_conflict |= rmf_traffic::DetectConflict::between(
          p0.description().profile(), t0.trajectory(), nullptr,
          p1.description().profile(), t1.trajectory(), nullptr).has_value();
      }
    }

    CHECK(at_least_one_conflict);
  }
}
