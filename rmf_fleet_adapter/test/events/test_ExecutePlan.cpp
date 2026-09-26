/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <events/ExecutePlan.hpp>

#include <agv/internal_FleetUpdateHandle.hpp>
#include <agv/internal_RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include "../mock/MockRobotCommand.hpp"

#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace events {
namespace test {

namespace {

const std::string lift_name = "test_lift";
const std::string departure_map = "L1";
const std::string destination_map = "L2";

// A lift specific counterpart of phases/MockAdapterFixture: that fixture's
// graph has a single map and no lift, so it cannot reach the lift insertion
// in ExecutePlan::make.
//
// A minimal two level graph where a single lift joins the two levels:
//
//   L1:  0 (outside) ---- 1 (inside the lift)
//                         |  LiftMove / LiftDoorOpen (upward only)
//   L2:  3 (outside) ---- 2 (inside the lift)
//
struct LiftFixture
{
  std::shared_ptr<rclcpp::Context> rcl_context;
  std::shared_ptr<agv::test::MockAdapter> adapter;
  std::shared_ptr<agv::FleetUpdateHandle> fleet;
  std::shared_ptr<agv::Node> node;
  rmf_traffic::agv::Graph graph;

  std::shared_ptr<agv::RobotContext> context;
  std::shared_ptr<rmf_fleet_adapter_test::MockRobotCommand> command;

  static std::size_t node_counter;

  LiftFixture()
  {
    rcl_context = std::make_shared<rclcpp::Context>();
    rcl_context->init(0, nullptr);

    adapter = std::make_shared<agv::test::MockAdapter>(
      "test_execute_plan_" + std::to_string(node_counter++),
      rclcpp::NodeOptions().context(rcl_context));

    graph.set_known_lift(
      rmf_traffic::agv::Graph::LiftProperties(
        lift_name, {0.0, 0.0}, 0.0, {2.0, 2.0}));
    const auto lift = graph.find_known_lift(lift_name);

    graph.add_waypoint(departure_map, {5.0, 0.0}).set_charger(true); // 0
    graph.add_waypoint(departure_map, {0.0, 0.0}).set_in_lift(lift); // 1
    graph.add_waypoint(destination_map, {0.0, 0.0}).set_in_lift(lift); // 2
    graph.add_waypoint(destination_map, {5.0, 0.0}); // 3

    // The lanes are shaped the way rmf_traffic_ros2::convert_Graph builds
    // them for a lift: entering the car begins a session and waits for the
    // doors, riding the car moves it and waits for the doors on the arrival
    // floor, and leaving the car ends the session. Only the upward ride is
    // modelled, because a downward lane would give the planner a zero length
    // cycle between the two coincident car waypoints.
    using Lane = rmf_traffic::agv::Graph::Lane;
    using Event = Lane::Event;
    const auto d = std::chrono::seconds(4);
    const auto no_event = rmf_utils::clone_ptr<Event>();

    graph.add_lane(
      {0, Event::make(Lane::LiftSessionBegin(lift_name, departure_map, d))},
      {1, Event::make(Lane::LiftDoorOpen(lift_name, departure_map, d))});
    graph.add_lane(
      {1, no_event},
      {0, Event::make(Lane::LiftSessionEnd(lift_name, departure_map, d))});
    graph.add_lane(
      {3, Event::make(Lane::LiftSessionBegin(lift_name, destination_map, d))},
      {2, Event::make(Lane::LiftDoorOpen(lift_name, destination_map, d))});
    graph.add_lane(
      {2, no_event},
      {3, Event::make(Lane::LiftSessionEnd(lift_name, destination_map, d))});
    graph.add_lane(
      {1, Event::make(Lane::LiftMove(lift_name, destination_map, d))},
      {2, Event::make(Lane::LiftDoorOpen(lift_name, destination_map, d))});

    const rmf_traffic::Profile profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(0.1)
    };

    const rmf_traffic::agv::VehicleTraits traits{
      {0.7, 0.3}, {1.0, 0.45}, profile};

    fleet = adapter->add_fleet("test_fleet", traits, graph);
    node = agv::FleetUpdateHandle::Implementation::get(*fleet).node;
    adapter->start();

    const auto now = rmf_traffic_ros2::convert(adapter->node()->now());
    command = std::make_shared<rmf_fleet_adapter_test::MockRobotCommand>(
      node, graph);

    std::promise<void> robot_added;
    fleet->add_robot(
      command, "test_robot", profile,
      rmf_traffic::agv::Plan::StartSet{{now, 0, 0.0}},
      [this, &robot_added](agv::RobotUpdateHandlePtr updater)
      {
        context = agv::RobotUpdateHandle::Implementation::get(*updater)
        .context.lock();
        command->updater = updater;
        robot_added.set_value();
      });

    robot_added.get_future().wait();
  }

  ~LiftFixture()
  {
    std::weak_ptr<rclcpp::Node> weak_node = node;
    node->stop();
    context.reset();
    command.reset();
    fleet.reset();
    node.reset();
    adapter.reset();

    std::size_t wait_count = 0;
    while (const auto n = weak_node.lock())
    {
      ++wait_count;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (wait_count > 200)
      {
        std::cerr << "Node is not dying during test teardown" << std::endl;
        std::terminate();
      }
    }
  }

  // Plans from `start` to `goal` and returns the names of the events that
  // ExecutePlan::make builds for that plan.
  std::vector<std::string> event_names(
    const rmf_traffic::agv::Plan::Start& start,
    std::size_t goal) const;

  template<typename T>
  T schedule_and_wait(std::function<T()> job) const
  {
    std::promise<T> promise;
    auto future = promise.get_future();
    context->worker().schedule(
      [&promise, job = std::move(job)](const auto&)
      {
        promise.set_value(job());
      });

    return future.get();
  }
};

std::size_t LiftFixture::node_counter = 0;

//==============================================================================
void collect_event_names(
  const rmf_task::Event::ConstStatePtr& state,
  std::vector<std::string>& names)
{
  rmf_task::VersionedString::Reader reader;
  names.push_back(*reader.read(state->name()));
  names.push_back(*reader.read(state->detail()));
  for (const auto& dep : state->dependencies())
    collect_event_names(dep, names);
}

//==============================================================================
bool any_name_contains(
  const std::vector<std::string>& names,
  const std::string& snippet)
{
  for (const auto& name : names)
  {
    if (name.find(snippet) != std::string::npos)
      return true;
  }

  return false;
}

//==============================================================================
std::vector<std::string> LiftFixture::event_names(
  const rmf_traffic::agv::Plan::Start& start,
  std::size_t goal) const
{
  const auto planner = context->planner();
  REQUIRE(planner);

  const auto plan = planner->plan(start, rmf_traffic::agv::Plan::Goal(goal));
  REQUIRE(plan.success());
  REQUIRE_FALSE(plan->get_waypoints().front().graph_index().has_value());

  return schedule_and_wait<std::vector<std::string>>(
    [&]()
    {
      auto state = rmf_task::events::SimpleEventState::make(
        0, "test", "", rmf_task::Event::Status::Standby, {},
        context->clock());

      const auto execution = ExecutePlan::make(
        context, context->itinerary().assign_plan_id(), *plan,
        rmf_traffic::agv::Plan::Goal(goal), plan->get_itinerary(),
        rmf_task::Event::AssignID::make(), state, []() {}, []() {},
        std::nullopt);

      std::vector<std::string> result;
      if (execution.has_value())
        collect_event_names(execution->sequence->state(), result);

      return result;
    });
}

} // anonymous namespace

//==============================================================================
SCENARIO_METHOD(LiftFixture, "execute plan lift summon after replan",
  "[events]")
{
  GIVEN("the robot holds a lift session requested from inside")
  {
    const auto session = schedule_and_wait<std::shared_ptr<void>>(
      [&]()
      {
        return context->set_lift_destination(
          lift_name, destination_map, true);
      });
    REQUIRE(session);

    // The replanned start is inside the car, off the graph, and still on the
    // map of the floor the robot boarded from.
    const auto now = rmf_traffic_ros2::convert(adapter->node()->now());
    const rmf_traffic::agv::Plan::Start start(
      now, 1, 0.0, Eigen::Vector2d(0.3, 0.0));

    WHEN("the new plan continues to the floor the lift is riding to")
    {
      const auto names = event_names(start, 3);
      REQUIRE_FALSE(names.empty());

      THEN("every summon targets the destination floor")
      {
        CHECK(any_name_contains(
            names, "lift [" + lift_name + "] to [" + destination_map + "]"));
        CHECK_FALSE(any_name_contains(
            names, "lift [" + lift_name + "] to [" + departure_map + "]"));
      }
    }

    WHEN("the new plan returns to the floor the robot boarded from")
    {
      const auto names = event_names(start, 0);
      REQUIRE_FALSE(names.empty());

      THEN("the inserted summon follows the session, not the reported map")
      {
        CHECK(any_name_contains(
            names, "lift [" + lift_name + "] to [" + destination_map + "]"));
        CHECK_FALSE(any_name_contains(
            names, "lift [" + lift_name + "] to [" + departure_map + "]"));
      }
    }
  }
}

} // namespace test
} // namespace events
} // namespace rmf_fleet_adapter
