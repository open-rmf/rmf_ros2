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

#include "../mock/MockRobotCommand.hpp"

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>

#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <rmf_utils/catch.hpp>

#include "../thread_cooldown.hpp"

#include <rmf_task_msgs/msg/task_summary.hpp>

//==============================================================================
/// This mock dispenser will not publish any states; it will only publish a
/// successful result and nothing else.
class MockQuietDispenser
{
public:

  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

  MockQuietDispenser(
    std::shared_ptr<rclcpp::Node> node,
    std::string name)
  : _node(std::move(node)),
    _name(std::move(name))
  {
    _request_sub = _node->create_subscription<DispenserRequest>(
      rmf_fleet_adapter::DispenserRequestTopicName,
      rclcpp::SystemDefaultsQoS(),
      [this](DispenserRequest::SharedPtr msg)
      {
        _process_request(*msg);
      });

    _result_pub = _node->create_publisher<DispenserResult>(
      rmf_fleet_adapter::DispenserResultTopicName,
      rclcpp::SystemDefaultsQoS());
  }

  std::promise<bool> success_promise;

private:
  std::shared_ptr<rclcpp::Node> _node;
  std::string _name;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;
  std::unordered_map<std::string, bool> _task_complete_map;
  rclcpp::TimerBase::SharedPtr _timer;

  void _process_request(const DispenserRequest& msg)
  {
    if (msg.target_guid != _name)
      return;

    const auto insertion = _task_complete_map.insert({msg.request_guid, false});
    uint8_t status = DispenserResult::ACKNOWLEDGED;
    if (insertion.first->second)
    {
      status = DispenserResult::SUCCESS;
    }
    else
    {
      using namespace std::chrono_literals;
      _timer = _node->create_wall_timer(
        10ms, [this, msg]()
        {
          _timer.reset();
          _task_complete_map[msg.request_guid] = true;

          DispenserResult result;
          result.time = _node->now();
          result.status = DispenserResult::SUCCESS;
          result.source_guid = _name;
          result.request_guid = msg.request_guid;
          _result_pub->publish(result);

          success_promise.set_value(true);
        });
    }

    DispenserResult result;
    result.time = _node->now();
    result.status = status;
    result.source_guid = _name;
    result.request_guid = msg.request_guid;
    _result_pub->publish(result);
  }

};

//==============================================================================
/// This mock ingestor will not publish any results; it will only publish
/// states. This is representative of network issues where a result might not
/// actually arrive, but the state heartbeats can still get through.
class MockFlakyIngestor
{
public:

  using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
  using IngestorState = rmf_ingestor_msgs::msg::IngestorState;

  MockFlakyIngestor(
    std::shared_ptr<rclcpp::Node> node,
    std::string name)
  : _node(std::move(node)),
    _name(std::move(name))
  {
    _request_sub = _node->create_subscription<IngestorRequest>(
      rmf_fleet_adapter::IngestorRequestTopicName,
      rclcpp::SystemDefaultsQoS(),
      [this](IngestorRequest::SharedPtr msg)
      {
        _process_request(*msg);
      });

    _state_pub = _node->create_publisher<IngestorState>(
      rmf_fleet_adapter::IngestorStateTopicName,
      rclcpp::SystemDefaultsQoS());

    using namespace std::chrono_literals;
    _timer = _node->create_wall_timer(
      100ms, [this]()
      {
        IngestorState msg;
        msg.guid = _name;

        if (_request_queue.empty())
          msg.mode = IngestorState::IDLE;
        else
          msg.mode = IngestorState::BUSY;

        msg.time = _node->now();
        msg.seconds_remaining = 0.1;

        for (auto& r : _request_queue)
        {
          msg.request_guid_queue.push_back(r.request.request_guid);
          ++r.publish_count;
        }

        const std::size_t initial_count = _request_queue.size();

        _request_queue.erase(std::remove_if(
          _request_queue.begin(), _request_queue.end(),
          [](const auto& r)
          {
            return r.publish_count > 2;
          }), _request_queue.end());

        if (_request_queue.size() < initial_count)
        {
          if (!_fulfilled_promise)
          {
            _fulfilled_promise = true;
            success_promise.set_value(true);
          }
        }

        _state_pub->publish(msg);
      });
  }

  std::promise<bool> success_promise;

private:

  struct RequestEntry
  {
    IngestorRequest request;
    std::size_t publish_count = 0;
  };

  std::shared_ptr<rclcpp::Node> _node;
  std::string _name;
  rclcpp::Subscription<IngestorRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<IngestorState>::SharedPtr _state_pub;
  std::vector<RequestEntry> _request_queue;
  rclcpp::TimerBase::SharedPtr _timer;
  bool _fulfilled_promise = false;

  void _process_request(const IngestorRequest& msg)
  {
    if (msg.target_guid != _name)
    {
      return;
    }

    _request_queue.push_back({msg, 0});
  }
};

//==============================================================================
SCENARIO("Test Delivery")
{
  rmf_fleet_adapter_test::thread_cooldown = true;
  using namespace std::chrono_literals;

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                   10
   *                   |
   *                  (D)
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6--(D)--7
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

  auto add_dock_lane = [&](
    const std::size_t w0,
    const std::size_t w1,
    std::string dock_name)
    {
      using Lane = rmf_traffic::agv::Graph::Lane;
      graph.add_lane({w0, Lane::Event::make(Lane::Dock(dock_name, 10s))}, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);  // 0   1
  add_bidir_lane(1, 2);  // 2   3
  add_bidir_lane(1, 5);  // 4   5
  add_bidir_lane(2, 6);  // 6   7
  add_bidir_lane(3, 4);  // 8   9
  add_bidir_lane(4, 5);  // 10 11
  add_bidir_lane(5, 6);  // 12 13
  add_dock_lane(6, 7, "A");  // 14 15
  add_bidir_lane(5, 8);  // 16 17
  add_bidir_lane(6, 9);  // 18 19
  add_bidir_lane(8, 9);  // 20 21
  add_dock_lane(8, 10, "B"); // 22 23

  const std::string pickup_name = "pickup";
  REQUIRE(graph.add_key(pickup_name, 7));

  const std::string dropoff_name = "dropoff";
  REQUIRE(graph.add_key(dropoff_name, 10));

  const std::string delivery_id = "test_delivery";

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  auto rcl_context = std::make_shared<rclcpp::Context>();
  rcl_context->init(0, nullptr);
  rmf_fleet_adapter::agv::test::MockAdapter adapter(
    "test_Delivery", rclcpp::NodeOptions().context(rcl_context));

  std::promise<bool> completed_promise;
  bool at_least_one_incomplete = false;
  auto completed_future = completed_promise.get_future();
  const auto task_sub = adapter.node()->create_subscription<
    rmf_task_msgs::msg::TaskSummary>(
    rmf_fleet_adapter::TaskSummaryTopicName, rclcpp::SystemDefaultsQoS(),
    [&completed_promise, &at_least_one_incomplete](
      const rmf_task_msgs::msg::TaskSummary::SharedPtr msg)
    {
      if (msg->STATE_COMPLETED == msg->state)
        completed_promise.set_value(true);
      else
        at_least_one_incomplete = true;
    });

  const auto fleet = adapter.add_fleet("test_fleet", traits, graph);

  // Configure default battery param
  using BatterySystem = rmf_battery::agv::BatterySystem;
  using PowerSystem = rmf_battery::agv::PowerSystem;
  using MechanicalSystem = rmf_battery::agv::MechanicalSystem;
  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;

  auto battery_system = std::make_shared<BatterySystem>(
    *BatterySystem::make(24.0, 40.0, 8.8));

  auto mechanical_system = MechanicalSystem::make(70.0, 40.0, 0.22);
  auto motion_sink = std::make_shared<SimpleMotionPowerSink>(
    *battery_system, *mechanical_system);

  auto ambient_power_system = PowerSystem::make(20.0);
  auto ambient_sink = std::make_shared<SimpleDevicePowerSink>(
    *battery_system, *ambient_power_system);

  auto tool_power_system = PowerSystem::make(10.0);
  auto tool_sink = std::make_shared<SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  fleet->set_task_planner_params(
    battery_system, motion_sink, ambient_sink, tool_sink, 0.2, 1.0, false);

  fleet->accept_task_requests(
    [&delivery_id](const rmf_task_msgs::msg::TaskProfile& task)
    {
      // Accept all delivery task requests
      CHECK(task.description.task_type.type ==
      rmf_task_msgs::msg::TaskType::TYPE_DELIVERY);
      CHECK(task.task_id == delivery_id);
      return true;
    });

  const auto now = rmf_traffic_ros2::convert(adapter.node()->now());
  const rmf_traffic::agv::Plan::StartSet starts = {{now, 0, 0.0}};
  auto robot_cmd = std::make_shared<
    rmf_fleet_adapter_test::MockRobotCommand>(adapter.node(), graph);

  fleet->add_robot(
    robot_cmd, "T0", profile, starts,
    [&robot_cmd](rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
    {
      // assume battery soc is full
      updater->update_battery_soc(1.0);
      robot_cmd->updater = std::move(updater);
    });

  const std::string quiet_dispenser_name = "quiet";
  auto quiet_dispenser = MockQuietDispenser(
    adapter.node(), quiet_dispenser_name);
  auto quiet_future = quiet_dispenser.success_promise.get_future();

  const std::string flaky_ingestor_name = "flaky";
  auto flaky_ingestor = MockFlakyIngestor(
    adapter.node(), flaky_ingestor_name);
  auto flaky_future = flaky_ingestor.success_promise.get_future();

  adapter.start();

  // Note: wait for task_manager to start, else TM will be suspicously "empty"
  std::this_thread::sleep_for(1s);

  // Dispatch Delivery Task
  rmf_task_msgs::msg::TaskProfile task_profile;
  task_profile.task_id = delivery_id;
  task_profile.description.start_time = adapter.node()->now();
  task_profile.description.task_type.type =
    rmf_task_msgs::msg::TaskType::TYPE_DELIVERY;

  rmf_task_msgs::msg::Delivery delivery;
  delivery.pickup_place_name = pickup_name;
  delivery.pickup_dispenser = quiet_dispenser_name;
  delivery.dropoff_place_name = dropoff_name;
  delivery.dropoff_ingestor = flaky_ingestor_name;

  task_profile.description.delivery = delivery;
  adapter.dispatch_task(task_profile);

  const auto quiet_status = quiet_future.wait_for(15s);
  REQUIRE(quiet_status == std::future_status::ready);
  REQUIRE(quiet_future.get());

  const auto flaky_status = flaky_future.wait_for(15s);
  REQUIRE(flaky_status == std::future_status::ready);
  REQUIRE(flaky_future.get());

  const auto& visits = robot_cmd->visited_wps();
  CHECK(visits.size() == 6);
  CHECK(visits.count(0));
  CHECK(visits.count(5));
  CHECK(visits.count(6));
  CHECK(visits.count(7));
  CHECK(visits.count(8));
  CHECK(visits.count(10));

  const auto completed_status = completed_future.wait_for(15s);
  REQUIRE(completed_status == std::future_status::ready);
  REQUIRE(completed_future.get());
  CHECK(at_least_one_incomplete);

  adapter.stop();
}
