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
#include <rmf_websocket/BroadcastServer.hpp>

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
  : public std::enable_shared_from_this<MockQuietDispenser>
{
public:

  static std::shared_ptr<MockQuietDispenser> make(
    std::shared_ptr<rclcpp::Node> node,
    std::string name)
  {
    auto dispenser = std::shared_ptr<MockQuietDispenser>(
      new MockQuietDispenser(std::move(node), std::move(name)));

    dispenser->_request_sub =
      dispenser->_node->create_subscription<DispenserRequest>(
      rmf_fleet_adapter::DispenserRequestTopicName,
      rclcpp::SystemDefaultsQoS(),
      [me = dispenser->weak_from_this()](DispenserRequest::SharedPtr msg)
      {
        if (const auto self = me.lock())
          self->_process_request(*msg);
      });

    dispenser->_result_pub =
      dispenser->_node->create_publisher<DispenserResult>(
      rmf_fleet_adapter::DispenserResultTopicName,
      rclcpp::SystemDefaultsQoS());

    return dispenser;
  }

  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

  std::promise<bool> success_promise;

private:

  MockQuietDispenser(
    std::shared_ptr<rclcpp::Node> node,
    std::string name)
  : _node(std::move(node)),
    _name(std::move(name))
  {
    // Initialized in make()
  }

  std::shared_ptr<rclcpp::Node> _node;
  std::string _name;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;
  std::unordered_map<std::string, bool> _task_complete_map;
  rclcpp::TimerBase::SharedPtr _timer;
  bool _fulfilled_promise = false;

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
        1ms, [me = weak_from_this(), msg]()
        {
          const auto self = me.lock();
          if (!self)
            return;

          self->_timer.reset();
          self->_task_complete_map[msg.request_guid] = true;

          DispenserResult result;
          result.time = self->_node->now();
          result.status = DispenserResult::SUCCESS;
          result.source_guid = self->_name;
          result.request_guid = msg.request_guid;
          self->_result_pub->publish(result);

          if (!self->_fulfilled_promise)
          {
            self->_fulfilled_promise = true;
            self->success_promise.set_value(true);
          }
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
class MockFlakyIngestor : public std::enable_shared_from_this<MockFlakyIngestor>
{
public:

  using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
  using IngestorState = rmf_ingestor_msgs::msg::IngestorState;

  static std::shared_ptr<MockFlakyIngestor> make(
    std::shared_ptr<rclcpp::Node> node,
    std::string name)
  {
    auto ingestor = std::shared_ptr<MockFlakyIngestor>(
      new MockFlakyIngestor(std::move(node), std::move(name)));

    ingestor->_request_sub =
      ingestor->_node->create_subscription<IngestorRequest>(
      rmf_fleet_adapter::IngestorRequestTopicName,
      rclcpp::SystemDefaultsQoS(),
      [me = ingestor->weak_from_this()](IngestorRequest::SharedPtr msg)
      {
        if (const auto self = me.lock())
          self->_process_request(*msg);
      });

    ingestor->_state_pub = ingestor->_node->create_publisher<IngestorState>(
      rmf_fleet_adapter::IngestorStateTopicName,
      rclcpp::SystemDefaultsQoS());

    using namespace std::chrono_literals;
    ingestor->_timer = ingestor->_node->create_wall_timer(
      1ms, [me = ingestor->weak_from_this()]()
      {
        const auto self = me.lock();
        if (!self)
          return;

        IngestorState msg;
        msg.guid = self->_name;

        if (self->_request_queue.empty())
          msg.mode = IngestorState::IDLE;
        else
          msg.mode = IngestorState::BUSY;

        msg.time = self->_node->now();
        msg.seconds_remaining = 0.1;

        for (auto& r : self->_request_queue)
        {
          msg.request_guid_queue.push_back(r.request.request_guid);
          ++r.publish_count;
        }

        const std::size_t initial_count = self->_request_queue.size();

        self->_request_queue.erase(std::remove_if(
          self->_request_queue.begin(), self->_request_queue.end(),
          [](const auto& r)
          {
            return r.publish_count > 2;
          }), self->_request_queue.end());

        if (self->_request_queue.size() < initial_count)
        {
          if (!self->_fulfilled_promise)
          {
            self->_fulfilled_promise = true;
            self->success_promise.set_value(true);
          }
        }

        self->_state_pub->publish(msg);
      });

    return ingestor;
  }

  std::promise<bool> success_promise;

private:

  MockFlakyIngestor(
    std::shared_ptr<rclcpp::Node> node,
    std::string name)
  : _node(std::move(node)),
    _name(std::move(name))
  {
    // Further initialization is done in make()
  }

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
  graph.add_waypoint(test_map_name, {0.0, -10.0}).set_charger(true); // 0
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
  const std::string quiet_dispenser_name = "quiet";
  const std::string flaky_ingestor_name = "flaky";

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
  bool fulfilled_promise = false;
  auto completed_future = completed_promise.get_future();
  std::mutex cb_mutex;

  /// Mock Task State observer server, This checks the task_state
  /// of the targeted task id, by listening to the task_state_update
  /// from the websocket connection
  /* *INDENT-OFF* */
  using WebsocketServer = rmf_websocket::BroadcastServer;
	const auto ws_server = WebsocketServer::make(
		37878,
    [ &cb_mutex, delivery_id, &completed_promise,
      &at_least_one_incomplete, &fulfilled_promise](
      const nlohmann::json& data)
      {
        std::lock_guard<std::mutex> lock(cb_mutex);
        assert(data.contains("booking"));
        assert(data.contains("status"));
        const auto id = data.at("booking").at("id");
        const auto status = data.at("status");
        std::cout << "[WebSocketServer] id: [" << id
                  << "] ::: json state ::: " << status << std::endl;

        if (id == delivery_id && status == "completed")
        {
          if (!fulfilled_promise)
          {
            fulfilled_promise = true;
            completed_promise.set_value(true);
          }
        }
        else
          at_least_one_incomplete = true;
      },
    WebsocketServer::ApiMsgType::TaskStateUpdate
  );
  /* *INDENT-ON* */

  // provide the same port number as the observer mock server
  const auto fleet = adapter.add_fleet(
    "test_fleet", traits, graph, "ws://localhost:37878");

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

  /// Callback function when a task is requested
  ///   replacement api for deprecated: 'accept_task_requests'
  fleet->consider_delivery_requests(
    [pickup_name, quiet_dispenser_name](
      const nlohmann::json& msg,
      rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      CHECK(msg.at("place") == pickup_name);
      CHECK(msg.at("handler") == quiet_dispenser_name);
      confirm.accept();
    },
    [dropoff_name, flaky_ingestor_name](
      const nlohmann::json& msg,
      rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      CHECK(msg.at("place") == dropoff_name);
      CHECK(msg.at("handler") == flaky_ingestor_name);
      confirm.accept();
    }
  );

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

  auto quiet_dispenser = MockQuietDispenser::make(
    adapter.node(), quiet_dispenser_name);
  auto quiet_future = quiet_dispenser->success_promise.get_future();

  auto flaky_ingestor = MockFlakyIngestor::make(
    adapter.node(), flaky_ingestor_name);
  auto flaky_future = flaky_ingestor->success_promise.get_future();

  adapter.start();
  ws_server->start();

  // Note: wait for task_manager to start, else TM will be suspicously "empty"
  std::this_thread::sleep_for(1s);

  std::cout << "start to dispatch" << std::endl;

  // Dispatch Delivery Task
  nlohmann::json request;
  request["category"] = "delivery";

  auto& desc = request["description"];
  auto& pickup = desc["pickup"];
  pickup["place"] = pickup_name;
  pickup["handler"] = quiet_dispenser_name;
  pickup["payload"] = std::vector<nlohmann::json>();
  auto& dropoff = desc["dropoff"];
  dropoff["place"] = dropoff_name;
  dropoff["handler"] = flaky_ingestor_name;
  dropoff["payload"] = std::vector<nlohmann::json>();
  std::cout << request << std::endl;
  adapter.dispatch_task(delivery_id, request);

  std::cout << "end dispatch" << std::endl;

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
  std::lock_guard<std::mutex> lock(cb_mutex);
  CHECK(at_least_one_incomplete);

  ws_server->stop();
  adapter.stop();
}
