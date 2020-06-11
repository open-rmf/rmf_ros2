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

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>

#include <services/FindPath.hpp>
#include <agv/internal_RobotUpdateHandle.hpp>

#include <rmf_utils/catch.hpp>

#include "../thread_cooldown.hpp"

//==============================================================================
class MockRobotCommand : public rmf_fleet_adapter::agv::RobotCommandHandle
{
public:

  MockRobotCommand(std::shared_ptr<rclcpp::Node> node)
    : _node(std::move(node))
  {
    // Do nothing
  }

  std::shared_ptr<rmf_fleet_adapter::agv::RobotUpdateHandle> updater;

  void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      ArrivalEstimator next_arrival_estimator,
      std::function<void()> path_finished_callback) final
  {
    _current_waypoint_target = 0;
    _timer = _node->create_wall_timer(
          std::chrono::milliseconds(10),
          [this,
           waypoints,
           next_arrival_estimator = std::move(next_arrival_estimator),
           path_finished_callback = std::move(path_finished_callback)]
    {
      if (_current_waypoint_target < waypoints.size())
        ++_current_waypoint_target;

      if (updater)
      {
        const auto& previous_wp = waypoints[_current_waypoint_target-1];
        std::cout << "Reached " << previous_wp.position().transpose() << std::endl;
        if (previous_wp.graph_index())
        {
          std::cout << "waypoint: " << *previous_wp.graph_index() << std::endl;
          updater->update_position(
                *previous_wp.graph_index(), previous_wp.position()[2]);
          _visited_wps.insert(*previous_wp.graph_index());
        }
        else
        {
          updater->update_position("test_map", previous_wp.position());
        }
      }

      if (_current_waypoint_target < waypoints.size())
      {
        const auto& wp = waypoints[_current_waypoint_target];
        const auto test_delay =
            std::chrono::milliseconds(750) * _current_waypoint_target;

        const auto delayed_arrival_time = wp.time() + test_delay;
        const auto remaining_time =
            std::chrono::steady_clock::time_point(
              std::chrono::steady_clock::duration(_node->now().nanoseconds()))
              - delayed_arrival_time;

        next_arrival_estimator(_current_waypoint_target, remaining_time);
        return;
      }

      std::cout << "Finished!" << std::endl;
      _timer.reset();
      path_finished_callback();
    });
  }

  void stop() final
  {
    _timer.reset();
  }

  void dock(
      const std::string& dock_name,
      std::function<void()> docking_finished_callback) final
  {
    std::cout << "Docking into " << dock_name << std::endl;
    ++_dockings.insert({dock_name, 0}).first->second;
    docking_finished_callback();
  }

  const std::unordered_map<std::string, std::size_t>& dockings() const
  {
    return _dockings;
  }

  const std::unordered_set<std::size_t> visited_wps() const
  {
    return _visited_wps;
  }

private:
  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::TimerBase::SharedPtr _timer;
  std::size_t _current_waypoint_target = 0;
  std::unordered_map<std::string, std::size_t> _dockings;
  std::unordered_set<std::size_t> _visited_wps;
};

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

    std::cout << " -- Received request" << std::endl;
    const auto insertion = _task_complete_map.insert({msg.request_guid, false});
    uint8_t status = DispenserResult::ACKNOWLEDGED;
    if (insertion.first->second)
    {
      status = DispenserResult::SUCCESS;
    }
    else
    {
      using namespace std::chrono_literals;
      std::cout << " -- Beginning timer" << std::endl;
      _timer = _node->create_wall_timer(
            10ms, [this, msg]()
      {
        std::cout << " -- Timer triggered" << std::endl;
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
/// This mock dispenser will not publish any results; it will only publish
/// states. This is representative of network issues where a result might not
/// actually arrive, but the state heartbeats can still get through.
class MockFlakyDispenser
{
public:

  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;

  MockFlakyDispenser(
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

    _state_pub = _node->create_publisher<DispenserState>(
          rmf_fleet_adapter::DispenserStateTopicName,
          rclcpp::SystemDefaultsQoS());

    using namespace std::chrono_literals;
    _timer = _node->create_wall_timer(
          100ms, [this]()
    {
      std::cout << " >> Updating Flaky Dispenser state" << std::endl;
      DispenserState msg;
      msg.guid = _name;

      if (_request_queue.empty())
        msg.mode = DispenserState::IDLE;
      else
        msg.mode = DispenserState::BUSY;

      msg.time = _node->now();
      msg.seconds_remaining = 0.1;

      for (auto& r : _request_queue)
      {
        std::cout << " -- Adding [" << r.request.request_guid << "] to the queue" << std::endl;
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
        std::cout << " -- Removed " << initial_count - _request_queue.size()
                  << " requests from the queue" << std::endl;
        if (!_fulfilled_promise)
        {
          std::cout << " -- Declaring the promise fulfilled" << std::endl;
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
    DispenserRequest request;
    std::size_t publish_count = 0;
  };

  std::shared_ptr<rclcpp::Node> _node;
  std::string _name;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  std::vector<RequestEntry> _request_queue;
  rclcpp::TimerBase::SharedPtr _timer;
  bool _fulfilled_promise = false;

  void _process_request(const DispenserRequest& msg)
  {
    if (msg.target_guid != _name)
      return;

    std::cout << " -- Received dispense request [" << msg.request_guid
              << "]" << std::endl;
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
   *                  10(D)
   *                   |
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6------7(D)
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

  add_bidir_lane(0, 1);  // 1   2
  add_bidir_lane(1, 2);  // 3   4
  add_bidir_lane(1, 5);  // 5   6
  add_bidir_lane(2, 6);  // 7   8
  add_bidir_lane(3, 4);  // 9  10
  add_bidir_lane(4, 5);  // 11 12
  add_bidir_lane(5, 6);  // 13 14
  add_dock_lane(6, 7, "A");  // 15 16
  add_bidir_lane(5, 8);  // 17 18
  add_bidir_lane(6, 9);  // 19 20
  add_bidir_lane(8, 9);  // 21 22
  add_dock_lane(8, 10, "B"); // 23 24

  const std::string pickup_name = "pickup";
  REQUIRE(graph.add_key(pickup_name, 7));

  const std::string dropoff_name = "dropoff";
  REQUIRE(graph.add_key(dropoff_name, 10));


  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  rclcpp::init(0, nullptr);
  auto adapter = rmf_fleet_adapter::agv::test::MockAdapter("test_Delivery");
  auto fleet = adapter.add_fleet("test_fleet", traits, graph);
  fleet->accept_delivery_requests(
        [](const rmf_task_msgs::msg::Delivery&)
  {
    // Accept all delivery requests
    return true;
  });

  const auto now = std::chrono::steady_clock::now();
  const rmf_traffic::agv::Plan::StartSet starts = {{now, 0, 0.0}};
  auto robot_cmd = std::make_shared<MockRobotCommand>(adapter.node());
  fleet->add_robot(
        robot_cmd, "T0", profile, starts,
        [&robot_cmd](rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
  {
    robot_cmd->updater = std::move(updater);
  });

  const std::string flaky_dispenser_name = "flaky";
  auto flaky_dispenser = MockFlakyDispenser(
        adapter.node(), flaky_dispenser_name);
  auto flaky_future = flaky_dispenser.success_promise.get_future();

  const std::string quiet_dispenser_name = "quiet";
  auto quiet_dispenser = MockQuietDispenser(
        adapter.node(), quiet_dispenser_name);
  auto quiet_future = quiet_dispenser.success_promise.get_future();

  adapter.start();

  rmf_task_msgs::msg::Delivery request;
  request.task_id = "test_id";

  request.pickup_place_name = pickup_name;
//  request.pickup_dispenser = flaky_dispenser_name;
  request.pickup_dispenser = quiet_dispenser_name;

  request.dropoff_place_name = dropoff_name;
//  request.dropoff_dispenser = quiet_dispenser_name;
  request.dropoff_dispenser = flaky_dispenser_name;

  adapter.request_delivery(request);

  const auto quiet_status = quiet_future.wait_for(5s);
  REQUIRE(quiet_status == std::future_status::ready);
  REQUIRE(quiet_future.get());

  const auto flaky_status = flaky_future.wait_for(5s);
  REQUIRE(flaky_status == std::future_status::ready);
  REQUIRE(flaky_future.get());

  const auto& visits = robot_cmd->visited_wps();
  CHECK(visits.size() == 5);
  CHECK(visits.count(0));
  CHECK(visits.count(5));
  CHECK(visits.count(6));
  CHECK(visits.count(7));
  CHECK(visits.count(8));
}
