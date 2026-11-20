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

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <rmf_fleet_msgs/msg/emergency_signal.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include <rmf_task_msgs/msg/dispatch_command.hpp>
#include <std_msgs/msg/string.hpp>

#include <rmf_utils/catch.hpp>

#include "../thread_cooldown.hpp"

#include <condition_variable>

namespace {

using namespace std::chrono_literals;

//==============================================================================
/// Collects everything the tests watch: the commands sent to the robot, the
/// latest status of each task, and the responses to task API requests.
class Monitor
{
public:

  void path_commanded(std::optional<std::size_t> destination)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _destinations.push_back(destination);
    _cv.notify_all();
  }

  void stop_commanded()
  {
    std::lock_guard<std::mutex> lock(_mutex);
    ++_stops;
  }

  void task_status(const std::string& task_id, const std::string& status)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _status[task_id] = status;
    _cv.notify_all();
  }

  void api_response(const std::string& request_id, nlohmann::json response)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _responses[request_id] = std::move(response);
    _cv.notify_all();
  }

  /// Number of path commands that were sent to a destination so far
  std::size_t commands_to(std::size_t destination) const
  {
    std::lock_guard<std::mutex> lock(_mutex);
    return std::count(
      _destinations.begin(), _destinations.end(),
      std::optional<std::size_t>(destination));
  }

  /// Number of path commands that were sent so far
  std::size_t commands() const
  {
    std::lock_guard<std::mutex> lock(_mutex);
    return _destinations.size();
  }

  /// Number of stop commands that were sent so far
  std::size_t stops() const
  {
    std::lock_guard<std::mutex> lock(_mutex);
    return _stops;
  }

  bool wait_for_command_to(
    std::size_t destination,
    std::chrono::milliseconds timeout = 10s)
  {
    std::unique_lock<std::mutex> lock(_mutex);
    return _cv.wait_for(
      lock, timeout, [&]()
      {
        return std::find(
          _destinations.begin(), _destinations.end(),
          std::optional<std::size_t>(destination)) != _destinations.end();
      });
  }

  std::string status(const std::string& task_id) const
  {
    std::lock_guard<std::mutex> lock(_mutex);
    const auto it = _status.find(task_id);
    if (it == _status.end())
      return "";

    return it->second;
  }

  bool wait_for_status(
    const std::string& task_id,
    const std::string& status,
    std::chrono::milliseconds timeout = 10s)
  {
    std::unique_lock<std::mutex> lock(_mutex);
    return _cv.wait_for(
      lock, timeout, [&]()
      {
        const auto it = _status.find(task_id);
        return it != _status.end() && it->second == status;
      });
  }

  std::optional<nlohmann::json> wait_for_response(
    const std::string& request_id,
    std::chrono::milliseconds timeout = 10s)
  {
    std::unique_lock<std::mutex> lock(_mutex);
    const bool received = _cv.wait_for(
      lock, timeout, [&]() { return _responses.count(request_id) > 0; });

    if (!received)
      return std::nullopt;

    return _responses.at(request_id);
  }

private:
  mutable std::mutex _mutex;
  std::condition_variable _cv;
  std::vector<std::optional<std::size_t>> _destinations;
  std::size_t _stops = 0;
  std::unordered_map<std::string, std::string> _status;
  std::unordered_map<std::string, nlohmann::json> _responses;
};

//==============================================================================
/// A robot that never reaches its destination, so the task that sent it stays
/// active until it gets interrupted or cancelled.
class HoldingRobotCommand : public rmf_fleet_adapter::agv::RobotCommandHandle
{
public:

  HoldingRobotCommand(std::shared_ptr<Monitor> monitor)
  : _monitor(std::move(monitor))
  {
    // Do nothing
  }

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator,
    RequestCompleted) final
  {
    if (waypoints.empty())
      return;

    _monitor->path_commanded(waypoints.back().graph_index());
  }

  void stop() final
  {
    _monitor->stop_commanded();
  }

  void dock(const std::string&, RequestCompleted) final
  {
    // Do nothing
  }

private:
  std::shared_ptr<Monitor> _monitor;
};

//==============================================================================
nlohmann::json patrol_request(const std::string& place)
{
  nlohmann::json request;
  request["category"] = "patrol";
  request["description"]["places"] = std::vector<std::string>{place};
  request["description"]["rounds"] = 1;
  return request;
}

} // anonymous namespace

//==============================================================================
SCENARIO("Cancel a task while it is interrupted")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, 0.0}).set_charger(true); // 0
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 1
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 2
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 3
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 4
  graph.add_waypoint(test_map_name, {-5.0, 0.0}).set_parking_spot(true); // 5

  /*
   *         4
   *         |
   *         3
   *         |
   *   5-----0-----1-----2
   **/

  const auto add_bidir_lane =
    [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(0, 3);
  add_bidir_lane(3, 4);
  add_bidir_lane(0, 5);

  const std::size_t place_a = 2;
  const std::size_t place_b = 4;
  const std::size_t parking_spot = 5;
  REQUIRE(graph.add_key("place_a", place_a));
  REQUIRE(graph.add_key("place_b", place_b));

  const std::string task_a = "task_a";
  const std::string task_b = "task_b";
  const std::string fleet_name = "test_fleet";

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
    "test_CancelInterruptedTask", rclcpp::NodeOptions().context(rcl_context));

  const auto fleet = adapter.add_fleet(fleet_name, traits, graph);

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

  fleet->consider_patrol_requests(
    [](const nlohmann::json&,
    rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      confirm.accept();
    });

  const auto monitor = std::make_shared<Monitor>();
  const auto robot_cmd = std::make_shared<HoldingRobotCommand>(monitor);

  std::promise<rmf_fleet_adapter::agv::RobotUpdateHandlePtr> updater_promise;
  auto updater_future = updater_promise.get_future();
  const auto now = rmf_traffic_ros2::convert(adapter.node()->now());
  const rmf_traffic::agv::Plan::StartSet starts = {{now, 0, 0.0}};
  fleet->add_robot(
    robot_cmd, "R0", profile, starts,
    [&updater_promise](rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
    {
      updater->update_battery_soc(1.0);
      updater_promise.set_value(std::move(updater));
    });

  const auto node = adapter.node();
  const auto transient_qos =
    rclcpp::SystemDefaultsQoS().keep_last(10).reliable().transient_local();

  using ApiRequest = rmf_task_msgs::msg::ApiRequest;
  using ApiResponse = rmf_task_msgs::msg::ApiResponse;
  const auto api_request_pub = node->create_publisher<ApiRequest>(
    rmf_fleet_adapter::TaskApiRequests, transient_qos);

  const auto api_response_sub = node->create_subscription<ApiResponse>(
    rmf_fleet_adapter::TaskApiResponses, transient_qos,
    [monitor](const ApiResponse::SharedPtr msg)
    {
      monitor->api_response(
        msg->request_id, nlohmann::json::parse(msg->json_msg));
    });

  const auto task_state_sub = node->create_subscription<std_msgs::msg::String>(
    rmf_fleet_adapter::TaskStateUpdateTopicName, transient_qos,
    [monitor](const std_msgs::msg::String::SharedPtr msg)
    {
      const auto update = nlohmann::json::parse(msg->data);
      const auto& state = update.at("data");
      monitor->task_status(
        state.at("booking").at("id").get<std::string>(),
        state.at("status").get<std::string>());
    });

  using EmergencySignal = rmf_fleet_msgs::msg::EmergencySignal;
  const auto emergency_pub = node->create_publisher<EmergencySignal>(
    rmf_traffic_ros2::EmergencySignalTopicName, transient_qos);

  using DispatchCommand = rmf_task_msgs::msg::DispatchCommand;
  const auto dispatch_pub = node->create_publisher<DispatchCommand>(
    rmf_fleet_adapter::DispatchRequestTopicName,
    rclcpp::ServicesQoS().keep_last(20).transient_local());

  // The task API topics are transient local, so a response from an earlier
  // run of this scenario can still arrive. Request IDs must not repeat.
  const std::string request_prefix = "test_request_" + std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count()) + "_";
  std::size_t next_request = 0;
  const auto send_request =
    [&](nlohmann::json request) -> std::string
    {
      ApiRequest msg;
      msg.request_id = request_prefix + std::to_string(next_request++);
      msg.json_msg = request.dump();
      api_request_pub->publish(msg);
      return msg.request_id;
    };

  const auto pause =
    [&](const std::string& task_id) -> std::string
    {
      const auto response = monitor->wait_for_response(
        send_request(
          {
            {"type", "interrupt_task_request"},
            {"task_id", task_id},
            {"labels", {"test pause"}}
          }));

      REQUIRE(response.has_value());
      REQUIRE(response->at("success").get<bool>());
      return response->at("token").get<std::string>();
    };

  const auto cancel =
    [&](const std::string& task_id)
    {
      const auto response = monitor->wait_for_response(
        send_request(
          {
            {"type", "cancel_task_request"},
            {"task_id", task_id},
            {"labels", {"test cancel"}}
          }));

      REQUIRE(response.has_value());
      CHECK(response->at("success").get<bool>());
    };

  const auto kill =
    [&](const std::string& task_id)
    {
      const auto response = monitor->wait_for_response(
        send_request(
          {
            {"type", "kill_task_request"},
            {"task_id", task_id},
            {"labels", {"test kill"}}
          }));

      REQUIRE(response.has_value());
      CHECK(response->at("success").get<bool>());
    };

  const auto resume =
    [&](const std::string& task_id, const std::string& token)
    {
      send_request(
        {
          {"type", "resume_task_request"},
          {"for_task", task_id},
          {"for_tokens", {token}},
          {"labels", {"test resume"}}
        });
    };

  adapter.start();

  REQUIRE(updater_future.wait_for(5s) == std::future_status::ready);
  const auto updater = updater_future.get();

  // Wait for the task manager to start
  std::this_thread::sleep_for(1s);

  // Task A sends the robot towards place_a, which it never reaches. Task B
  // waits in the queue behind it.
  adapter.dispatch_task(task_a, patrol_request("place_a"));
  REQUIRE(monitor->wait_for_command_to(place_a));
  REQUIRE(monitor->wait_for_status(task_a, "underway"));

  adapter.dispatch_task(task_b, patrol_request("place_b"));
  REQUIRE(monitor->wait_for_status(task_b, "queued"));

  // While a task is interrupted, the robot must not be sent any command, so
  // that whoever interrupted the task can take control of the robot. Commands
  // sent so far are counted once the interruption has stopped the robot.
  std::size_t commands = 0;
  std::size_t stops = 0;
  const auto robot_is_stopped =
    [&]()
    {
      std::this_thread::sleep_for(500ms);
      commands = monitor->commands();
      stops = monitor->stops();
    };

  const auto robot_was_left_alone =
    [&]()
    {
      std::this_thread::sleep_for(2s);
      CHECK(monitor->commands() == commands);
      CHECK(monitor->stops() == stops);
      CHECK(monitor->status(task_b) == "queued");
    };

  WHEN("A paused task is cancelled")
  {
    const auto token = pause(task_a);
    robot_is_stopped();
    cancel(task_a);

    THEN("The cancellation waits for the resume")
    {
      robot_was_left_alone();

      const auto commands_to_a = monitor->commands_to(place_a);
      resume(task_a, token);
      CHECK(monitor->wait_for_status(task_a, "canceled"));
      std::this_thread::sleep_for(2s);

      // The cancelled event does not start again
      CHECK(monitor->commands_to(place_a) == commands_to_a);
    }
  }

  WHEN("A task paused through the task API and the robot is cancelled")
  {
    std::optional<rmf_fleet_adapter::agv::RobotUpdateHandle::Interruption>
    robot_pause = updater->interrupt({"robot pause"}, []() {});
    const auto token = pause(task_a);
    robot_is_stopped();
    cancel(task_a);

    THEN("The cancellation waits until every pause is released")
    {
      resume(task_a, token);
      robot_was_left_alone();

      // Releasing the last pause finishes the cancelled task from inside of
      // the resume, which replaces the active task while the resume is still
      // being handled.
      const auto commands_to_a = monitor->commands_to(place_a);
      robot_pause->resume({"robot resume"});
      CHECK(monitor->wait_for_status(task_a, "canceled"));
      std::this_thread::sleep_for(2s);
      CHECK(monitor->commands_to(place_a) == commands_to_a);
    }
  }

  WHEN("A task paused by an emergency is cancelled by the dispatcher")
  {
    EmergencySignal emergency;
    emergency.is_emergency = true;
    emergency.fleet_names = {fleet_name};
    emergency_pub->publish(emergency);
    REQUIRE(monitor->wait_for_command_to(parking_spot));
    const auto commands_to_a = monitor->commands_to(place_a);

    // While the emergency is active, cancel_task_request is refused, but the
    // dispatcher can still remove the task.
    DispatchCommand remove;
    remove.fleet_name = fleet_name;
    remove.task_id = task_a;
    remove.dispatch_id = 1;
    remove.type = DispatchCommand::TYPE_REMOVE;
    dispatch_pub->publish(remove);

    THEN("The cancellation waits until the emergency is over")
    {
      std::this_thread::sleep_for(2s);
      CHECK(monitor->commands_to(place_a) == commands_to_a);
      CHECK(monitor->commands_to(place_b) == 0);

      // Ending the emergency resumes the task, which finishes it right away
      emergency.is_emergency = false;
      emergency_pub->publish(emergency);
      CHECK(monitor->wait_for_status(task_a, "canceled"));
      std::this_thread::sleep_for(2s);
      CHECK(monitor->commands_to(place_a) == commands_to_a);
    }
  }

  WHEN("A paused task is killed")
  {
    const auto token = pause(task_a);
    robot_is_stopped();
    kill(task_a);

    THEN("The task is killed once it is resumed")
    {
      robot_was_left_alone();

      const auto commands_to_a = monitor->commands_to(place_a);
      resume(task_a, token);
      CHECK(monitor->wait_for_status(task_a, "killed"));
      std::this_thread::sleep_for(2s);
      CHECK(monitor->commands_to(place_a) == commands_to_a);
    }
  }

  adapter.stop();
}
