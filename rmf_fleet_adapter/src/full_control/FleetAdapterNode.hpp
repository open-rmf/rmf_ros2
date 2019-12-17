/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef SRC__FULL_CONTROL__FLEETADAPTERNODE_HPP
#define SRC__FULL_CONTROL__FLEETADAPTERNODE_HPP

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>

#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>

#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>
#include <rmf_task_msgs/msg/task_summary.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <std_msgs/msg/bool.hpp>

#include <rclcpp/node.hpp>

#include <rmf_utils/optional.hpp>

#include <unordered_map>
#include <vector>
#include <queue>

#include "Action.hpp"
#include "Task.hpp"
#include "../rmf_fleet_adapter/ParseGraph.hpp"

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
class FleetAdapterNode : public rclcpp::Node
{
public:

  static std::shared_ptr<FleetAdapterNode> make();

  using WaypointKeys = std::unordered_map<std::string, std::size_t>;
  using WaypointNames = std::unordered_map<std::size_t, std::string>;

  using Delivery = rmf_task_msgs::msg::Delivery;
  using Location = rmf_fleet_msgs::msg::Location;

  using RobotState = rmf_fleet_msgs::msg::RobotState;
  using RobotStateListeners = std::unordered_set<Listener<RobotState>*>;

  struct RobotContext
  {
    RobotContext(std::string name, Location location);

    Location location;

    void next_task();

    void add_task(std::unique_ptr<Task> new_task);

    void discard_task(Task* discarded_task);

    void interrupt();

    void resume();

    std::size_t num_tasks() const;

    const std::string& robot_name() const;

    void insert_listener(Listener<RobotState>* listener);

    void remove_listener(Listener<RobotState>* listener);

    void update_listeners(const RobotState& state);

  private:
    std::unique_ptr<Task> _task;
    std::vector<std::unique_ptr<Task>> _task_queue;
    const std::string _name;
    RobotStateListeners state_listeners;
  };

  bool ignore_fleet(const std::string& fleet_name) const;

  const std::string& get_fleet_name() const;

  rmf_traffic::Duration get_plan_time() const;

  rmf_traffic::Duration get_delay_threshold() const;

  const rmf_traffic::agv::Planner& get_planner() const;

  const rmf_traffic::agv::Graph& get_graph() const;

  std::vector<rmf_traffic::agv::Plan::Start> compute_plan_starts(
      const Location& location,
      const std::chrono::nanoseconds start_delay);

  const WaypointKeys& get_waypoint_keys() const;

  const WaypointNames& get_waypoint_names() const;

  const std::vector<std::size_t>& get_parking_spots() const;

  struct Fields
  {
    rmf_traffic_ros2::schedule::MirrorManager mirror;
    std::unique_ptr<ScheduleConnections> schedule;
    GraphInfo graph_info;
    rmf_traffic::agv::VehicleTraits traits;
    rmf_traffic::agv::Planner planner;

    Fields(
        GraphInfo graph_info_,
        rmf_traffic::agv::VehicleTraits traits_,
        rmf_traffic_ros2::schedule::MirrorManager mirror_,
        std::unique_ptr<ScheduleConnections> connections_)
    : mirror(std::move(mirror_)),
      schedule(std::move(connections_)),
      graph_info(std::move(graph_info_)),
      traits(std::move(traits_)),
      planner(
        rmf_traffic::agv::Planner::Configuration(graph_info.graph, traits),
        rmf_traffic::agv::Planner::Options(mirror.viewer()))
    {
      // Do nothing
    }
  };

  Fields& get_fields();

  const Fields& get_fields() const;

  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorStateListeners = std::unordered_set<Listener<DoorState>*>;
  DoorStateListeners door_state_listeners;

  using LiftState = rmf_lift_msgs::msg::LiftState;
  using LiftStateListeners = std::unordered_set<Listener<LiftState>*>;
  LiftStateListeners lift_state_listeners;

  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;

  using DispenserResultListeners =
      std::unordered_set<Listener<DispenserResult>*>;
  DispenserResultListeners dispenser_result_listeners;

  using DispenserStateListeners =
      std::unordered_set<Listener<DispenserState>*>;
  DispenserStateListeners dispenser_state_listeners;

  using PathRequest = rmf_fleet_msgs::msg::PathRequest;
  using PathRequestPub = rclcpp::Publisher<PathRequest>;
  PathRequestPub::SharedPtr path_request_publisher;

  using ModeRequest = rmf_fleet_msgs::msg::ModeRequest;
  using ModeRequestPub = rclcpp::Publisher<ModeRequest>;
  ModeRequestPub::SharedPtr mode_request_publisher;

  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using DoorRequestPub = rclcpp::Publisher<DoorRequest>;
  DoorRequestPub::SharedPtr door_request_publisher;

  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  using LiftRequestPub = rclcpp::Publisher<LiftRequest>;
  LiftRequestPub::SharedPtr lift_request_publisher;

  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserRequestPub = rclcpp::Publisher<DispenserRequest>;
  DispenserRequestPub::SharedPtr dispenser_request_publisher;

  using TaskSummary = rmf_task_msgs::msg::TaskSummary;
  using TaskSummaryPub = rclcpp::Publisher<TaskSummary>;
  TaskSummaryPub::SharedPtr task_summary_publisher;

private:

  FleetAdapterNode();

  std::string _fleet_name;

  rmf_traffic::Duration _delay_threshold;

  rmf_traffic::Duration _plan_time;

  void start(Fields fields);

  rmf_utils::optional<Fields> _field;

  using DeliverySub = rclcpp::Subscription<Delivery>;
  DeliverySub::SharedPtr _delivery_sub;
  void delivery_request(Delivery::UniquePtr msg);

  using LoopRequest = rmf_task_msgs::msg::Loop;
  using LoopRequestSub = rclcpp::Subscription<LoopRequest>;
  LoopRequestSub::SharedPtr _loop_request_sub;
  void loop_request(LoopRequest::UniquePtr msg);

  // FIXME(MXG): To avoid memory leaks, this set should be periodically purged
  // of tasks that are long past completed.
  std::unordered_set<std::string> _received_tasks;

  using DispenserResultSub = rclcpp::Subscription<DispenserResult>;
  DispenserResultSub::SharedPtr _dispenser_result_sub;
  void dispenser_result_update(DispenserResult::UniquePtr msg);

  using DispenserStateSub = rclcpp::Subscription<DispenserState>;
  DispenserStateSub::SharedPtr _dispenser_state_sub;
  void dispenser_state_update(DispenserState::UniquePtr msg);

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStateSub = rclcpp::Subscription<FleetState>;
  FleetStateSub::SharedPtr _fleet_state_sub;
  void fleet_state_update(FleetState::UniquePtr msg);

  using DoorStateSub = rclcpp::Subscription<DoorState>;
  DoorStateSub::SharedPtr _door_state_sub;
  void door_state_update(DoorState::UniquePtr msg);

  using LiftStateSub = rclcpp::Subscription<LiftState>;
  LiftStateSub::SharedPtr _lift_state_sub;
  void lift_state_update(LiftState::UniquePtr msg);

  using EmergencyNotice = std_msgs::msg::Bool;
  using EmergencyNoticeSub = rclcpp::Subscription<EmergencyNotice>;
  EmergencyNoticeSub::SharedPtr _emergency_notice_sub;
  void emergency_notice_update(EmergencyNotice::UniquePtr msg);

  bool _in_emergency_mode = false;

  bool _perform_deliveries = false;

  using Context =
      std::unordered_map<std::string, std::unique_ptr<RobotContext>>;
  Context _contexts;
};

} // namespace full_control
} // namespace rmf_fleet_adapter

#endif // SRC__FULL_CONTROL__FLEETADAPTERNODE_HPP
