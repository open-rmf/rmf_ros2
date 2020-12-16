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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__NODE_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__NODE_HPP

#include <rmf_rxcpp/Transport.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>
#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>
#include <rmf_task_msgs/msg/task_summary.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Node : public rmf_rxcpp::Transport
{
public:

  static std::shared_ptr<Node> make(
      rxcpp::schedulers::worker worker,
      const std::string& node_name,
      const rclcpp::NodeOptions& options);

  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorStateObs = rxcpp::observable<DoorState::SharedPtr>;
  const DoorStateObs& door_state() const;

  using DoorSupervisorState = rmf_door_msgs::msg::SupervisorHeartbeat;
  using DoorSupervisorObs =   rxcpp::observable<DoorSupervisorState::SharedPtr>;
  const DoorSupervisorObs& door_supervisor() const;

  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using DoorRequestPub = rclcpp::Publisher<DoorRequest>::SharedPtr;
  const DoorRequestPub& door_request() const;

  using LiftState = rmf_lift_msgs::msg::LiftState;
  using LiftStateObs = rxcpp::observable<LiftState::SharedPtr>;
  const LiftStateObs& lift_state() const;

  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  using LiftRequestPub = rclcpp::Publisher<LiftRequest>::SharedPtr;
  const LiftRequestPub& lift_request() const;

  using TaskSummary = rmf_task_msgs::msg::TaskSummary;
  using TaskSummaryPub = rclcpp::Publisher<TaskSummary>::SharedPtr;
  const TaskSummaryPub& task_summary() const;

  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserRequestPub = rclcpp::Publisher<DispenserRequest>::SharedPtr;
  const DispenserRequestPub& dispenser_request() const;

  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using DispenserResultObs = rxcpp::observable<DispenserResult::SharedPtr>;
  const DispenserResultObs& dispenser_result() const;

  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserStateObs = rxcpp::observable<DispenserState::SharedPtr>;
  const DispenserStateObs& dispenser_state() const;

  using EmergencyNotice = std_msgs::msg::Bool;
  using EmergencyNoticeObs = rxcpp::observable<EmergencyNotice::SharedPtr>;
  const EmergencyNoticeObs& emergency_notice() const;

  using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
  using IngestorRequestPub = rclcpp::Publisher<IngestorRequest>::SharedPtr;
  const IngestorRequestPub& ingestor_request() const;

  using IngestorResult = rmf_ingestor_msgs::msg::IngestorResult;
  using IngestorResultObs = rxcpp::observable<IngestorResult::SharedPtr>;
  const IngestorResultObs& ingestor_result() const;

  using IngestorState = rmf_ingestor_msgs::msg::IngestorState;
  using IngestorStateObs = rxcpp::observable<IngestorState::SharedPtr>;
  const IngestorStateObs& ingestor_state() const;

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStatePub = rclcpp::Publisher<FleetState>::SharedPtr;
  const FleetStatePub& fleet_state() const;

private:

  Node(
      rxcpp::schedulers::worker worker,
      const std::string& node_name,
      const rclcpp::NodeOptions& options);

  DoorStateObs _door_state_obs;
  DoorSupervisorObs _door_supervisor_obs;
  DoorRequestPub _door_request_pub;
  LiftStateObs _lift_state_obs;
  LiftRequestPub _lift_request_pub;
  TaskSummaryPub _task_summary_pub;
  DispenserRequestPub _dispenser_request_pub;
  DispenserResultObs _dispenser_result_obs;
  DispenserStateObs _dispenser_state_obs;
  EmergencyNoticeObs _emergency_notice_obs;
  IngestorRequestPub _ingestor_request_pub;
  IngestorResultObs _ingestor_result_obs;
  IngestorStateObs _ingestor_state_obs;
  FleetStatePub _fleet_state_pub;

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__NODE_HPP
