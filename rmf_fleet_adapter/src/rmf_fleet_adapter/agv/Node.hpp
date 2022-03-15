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

#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>

#include <rmf_traffic/Time.hpp>

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

  std::function<rmf_traffic::Time()> clock() const;

  rmf_traffic::Time rmf_now() const;

  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorStateObs = rxcpp::observable<DoorState::SharedPtr>;
  const DoorStateObs& door_state() const;

  using DoorSupervisorState = rmf_door_msgs::msg::SupervisorHeartbeat;
  using DoorSupervisorObs = rxcpp::observable<DoorSupervisorState::SharedPtr>;
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

  using ApiRequest = rmf_task_msgs::msg::ApiRequest;
  using ApiRequestObs = rxcpp::observable<ApiRequest::SharedPtr>;
  const ApiRequestObs& task_api_request() const;

  using ApiResponse = rmf_task_msgs::msg::ApiResponse;
  using ApiResponsePub = rclcpp::Publisher<ApiResponse>::SharedPtr;
  const ApiResponsePub& task_api_response() const;

  template<typename DurationRepT, typename DurationT, typename CallbackT>
  rclcpp::TimerBase::SharedPtr try_create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback)
  {
    // Race conditions with shutting down the ROS2 node may cause
    // create_wall_timer to throw an exception. We'll catch that exception here
    // so that the thread and process can wind down gracefully.
    try
    {
      return create_wall_timer(period, std::move(callback));
    }
    catch (const rclcpp::exceptions::RCLError& e)
    {
      // RCL_RET_NOT_INIT will be given by rcl when trying to create a timer if
      // either:
      // 1. The node has not been initialized yet, or
      // 2. The node's context has already been shutdown.
      // Scenario (1) should not be possible because the rmf_fleet_adapter API
      // does not allow a TrafficLight instance to be created before the node is
      // initialized. Therefore we will assume scenario (2) here, in which case we
      // will simply allow this exception to pass by us since the TrafficLight
      // instance should be getting torn down soon.
      //
      // If the exception is being caused by anything else, then we should
      // continue to throw it because we don't have a valid reason to expect any
      // other
      if (e.ret == RCL_RET_NOT_INIT)
        return nullptr;

      throw e;
    }
  }

private:

  Node(
    rxcpp::schedulers::worker worker,
    const std::string& node_name,
    const rclcpp::NodeOptions& options);

  Bridge<DoorState> _door_state_obs;
  Bridge<DoorSupervisorState> _door_supervisor_obs;
  DoorRequestPub _door_request_pub;
  Bridge<LiftState> _lift_state_obs;
  LiftRequestPub _lift_request_pub;
  TaskSummaryPub _task_summary_pub;
  DispenserRequestPub _dispenser_request_pub;
  Bridge<DispenserResult> _dispenser_result_obs;
  Bridge<DispenserState> _dispenser_state_obs;
  Bridge<EmergencyNotice> _emergency_notice_obs;
  IngestorRequestPub _ingestor_request_pub;
  Bridge<IngestorResult> _ingestor_result_obs;
  Bridge<IngestorState> _ingestor_state_obs;
  FleetStatePub _fleet_state_pub;
  Bridge<ApiRequest> _task_api_request_obs;
  ApiResponsePub _task_api_response_pub;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__NODE_HPP
