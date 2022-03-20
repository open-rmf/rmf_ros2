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

#include "Node.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
std::shared_ptr<Node> Node::make(
  rxcpp::schedulers::worker worker,
  const std::string& node_name,
  const rclcpp::NodeOptions& options)
{
  auto node = std::shared_ptr<Node>(
    new Node(std::move(worker), node_name, options));

  auto default_qos = rclcpp::SystemDefaultsQoS();
  default_qos.keep_last(100);

  node->_door_state_obs =
    node->create_observable<DoorState>(
    DoorStateTopicName, default_qos);

  node->_door_supervisor_obs =
    node->create_observable<DoorSupervisorState>(
    DoorSupervisorHeartbeatTopicName, default_qos);

  node->_door_request_pub =
    node->create_publisher<DoorRequest>(
    AdapterDoorRequestTopicName, default_qos);

  node->_lift_state_obs =
    node->create_observable<LiftState>(
    LiftStateTopicName, default_qos);

  node->_lift_request_pub =
    node->create_publisher<LiftRequest>(
    AdapterLiftRequestTopicName, default_qos);

  node->_task_summary_pub =
    node->create_publisher<TaskSummary>(
    TaskSummaryTopicName, default_qos);

  node->_dispenser_request_pub =
    node->create_publisher<DispenserRequest>(
    DispenserRequestTopicName, default_qos);

  node->_dispenser_result_obs =
    node->create_observable<DispenserResult>(
    DispenserResultTopicName, default_qos);

  node->_dispenser_state_obs =
    node->create_observable<DispenserState>(
    DispenserStateTopicName, default_qos);

  node->_emergency_notice_obs =
    node->create_observable<EmergencyNotice>(
    rmf_traffic_ros2::EmergencyTopicName, default_qos);

  node->_ingestor_request_pub =
    node->create_publisher<IngestorRequest>(
    IngestorRequestTopicName, default_qos);

  node->_ingestor_result_obs =
    node->create_observable<IngestorResult>(
    IngestorResultTopicName, default_qos);

  node->_ingestor_state_obs =
    node->create_observable<IngestorState>(
    IngestorStateTopicName, default_qos);

  node->_fleet_state_pub =
    node->create_publisher<FleetState>(
    FleetStateTopicName, default_qos);

  node->_task_api_request_obs =
    node->create_observable<ApiRequest>(
    TaskApiRequests, rclcpp::SystemDefaultsQoS().reliable().transient_local());

  node->_task_api_response_pub =
    node->create_publisher<ApiResponse>(
    TaskApiResponses, rclcpp::SystemDefaultsQoS().reliable().transient_local());

  return node;
}

//==============================================================================
Node::Node(
  rxcpp::schedulers::worker worker,
  const std::string& node_name,
  const rclcpp::NodeOptions& options)
: rmf_rxcpp::Transport(std::move(worker), node_name, options)
{
  // Do nothing
}

//==============================================================================
std::function<rmf_traffic::Time()> Node::clock() const
{
  return [self = shared_from_this()]()
    {
      return rmf_traffic_ros2::convert(self->now());
    };
}

//==============================================================================
rmf_traffic::Time Node::rmf_now() const
{
  return rmf_traffic_ros2::convert(now());
}

//==============================================================================
auto Node::door_state() const -> const DoorStateObs&
{
  return _door_state_obs->observe();
}

//==============================================================================
auto Node::door_supervisor() const -> const DoorSupervisorObs&
{
  return _door_supervisor_obs->observe();
}

//==============================================================================
auto Node::door_request() const -> const DoorRequestPub&
{
  return _door_request_pub;
}

//==============================================================================
auto Node::lift_state() const -> const LiftStateObs&
{
  return _lift_state_obs->observe();
}

//==============================================================================
auto Node::lift_request() const -> const LiftRequestPub&
{
  return _lift_request_pub;
}

//==============================================================================
auto Node::task_summary() const -> const TaskSummaryPub&
{
  return _task_summary_pub;
}

//==============================================================================
auto Node::dispenser_request() const -> const DispenserRequestPub&
{
  return _dispenser_request_pub;
}

//==============================================================================
auto Node::dispenser_result() const -> const DispenserResultObs&
{
  return _dispenser_result_obs->observe();
}

//==============================================================================
auto Node::dispenser_state() const -> const DispenserStateObs&
{
  return _dispenser_state_obs->observe();
}

//==============================================================================
auto Node::emergency_notice() const -> const EmergencyNoticeObs&
{
  return _emergency_notice_obs->observe();
}

auto Node::ingestor_request() const -> const IngestorRequestPub&
{
  return _ingestor_request_pub;
}

//==============================================================================
auto Node::ingestor_result() const -> const IngestorResultObs&
{
  return _ingestor_result_obs->observe();
}

//==============================================================================
auto Node::ingestor_state() const -> const IngestorStateObs&
{
  return _ingestor_state_obs->observe();
}

//==============================================================================
auto Node::fleet_state() const -> const FleetStatePub&
{
  return _fleet_state_pub;
}

//==============================================================================
auto Node::task_api_request() const -> const ApiRequestObs&
{
  return _task_api_request_obs->observe();
}

//==============================================================================
auto Node::task_api_response() const -> const ApiResponsePub&
{
  return _task_api_response_pub;
}

} // namespace agv
} // namespace rmf_fleet_adapter
