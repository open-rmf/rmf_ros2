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

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
Node::Node(const std::string& node_name, const rclcpp::NodeOptions& options)
  : rmf_rxcpp::Transport(node_name, options)
{
  auto default_qos = rclcpp::SystemDefaultsQoS();
  _door_state_obs = create_observable<DoorState>(
        DoorStateTopicName, default_qos);
  _door_supervisor_obs = create_observable<DoorSupervisorState>(
        DoorSupervisorHeartbeatTopicName, default_qos);
  _door_request_pub = create_publisher<DoorRequest>(
        AdapterDoorRequestTopicName, default_qos);
  _lift_state_obs = create_observable<LiftState>(
        LiftStateTopicName, default_qos);
  _lift_request_pub = create_publisher<LiftRequest>(
        AdapterLiftRequestTopicName, default_qos);
}

//==============================================================================
auto Node::door_state() const -> const DoorStateObs&
{
  return _door_state_obs;
}

//==============================================================================
auto Node::door_supervisor() const -> const DoorSupervisorObs&
{
  return _door_supervisor_obs;
}

//==============================================================================
auto Node::door_request() const -> const DoorRequestPub&
{
  return _door_request_pub;
}

//==============================================================================
auto Node::lift_state() const -> const LiftStateObs&
{
  return _lift_state_obs;
}

//==============================================================================
auto Node::lift_request() const -> const LiftRequestPub&
{
  return _lift_request_pub;
}

} // namespace agv
} // namespace rmf_fleet_adapter
