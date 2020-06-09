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
  _door_state_obs = create_observable<DoorState>(DoorStateTopicName, 10);
  _door_supervisor_obs = create_observable<DoorSupervisorState>(
        DoorSupervisorHeartbeatTopicName, 10);
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

} // namespace agv
} // namespace rmf_fleet_adapter
