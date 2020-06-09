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

#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Node : public rmf_rxcpp::Transport
{
public:

  Node(
      const std::string& node_name,
      const rclcpp::NodeOptions& options);

  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorStateObs = rxcpp::observable<DoorState::SharedPtr>;

  using DoorSupervisorState = rmf_door_msgs::msg::SupervisorHeartbeat;
  using DoorSupervisorObs =   rxcpp::observable<DoorSupervisorState::SharedPtr>;

  const DoorStateObs& door_state() const;

  const DoorSupervisorObs& door_supervisor() const;

private:

  DoorStateObs _door_state_obs;
  DoorSupervisorObs _door_supervisor_obs;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__NODE_HPP
