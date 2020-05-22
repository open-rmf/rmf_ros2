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

#include "DoorClose.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
DoorClose::ActivePhase::ActivePhase(
  std::string door_name,
  std::shared_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState> door_state_obs,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat> supervisor_heartbeat_obs)
  : _door_name{std::move(door_name)},
    _transport{std::move(transport)},
    _door_state_obs{std::move(door_state_obs)},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)}
{
  _job = rmf_rxcpp::make_job<Task::StatusMsg>(
    std::make_shared<DoorControlAction>(
      _door_name,
      rmf_door_msgs::msg::DoorMode::MODE_CLOSED,
      _transport,
      _door_state_obs,
      _supervisor_heartbeat_obs));
  _description = "Closing door \"" + _door_name + "\"";
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& DoorClose::ActivePhase::observe() const
{
  return _job;
}

//==============================================================================
rmf_traffic::Duration DoorClose::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DoorClose::ActivePhase::emergency_alarm(bool /*on*/)
{
  // TODO: implement
}

//==============================================================================
void DoorClose::ActivePhase::cancel()
{
  // TODO: implement
}

//==============================================================================
const std::string& DoorClose::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
DoorClose::PendingPhase::PendingPhase(
  std::string  door_name,
  std::shared_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState> door_state_obs,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat> supervisor_heartbeat_obs)
  : _door_name{std::move(door_name)},
    _transport{std::move(transport)},
    _door_state_obs{std::move(door_state_obs)},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)}
{
  _description = "Close door \"" + _door_name + "\"";
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> DoorClose::PendingPhase::begin()
{
  return std::make_shared<DoorClose::ActivePhase>(
    _door_name,
    _transport,
    _door_state_obs,
    _supervisor_heartbeat_obs);
}

//==============================================================================
rmf_traffic::Duration DoorClose::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& DoorClose::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
