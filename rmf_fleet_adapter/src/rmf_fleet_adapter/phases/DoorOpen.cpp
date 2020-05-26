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

#include "DoorOpen.hpp"

#include <utility>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
DoorOpen::ActivePhase::ActivePhase(
  std::string door_name,
  std::shared_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState> door_state_obs,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat> supervisor_heartbeat_obs)
  : _door_name{std::move(door_name)},
    _transport{std::move(transport)},
    _door_state_obs{std::move(door_state_obs)},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)},
    _action{
      _door_name,
      rmf_door_msgs::msg::DoorMode::MODE_OPEN,
      _transport,
      _door_state_obs,
      _supervisor_heartbeat_obs}
{
  _description = "Opening door \"" + _door_name + "\"";

  // When the phase is cancelled, queue a door close action instead of finishing immediately to
  // make sure that there are no hanging opened doors.
  auto post_finish_action = std::make_shared<DoorControlAction>(
    _door_name,
    rmf_door_msgs::msg::DoorMode::MODE_CLOSED,
    _transport,
    _door_state_obs,
    _supervisor_heartbeat_obs);

  auto post_finish_obs = rxcpp::observable<>::create<Task::StatusMsg>(
    [this, post_finish_action](const auto& s)
    {
      if (!_action.is_cancelled())
        s.on_completed();
      else
        post_finish_action->get_observable().subscribe(s);
    });

  _obs = _action.get_observable().concat(post_finish_obs);
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& DoorOpen::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration DoorOpen::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DoorOpen::ActivePhase::emergency_alarm(bool /*on*/)
{
  // TODO: implement
}

//==============================================================================
void DoorOpen::ActivePhase::cancel()
{
  _action.cancel();
}

//==============================================================================
const std::string& DoorOpen::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
DoorOpen::PendingPhase::PendingPhase(
  std::string  door_name,
  std::shared_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState> door_state_obs,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat> supervisor_heartbeat_obs)
  : _door_name{std::move(door_name)},
    _transport{std::move(transport)},
    _door_state_obs{std::move(door_state_obs)},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)}
{
  _description = "Open door \"" + _door_name + "\"";
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> DoorOpen::PendingPhase::begin()
{
  return std::make_shared<DoorOpen::ActivePhase>(
    _door_name,
    _transport,
    _door_state_obs,
    _supervisor_heartbeat_obs);
}

//==============================================================================
rmf_traffic::Duration DoorOpen::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& DoorOpen::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
