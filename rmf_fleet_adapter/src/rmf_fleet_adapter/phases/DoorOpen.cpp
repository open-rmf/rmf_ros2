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
const std::string DoorOpen::_Description = "Opening door";

struct ASD
{
  template<typename Subscriber>
  void operator()(const Subscriber& s)
  {

  }
};
//==============================================================================
DoorOpen::ActivePhase::ActivePhase(
  std::string door_name,
  std::shared_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState> door_state_obs)
  : _door_name{std::move(door_name)},
    _transport{std::move(transport)},
    _door_state_obs{std::move(door_state_obs)}
{
  _job = make_job<Task::StatusMsg>(
    std::make_shared<DoorOpen::Action>(_door_name, *_transport, _door_state_obs));
}

//==============================================================================
rxcpp::observable<Task::StatusMsg>& DoorOpen::ActivePhase::observe()
{
  return _job;
}

//==============================================================================
rmf_traffic::Duration DoorOpen::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DoorOpen::ActivePhase::emergency_alarm(bool on)
{
  // TODO: implement
}

//==============================================================================
void DoorOpen::ActivePhase::cancel()
{
  // TODO: implement
}

//==============================================================================
const std::string& DoorOpen::ActivePhase::description() const
{
  return _Description;
}

//==============================================================================
void DoorOpen::ActivePhase::respond(
  const rmf_traffic::schedule::Negotiation::Table::ViewerPtr& table_viewer,
  const rmf_traffic::schedule::Negotiator::Responder& responder, const bool* interrupt_flag)
{
  // TODO: implement
}

//==============================================================================
DoorOpen::PendingPhase::PendingPhase(
  std::string  door_name,
  std::shared_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState> door_state_obs)
  : _door_name{std::move(door_name)},
    _transport{std::move(transport)},
    _door_state_obs{std::move(door_state_obs)}
{
  // no op
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> DoorOpen::PendingPhase::begin()
{
  return std::make_shared<DoorOpen::ActivePhase>(_door_name, _transport, _door_state_obs);
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
  return _Description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
