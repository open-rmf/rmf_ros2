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

#include "RequestLift.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
RequestLift::ActivePhase::ActivePhase(
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  std::string lift_name,
  std::string destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState> lift_state_obs)
  : _transport{std::move(transport)},
    _lift_name{std::move(lift_name)},
    _destination{std::move(destination)},
    _lift_state_obs{std::move(lift_state_obs)}
{
  std::ostringstream oss;
  oss << "Requesting lift \"" << lift_name << "\" to \"" << destination << "\"";

  _description = oss.str();

  _job = rmf_rxcpp::make_job<Task::StatusMsg>(
    std::make_shared<RequestLift::Action>(
      _transport, _lift_name, _destination, _lift_state_obs));
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& RequestLift::ActivePhase::observe() const
{
  return _job;
}

//==============================================================================
rmf_traffic::Duration RequestLift::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void RequestLift::ActivePhase::emergency_alarm(bool on)
{
  // TODO: implement
}

//==============================================================================
void RequestLift::ActivePhase::cancel()
{
  // TODO: implement
}

//==============================================================================
const std::string& RequestLift::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
RequestLift::PendingPhase::PendingPhase(
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  std::string lift_name,
  std::string destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState> lift_state_obs)
  : _transport{std::move(transport)},
    _lift_name{std::move(lift_name)},
    _destination{std::move(destination)},
    _lift_state_obs{std::move(lift_state_obs)}
{
  std::ostringstream oss;
  oss << "Requesting lift \"" << lift_name << "\" to \"" << destination << "\"";

  _description = oss.str();
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> RequestLift::PendingPhase::begin()
{
  return std::make_shared<RequestLift::ActivePhase>(
    _transport, _lift_name, _destination, _lift_state_obs);
}

//==============================================================================
rmf_traffic::Duration RequestLift::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& RequestLift::PendingPhase::description() const
{
  return _description;
}

//==============================================================================
RequestLift::Action::Action(
  std::weak_ptr<rmf_rxcpp::Transport>& transport,
  std::string& lift_name,
  std::string& destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState>& lift_state_obs)
  : _transport{transport},
    _lift_name{lift_name},
    _destination{destination},
    _lift_state_obs{lift_state_obs}
{
  // no op
}

} // namespace phases
} // namespace rmf_fleet_adapter