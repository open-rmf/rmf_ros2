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

#include "DockRobot.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
DockRobot::ActivePhase::ActivePhase(
  agv::RobotContextPtr context,
  std::string dock_name)
: _context{std::move(context)},
  _dock_name{std::move(dock_name)},
  _be_stubborn(_context->be_stubborn())
{
  std::ostringstream oss;
  oss << "Docking robot to " << _dock_name;
  _description = oss.str();

  _action = std::make_shared<Action>(this);
  _obs = rmf_rxcpp::make_job<LegacyTask::StatusMsg>(_action);

  _context->current_mode(rmf_fleet_msgs::msg::RobotMode::MODE_DOCKING);
}

//==============================================================================
const rxcpp::observable<LegacyTask::StatusMsg>&
DockRobot::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration DockRobot::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DockRobot::ActivePhase::emergency_alarm(bool)
{
  // TODO: implement
}

//==============================================================================
void DockRobot::ActivePhase::cancel()
{
  // no op
}

//==============================================================================
const std::string& DockRobot::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
DockRobot::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string dock_name)
: _context{std::move(context)},
  _dock_name{std::move(dock_name)}
{
  std::ostringstream oss;
  oss << "Dock robot to " << _dock_name;
  _description = oss.str();
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> DockRobot::PendingPhase::begin()
{
  return std::make_shared<DockRobot::ActivePhase>(_context, _dock_name);
}

//==============================================================================
rmf_traffic::Duration DockRobot::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& DockRobot::PendingPhase::description() const
{
  return _description;
}

//==============================================================================
DockRobot::Action::Action(ActivePhase* phase)
: _phase(phase)
{
  // Do nothing
}

} // namespace phases
} // namespace rmf_fleet_adapter
