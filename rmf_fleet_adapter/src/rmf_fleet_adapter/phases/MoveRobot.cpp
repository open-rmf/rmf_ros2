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

#include "MoveRobot.hpp"
#include "RxOperators.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
MoveRobot::ActivePhase::ActivePhase(
  agv::RobotContextPtr context,
  std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints,
  std::optional<rmf_traffic::Duration> tail_period)
: _context{std::move(context)},
  _tail_period(tail_period)
{
  std::ostringstream oss;
  oss << "Moving [" << _context->requester_id() << "]: ("
      << waypoints.front().position().transpose()
      << ") -> (" << waypoints.back().position().transpose() << ")";
  _description = oss.str();

  _action = std::make_shared<MoveRobot::Action>(
    _context, waypoints, _tail_period);

  auto job = rmf_rxcpp::make_job<Task::StatusMsg>(_action);

  _obs = make_cancellable(job, _cancel_subject.get_observable())
    .lift<Task::StatusMsg>(grab_while_active())
    .observe_on(rxcpp::identity_same_worker(_context->worker()));
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& MoveRobot::ActivePhase::observe()
const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration MoveRobot::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void MoveRobot::ActivePhase::emergency_alarm(bool)
{
  // TODO: implement
}

//==============================================================================
void MoveRobot::ActivePhase::cancel()
{
  _context->command()->stop();
  _cancel_subject.get_subscriber().on_next(true);
}

//==============================================================================
const std::string& MoveRobot::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
MoveRobot::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints,
  std::optional<rmf_traffic::Duration> tail_period)
: _context{std::move(context)},
  _waypoints{std::move(waypoints)},
  _tail_period(tail_period)
{
  std::ostringstream oss;
  oss << "Move [" << _context->requester_id() << "] to ("
      << _waypoints.back().position().transpose() << ")";
  _description = oss.str();
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> MoveRobot::PendingPhase::begin()
{
  return std::make_shared<MoveRobot::ActivePhase>(
    _context, _waypoints, _tail_period);
}

//==============================================================================
rmf_traffic::Duration MoveRobot::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& MoveRobot::PendingPhase::description() const
{
  return _description;
}

//==============================================================================
MoveRobot::Action::Action(
  agv::RobotContextPtr& context,
  std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  std::optional<rmf_traffic::Duration> tail_period)
: _context{context},
  _waypoints{waypoints},
  _tail_period{tail_period}
{
  // no op
}

} // namespace phases
} // namespace rmf_fleet_adapter
