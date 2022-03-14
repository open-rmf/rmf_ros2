/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "WaitForTraffic.hpp"

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
auto WaitForTraffic::Standby::make(
  agv::RobotContextPtr context,
  rmf_traffic::Dependencies dependencies,
  rmf_traffic::Time expected_time,
  const AssignIDPtr& id,
  std::function<void()> update) -> std::shared_ptr<Standby>
{
  auto standby = std::make_shared<Standby>();
  standby->_context = std::move(context);
  standby->_dependencies = std::move(dependencies);
  standby->_expected_time = expected_time;
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(), "Wait for traffic", "",
    rmf_task::Event::Status::Standby, {}, standby->_context->clock());
  standby->_update = std::move(update);

  return standby;
}

//==============================================================================
auto WaitForTraffic::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration WaitForTraffic::Standby::duration_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto WaitForTraffic::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  return Active::make(
    _context,
    _dependencies,
    _expected_time,
    _state,
    _update,
    std::move(finished));
}

//==============================================================================
auto WaitForTraffic::Active::make(
  agv::RobotContextPtr context,
  const rmf_traffic::Dependencies& dependencies,
  rmf_traffic::Time expected_time,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::make_shared<Active>();
  active->_context = std::move(context);
  active->_expected_time = expected_time;
  active->_state = std::move(state);
  active->_update = std::move(update);
  active->_finished = std::move(finished);

  const auto consider_going = [w = active->weak_from_this()]()
    {
      if (const auto self = w.lock())
      {
        self->_context->worker().schedule(
          [w = self->weak_from_this()](const auto&)
          {
            if (const auto self = w.lock())
              self->_consider_going();
          });
      }
    };

  active->_dependencies.reserve(dependencies.size());
  bool all_reached_already = true;
  bool one_deprecated = false;
  std::unordered_set<rmf_traffic::ParticipantId> waiting_for_participants;
  for (const auto& dep : dependencies)
  {
    active->_dependencies.push_back(
      active->_context->schedule()->watch_dependency(
        dep, consider_going, consider_going));

    const auto& sub = active->_dependencies.back();
    if (!sub.reached())
    {
      all_reached_already = false;
      waiting_for_participants.insert(dep.on_participant);
    }

    if (sub.deprecated())
      one_deprecated = true;
  }

  for (const auto p : waiting_for_participants)
  {
    const auto participant = active->_context->schedule()->get_participant(p);
    if (participant)
    {
      active->_state->update_log().info(
        "Waiting for [robot:" + participant->name() + "]");
    }
  }

  if (all_reached_already || one_deprecated)
    consider_going();

  active->_timer = active->_context->node()->try_create_wall_timer(
    std::chrono::seconds(1), consider_going);

  return active;
}

//==============================================================================
auto WaitForTraffic::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration WaitForTraffic::Active::remaining_time_estimate() const
{
  const auto estimate = _expected_time - _context->now();
  if (estimate.count() > 0)
    return estimate;

  return rmf_traffic::Duration(0);
}

//==============================================================================
auto WaitForTraffic::Active::backup() const -> Backup
{
  // WaitForTraffic does not do backups
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto WaitForTraffic::Active::interrupt(std::function<void()>) -> Resume
{
  _decision_made = true;
  // WaitForTraffic is not designed to resume after interrupting.
  // That is handled by GoToPlace.
  return Resume::make([]() { /* do nothing */ });
}

//==============================================================================
void WaitForTraffic::Active::cancel()
{
  _decision_made = true;
  _state->update_log().info("Received signal to cancel");
  _state->update_status(Status::Canceled);
  _finished();
}

//==============================================================================
void WaitForTraffic::Active::kill()
{
  _decision_made = true;
  _state->update_log().info("Received signal to kill");
  _state->update_status(Status::Killed);
  _finished();
}

//==============================================================================
void WaitForTraffic::Active::_consider_going()
{
  if (_decision_made)
    return;

  bool all_dependencies_reached = true;
  for (const auto& dep : _dependencies)
  {
    if (dep.deprecated())
    {
      const auto other =
        _context->schedule()->snapshot()
        ->get_participant(dep.dependency().on_participant);
      if (!other)
      {
        _state->update_log().info(
          "Replanning because a traffic dependency was dropped from the "
          "schedule");
      }
      else
      {
        _state->update_log().info(
          "Replanning because [robot:" + other->name() + "] changed its plan");
      }

      return _replan();
    }

    if (!dep.reached())
      all_dependencies_reached = false;
  }

  if (all_dependencies_reached)
  {
    _decision_made = true;
    _state->update_status(Status::Completed);
    _state->update_log().info("All traffic dependencies satisfied");
    return _finished();
  }

  const auto delay = _context->now() - _expected_time;
  if (std::chrono::seconds(30) < delay)
  {
    // TODO(MXG): Make the max waiting time configurable
    _state->update_status(Status::Delayed);
    _state->update_log().info(
      "Replanning because a traffic dependency is excessively delayed");
    return _replan();
  }

  if (_context->itinerary().delay() < delay)
    _context->itinerary().delay(delay - _context->itinerary().delay());
}

//==============================================================================
void WaitForTraffic::Active::_replan()
{
  if (_decision_made)
    return;

  _decision_made = true;
  _context->request_replan();
}

} // namespace events
} // namespace rmf_fleet_adapter
