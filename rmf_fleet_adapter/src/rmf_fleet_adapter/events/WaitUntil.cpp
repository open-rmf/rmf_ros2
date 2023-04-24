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

#include "WaitUntil.hpp"

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
auto WaitUntil::Standby::make(
  agv::RobotContextPtr context,
  rmf_traffic::Time until_time,
  const AssignIDPtr& id,
  std::function<void()> update) -> std::shared_ptr<Standby>
{
  auto standby = std::make_shared<Standby>();
  standby->_context = std::move(context);
  standby->_until_time = until_time;
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(), "Wait until time", "",
    rmf_task::Event::Status::Standby, {}, standby->_context->clock());
  standby->_update = std::move(update);

  return standby;
}

//==============================================================================
auto WaitUntil::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration WaitUntil::Standby::duration_estimate() const
{
  // TODO(MXG): It would be better to give back the difference between the
  // estimated finish time of the previous activity and the _until_time
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto WaitUntil::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  return Active::make(
    _context,
    _until_time,
    _state,
    _update,
    std::move(finished));
}

//==============================================================================
auto WaitUntil::Active::make(
  agv::RobotContextPtr context,
  const rmf_traffic::Time until_time,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::make_shared<Active>();
  active->_context = std::move(context);
  active->_until_time = until_time;
  active->_state = std::move(state);
  active->_update = std::move(update);
  active->_finished = std::move(finished);
  active->_update_waiting();

  active->_timer = active->_context->node()->try_create_wall_timer(
    std::chrono::milliseconds(200),
    [w = active->weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_update_waiting();
    });

  const auto now = active->_context->now();
  const auto duration_sec = rmf_traffic::time::to_seconds(
    active->_until_time - now);

  if (duration_sec > 0.0)
  {
    active->_state->update_log().info(
      "Waiting for the next " + std::to_string((int)(duration_sec)) + " sec");
    active->_state->update_status(Status::Underway);
  }

  return active;
}

//==============================================================================
auto WaitUntil::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration WaitUntil::Active::remaining_time_estimate() const
{
  const auto remaining = _until_time - _context->now();
  if (remaining.count() > 0)
    return remaining;

  return rmf_traffic::Duration(0);
}

//==============================================================================
auto WaitUntil::Active::backup() const -> Backup
{
  return Backup::make(0, {});
}

//==============================================================================
auto WaitUntil::Active::interrupt(std::function<void()> task_is_interrupted)
-> Resume
{
  _is_interrupted = true;
  _state->update_log().info("Interrupted");
  _state->update_status(Status::Standby);
  _context->worker().schedule(
    [task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });

  return Resume::make(
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_is_interrupted = false;
      self->_state->update_log().info("Resuming");
      self->_state->update_status(Status::Underway);
    });
}

//==============================================================================
void WaitUntil::Active::cancel()
{
  _state->update_log().info("Received signal to cancel");
  _state->update_status(Status::Canceled);
  if (_finished)
  {
    auto finish = _finished;
    _finished = nullptr;
    finish();
  }
}

//==============================================================================
void WaitUntil::Active::kill()
{
  _state->update_log().info("Received signal to kill");
  _state->update_status(Status::Killed);
  if (_finished)
  {
    auto finish = _finished;
    _finished = nullptr;
    finish();
  }
}

//==============================================================================
void WaitUntil::Active::_update_waiting()
{
  if (_is_interrupted)
    return;

  const auto now = _context->now();
  if (_until_time <= now)
  {
    if (_finished)
    {
      _state->update_log().info("Reached the wait time");
      _state->update_status(Status::Completed);

      auto finish = _finished;
      _finished = nullptr;
      finish();
    }

    return;
  }

  const Eigen::Vector3d position = _context->position();
  if (_last_position.has_value())
  {
    const Eigen::Vector2d p1 = position.block<2, 1>(0, 0);
    const Eigen::Vector2d p0 = _last_position->block<2, 1>(0, 0);
    if ((p1 - p0).norm() > 1e-2)
      _update_holding(now, position);
  }
  else
  {
    _update_holding(now, position);
  }
}

//==============================================================================
void WaitUntil::Active::_update_holding(
  rmf_traffic::Time now,
  Eigen::Vector3d position)
{
  _last_position = position;
  rmf_traffic::Trajectory trajectory;
  trajectory.insert(now, position, Eigen::Vector3d::Zero());
  trajectory.insert(_until_time, position, Eigen::Vector3d::Zero());

  _context->itinerary().set(
    _context->itinerary().assign_plan_id(),
    {{_context->map(), std::move(trajectory)}});
}

} // namespace events
} // namespace rmf_fleet_adapter
