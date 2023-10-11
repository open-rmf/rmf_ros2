/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "WaitForCancel.hpp"

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
auto WaitForCancel::Standby::make(
  agv::RobotContextPtr context,
  const AssignIDPtr& id)
-> std::shared_ptr<Standby>
{
  auto standby = std::shared_ptr<Standby>(new Standby);
  standby->_context = std::move(context);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    "Wait for cancel",
    "This task will remain active until it gets canceled",
    rmf_task::Event::Status::Standby, {}, standby->_context->clock());
  return standby;
}

//==============================================================================
auto WaitForCancel::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration WaitForCancel::Standby::duration_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto WaitForCancel::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  return Active::make(
    _context,
    _state,
    std::move(finished));
}

//==============================================================================
auto WaitForCancel::Active::make(
  agv::RobotContextPtr context,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::shared_ptr<Active>(new Active);
  active->_context = std::move(context);
  active->_finished = std::move(finished);
  active->_state = std::move(state);
  active->_state->update_status(Status::Underway);
  return active;
}

//==============================================================================
auto WaitForCancel::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration WaitForCancel::Active::remaining_time_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto WaitForCancel::Active::backup() const -> Backup
{
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto WaitForCancel::Active::interrupt(std::function<void()> task_is_interrupted)
-> Resume
{
  _context->worker().schedule([task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });
  return Resume::make([]() { /* do nothing */});
}

//==============================================================================
void WaitForCancel::Active::cancel()
{
  _state->update_status(Status::Canceled);
  _context->worker().schedule([finished = _finished](const auto&)
    {
      finished();
    });
}

//==============================================================================
void WaitForCancel::Active::kill()
{
  cancel();
}

} // namespace events
} // namespace rmf_fleet_adapter
