/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "Error.hpp"

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
auto Error::Standby::make(
  rmf_task::events::SimpleEventStatePtr state,
  Behavior behavior) -> std::shared_ptr<Standby>
{
  auto standby = std::make_shared<Standby>();
  standby->_state = std::move(state);
  standby->_behavior = behavior;

  // Make sure the status is set to an error
  standby->_state->update_status(Status::Error);

  return standby;
}

//==============================================================================
auto Error::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration Error::Standby::duration_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto Error::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  if (!_active)
  {
    _active = Active::make(_state, std::move(finished), _behavior);
  }

  return _active;
}

//==============================================================================
auto Error::Active::make(
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> finished,
  Behavior behavior) -> std::shared_ptr<Active>
{
  auto active = std::make_shared<Active>();
  active->_state = std::move(state);
  active->_finished = std::move(finished);

  active->_state->update_status(Status::Error);

  if (behavior == Behavior::Continue)
    active->_finished();

  return active;
}

//==============================================================================
auto Error::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration Error::Active::remaining_time_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto Error::Active::backup() const -> Backup
{
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto Error::Active::interrupt(std::function<void()> task_is_interrupted)
-> Resume
{
  task_is_interrupted();
  return Resume::make([]() {});
}

//==============================================================================
void Error::Active::cancel()
{
  _finished();
}

//==============================================================================
void Error::Active::kill()
{
  _finished();
}

} // namespace events
} // namespace rmf_fleet_adapter
