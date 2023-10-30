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

#include "LockMutexGroup.hpp"

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
auto LockMutexGroup::Standby::make(
  agv::RobotContextPtr context,
  const AssignIDPtr& id,
  std::string mutex_group)
-> std::shared_ptr<Standby>
{
  auto standby = std::shared_ptr<Standby>(new Standby);
  standby->_context = std::move(context);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    "Lock mutex group [" + mutex_group + "]",
    "Waiting for the mutex group to be locked",
    rmf_task::Event::Status::Standby, {}, standby->_context->clock());
  standby->_mutex_group = std::move(mutex_group);
  return standby;
}

//==============================================================================
auto LockMutexGroup::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration LockMutexGroup::Standby::duration_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto LockMutexGroup::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  return Active::make(_context, _state, std::move(finished), _mutex_group);
}

//==============================================================================
auto LockMutexGroup::Active::make(
  agv::RobotContextPtr context,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> finished,
  std::string mutex_group) -> std::shared_ptr<Active>
{
  auto active = std::shared_ptr<Active>(new Active);
  active->_context = std::move(context);
  active->_state = std::move(state);
  active->_finished = std::move(finished);
  active->_mutex_group = std::move(mutex_group);
  active->_state->update_status(Status::Underway);

  using MutexGroupStatesPtr =
    std::shared_ptr<rmf_fleet_msgs::msg::MutexGroupStates>;

  active->_listener = active->_context->node()->mutex_group_states()
    .observe_on(rxcpp::identity_same_worker(active->_context->worker()))
    .subscribe(
      [weak = active->weak_from_this()](
        const MutexGroupStatesPtr& mutex_group_states)
      {
        const auto self = weak.lock();
        if (!self)
          return;

        for (const auto& assignment : mutex_group_states->assignments)
        {
          if (assignment.group == self->_mutex_group)
          {
            if (assignment.claimed == self->_context->requester_id())
            {
              const auto finished = self->_finished;
              self->_finished = nullptr;
              if (finished)
              {
                finished();
                return;
              }
            }
          }
        }
      });

  active->_context->set_mutex_group(active->_mutex_group);

  return active;
}

//==============================================================================
auto LockMutexGroup::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration LockMutexGroup::Active::remaining_time_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto LockMutexGroup::Active::backup() const -> Backup
{
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto LockMutexGroup::Active::interrupt(
  std::function<void()> task_is_interrupted) -> Resume
{
  _context->worker().schedule([task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });
  return Resume::make([]() { /* do nothing */ });
}

//==============================================================================
void LockMutexGroup::Active::cancel()
{
  _state->update_status(Status::Canceled);
  _context->worker().schedule([finished = _finished](const auto&)
    {
      finished();
    });
}

//==============================================================================
void LockMutexGroup::Active::kill()
{
  cancel();
}

} // namespace events
} // namespace rmf_fleet_adapter
