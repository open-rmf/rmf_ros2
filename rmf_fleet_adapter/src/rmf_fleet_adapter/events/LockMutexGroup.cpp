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
  Data data)
-> std::shared_ptr<Standby>
{
  auto standby = std::shared_ptr<Standby>(new Standby);
  standby->_context = std::move(context);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    "Lock mutex group [" + data.mutex_group + "]",
    "Waiting for the mutex group to be locked",
    rmf_task::Event::Status::Standby, {}, standby->_context->clock());
  standby->_data = std::move(data);
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
  return Active::make(_context, _state, std::move(finished), _data);
}

//==============================================================================
auto LockMutexGroup::Active::make(
  agv::RobotContextPtr context,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> finished,
  Data data) -> std::shared_ptr<Active>
{
  auto active = std::shared_ptr<Active>(new Active);
  active->_context = std::move(context);
  active->_state = std::move(state);
  active->_finished = std::move(finished);
  active->_data = std::move(data);
  active->_initialize();

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

//==============================================================================
void LockMutexGroup::Active::_initialize()
{
  _state->update_status(State::Status::Underway);
  using MutexGroupStatesPtr =
    std::shared_ptr<rmf_fleet_msgs::msg::MutexGroupStates>;

  if (_context->locked_mutex_group() == _data.mutex_group)
  {
    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Mutex group [%s] was already locked for [%s]",
      _data.mutex_group.c_str(),
      _context->requester_id().c_str());

    _schedule(*_data.resume_itinerary);
    // We don't need to do anything further, we already got the mutex group
    // previously.
    _context->worker().schedule(
      [finished = _finished](const auto&)
      {
        finished();
      });
    return;
  }

  _state->update_log().info(
    "Waiting to lock mutex group [" + _data.mutex_group + "]");
  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Waiting to lock mutex group [%s] for robot [%s]",
    _data.mutex_group.c_str(),
    _context->requester_id().c_str());

  const auto cumulative_delay = _context->now() - _data.hold_time
    - std::chrono::seconds(2);
  _context->itinerary().cumulative_delay(*_data.plan_id, cumulative_delay);

  _listener = _context->node()->mutex_group_states()
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
      [weak = weak_from_this()](
        const MutexGroupStatesPtr& mutex_group_states)
      {
        const auto self = weak.lock();
        if (!self)
          return;

        for (const auto& assignment : mutex_group_states->assignments)
        {
          if (assignment.group == self->_data.mutex_group)
          {
            if (assignment.claimant == self->_context->participant_id())
            {
              const auto finished = self->_finished;
              self->_finished = nullptr;
              if (finished)
              {
                if (!self->_data.resume_itinerary->empty())
                {
                  self->_schedule(*self->_data.resume_itinerary);
                }
                std::cout << " === LOCKED MUTEX " << __LINE__ << " new plan id for "
                  << self->_context->requester_id() << ": "
                  << *self->_data.plan_id << std::endl;
                self->_state->update_status(State::Status::Completed);
                RCLCPP_INFO(
                  self->_context->node()->get_logger(),
                  "Finished locking mutex group [%s] for robot [%s]",
                  self->_data.mutex_group.c_str(),
                  self->_context->requester_id().c_str());
                finished();
                return;
              }
            }
          }
        }
      });

  _delay_timer = _context->node()->try_create_wall_timer(
    std::chrono::seconds(1),
    [weak = weak_from_this(), plan_id = *_data.plan_id]()
    {
      const auto self = weak.lock();
      if (!self)
        return;

      const auto cumulative_delay =
        self->_context->now() - self->_data.hold_time;
      self->_context->itinerary().cumulative_delay(plan_id, cumulative_delay);
    });

  std::cout << " ===== SETTING MUTEX GROUP FOR " << _context->requester_id().c_str()
    << " TO " << _data.mutex_group << std::endl;
  _context->request_mutex_group(_data.mutex_group, _data.hold_time);
}

//==============================================================================
void LockMutexGroup::Active::_schedule(
  rmf_traffic::schedule::Itinerary itinerary) const
{
  std::cout << " --- [" << _context->requester_id() << "] resuming with "
    << itinerary.size() << " routes" << std::endl;

  bool scheduled = false;
  std::size_t attempts = 0;
  while (!scheduled)
  {
    if (++attempts > 5)
    {
      RCLCPP_ERROR(
        _context->node()->get_logger(),
        "Repeatedly failled attempts to update schedule during LockMutexGroup "
        "action for robot [%s]. Last attempted value was [%s]. We will "
        "continue without updating the traffic schedule. This could lead to "
        "traffic management problems. Please report this bug to the "
        "maintainers of RMF.",
        _context->requester_id().c_str(),
        *_data.plan_id);
        break;
    }

    *_data.plan_id = _context->itinerary().assign_plan_id();
    scheduled = _context->itinerary().set(*_data.plan_id, itinerary);

    if (!scheduled)
    {
      RCLCPP_ERROR(
        _context->node()->get_logger(),
        "Invalid plan_id [%lu] when current plan_id is [%lu] for robot [%s] "
        "while performing a LockMutexGroup. Please report this bug to an RMF "
        "developer.",
        *_data.plan_id,
        _context->itinerary().current_plan_id(),
        _context->requester_id().c_str());
    }
  }
}

} // namespace events
} // namespace rmf_fleet_adapter
