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
  active->_state->update_status(Status::Underway);
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
  using MutexGroupStatesPtr =
    std::shared_ptr<rmf_fleet_msgs::msg::MutexGroupStates>;

  const auto t_buffer = std::chrono::seconds(10);
  const auto zero = Eigen::Vector3d::Zero();
  rmf_traffic::Trajectory hold_traj;
  hold_traj.insert(_data.hold_time, _data.hold_position, zero);
  hold_traj.insert(_data.hold_time + t_buffer, _data.hold_position, zero);
  rmf_traffic::Route hold(_data.hold_map, std::move(hold_traj));
  _schedule({hold});

  const auto cumulative_delay = _context->now() - _data.hold_time;
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
            if (assignment.claimed == self->_context->requester_id())
            {
              const auto finished = self->_finished;
              self->_finished = nullptr;
              if (finished)
              {
                self->_schedule(*self->_data.resume_itinerary);
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

  _context->set_mutex_group(_data.mutex_group);
}

//==============================================================================
void LockMutexGroup::Active::_schedule(
  rmf_traffic::schedule::Itinerary itinerary) const
{
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
