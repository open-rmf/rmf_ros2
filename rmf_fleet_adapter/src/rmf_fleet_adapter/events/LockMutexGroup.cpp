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
#include <iostream>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
std::string all_str(const std::unordered_set<std::string>& all)
{
  std::stringstream ss;
  for (const auto& item : all)
  {
    ss << "[" << item << "]";
  }
  return ss.str();
}

//==============================================================================
std::string LockMutexGroup::Data::all_groups_str() const
{
  return all_str(mutex_groups);
}

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
    "Lock mutex groups " + data.all_groups_str(),
    "Waiting for the mutex groups to be locked",
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
  const auto finished = _finished;
  _finished = nullptr;
  _context->worker().schedule([finished](const auto&)
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

  _remaining = _data.mutex_groups;
  for (const auto& [locked, _] : _context->locked_mutex_groups())
  {
    _remaining.erase(locked);
  }

  if (_remaining.empty())
  {
    RCLCPP_INFO(
      _context->node()->get_logger(),
      "All mutex groups were already locked for [%s]",
      _context->requester_id().c_str());

    _schedule(*_data.resume_itinerary);
    // We don't need to do anything further, we already got the mutex group
    // previously.
    _context->worker().schedule(
      [state = _state, finished = _finished](const auto&)
      {
        state->update_status(State::Status::Completed);
        finished();
      });
    return;
  }

  *_data.plan_id += 1;
  rmf_traffic::Trajectory hold;
  const auto zero = Eigen::Vector3d::Zero();
  const auto wait = std::chrono::seconds(5);
  hold.insert(_data.hold_time, _data.hold_position, zero);
  hold.insert(_data.hold_time + wait, _data.hold_position, zero);
  _stubborn = _context->be_stubborn();
  _schedule({rmf_traffic::Route(_data.hold_map, std::move(hold))});

  _state->update_log().info(
    "Waiting to lock mutex group " + _data.all_groups_str());
  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Waiting to lock mutex groups %s for robot [%s]",
    _data.all_groups_str().c_str(),
    _context->requester_id().c_str());

  const auto cumulative_delay = _context->now() - _data.hold_time;
  std::cout << __FILE__ << ": " << __LINE__ << "!!!!!" << std::endl;
  _context->itinerary().cumulative_delay(*_data.plan_id, cumulative_delay);

  _delay_timer = _context->node()->try_create_wall_timer(
    std::chrono::seconds(1),
    [weak = weak_from_this(), plan_id = *_data.plan_id]()
    {
      const auto self = weak.lock();
      if (!self)
        return;

      const auto cumulative_delay =
        self->_context->now() - self->_data.hold_time;
      std::cout << __FILE__ << ": " << __LINE__ << "!!!!!" << std::endl;
      self->_context->itinerary().cumulative_delay(plan_id, cumulative_delay);
    });

  std::cout << " ===== SETTING MUTEX GROUP FOR " << _context->requester_id().c_str()
    << " TO " << _data.all_groups_str() << " AT " << _data.hold_position.transpose() << std::endl;

  _listener = _context->request_mutex_groups(
    _data.mutex_groups, _data.hold_time)
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe([w = weak_from_this()](const std::string& locked)
      {
        const auto self = w.lock();
        if (!self)
          return;

        self->_remaining.erase(locked);
        if (self->_remaining.empty())
        {
          const auto now = self->_context->now();
          const auto delay = now - self->_data.hold_time;
          if (delay > std::chrono::seconds(2))
          {
            RCLCPP_INFO(
              self->_context->node()->get_logger(),
              "Replanning for [%s] after a delay of %0.2fs in locking mutexes %s",
              self->_context->requester_id().c_str(),
              rmf_traffic::time::to_seconds(delay),
              self->_data.all_groups_str().c_str());
            self->_context->request_replan();
            self->_finished = nullptr;
            return;
          }

          // We locked the mutex quickly enough that we should proceed.
          const auto finished = self->_finished;
          self->_finished = nullptr;
          if (finished)
          {
            RCLCPP_INFO(
              self->_context->node()->get_logger(),
              "Finished locking mutexes %s for [%s]",
              self->_data.all_groups_str().c_str(),
              self->_context->requester_id().c_str());

            self->_schedule(*self->_data.resume_itinerary);
            self->_state->update_status(Status::Completed);
            finished();
            return;
          }
        }
      });
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
        "action for robot [%s]. Last attempted value was [%lu]. We will "
        "continue without updating the traffic schedule. This could lead to "
        "traffic management problems. Please report this bug to the "
        "maintainers of RMF.",
        _context->requester_id().c_str(),
        *_data.plan_id);
        break;
    }

    scheduled = _context->itinerary().set(*_data.plan_id, itinerary);

    if (!scheduled)
    {
      *_data.plan_id = _context->itinerary().assign_plan_id();
      if (attempts > 1)
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
}

} // namespace events
} // namespace rmf_fleet_adapter
