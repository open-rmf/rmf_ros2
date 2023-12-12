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
  auto standby = std::shared_ptr<Standby>(new Standby(std::move(data)));
  standby->_context = std::move(context);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    "Lock mutex groups " + data.all_groups_str(),
    "Waiting for the mutex groups to be locked",
    rmf_task::Event::Status::Standby, {}, standby->_context->clock());
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
LockMutexGroup::Standby::Standby(Data data)
: _data(data)
{
  // Do nothing
}

//==============================================================================
auto LockMutexGroup::Active::make(
  agv::RobotContextPtr context,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> finished,
  Data data) -> std::shared_ptr<Active>
{
  auto active = std::shared_ptr<Active>(new Active(std::move(data)));
  active->_context = std::move(context);
  active->_state = std::move(state);
  active->_finished = std::move(finished);
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
LockMutexGroup::Active::Active(Data data)
: _data(std::move(data))
{
  // Do nothing
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
  _context->schedule_hold(
    _data.plan_id,
    _data.hold_time,
    std::chrono::seconds(5),
    _data.hold_position,
    _data.hold_map);
  _stubborn = _context->be_stubborn();

  _state->update_log().info(
    "Waiting to lock mutex group " + _data.all_groups_str());
  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Waiting to lock mutex groups %s for robot [%s]",
    _data.all_groups_str().c_str(),
    _context->requester_id().c_str());

  const auto cumulative_delay = _context->now() - _data.hold_time;
  _context->itinerary().cumulative_delay(*_data.plan_id, cumulative_delay);

  _delay_timer = _context->node()->try_create_wall_timer(
    std::chrono::seconds(1),
    [weak = weak_from_this(), plan_id = *_data.plan_id]()
    {
      const auto self = weak.lock();
      if (!self)
        return;

      self->_apply_cumulative_delay();
    });

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
          const auto finished = self->_finished;
          self->_finished = nullptr;
          if (!finished)
            return;

          const auto now = self->_context->now();
          const auto delay = now - self->_data.hold_time;
          if (delay > std::chrono::seconds(2))
          {
            const auto start = [&]()
            -> std::optional<rmf_traffic::agv::Plan::Start>
            {
              for (const auto& wp : self->_data.waypoints)
              {
                if (wp.graph_index().has_value())
                {
                  return rmf_traffic::agv::Plan::Start(
                    self->_context->now(),
                    *wp.graph_index(),
                    wp.position()[2]);
                }
              }
              return std::nullopt;
            }();

            if (start.has_value())
            {
              self->_find_path_service = std::make_shared<services::FindPath>(
                self->_context->planner(),
                std::vector<rmf_traffic::agv::Plan::Start>({*start}),
                self->_data.goal,
                self->_context->schedule()->snapshot(),
                self->_context->itinerary().id(),
                self->_context->profile(),
                std::chrono::seconds(5));

              self->_plan_subscription =
              rmf_rxcpp::make_job<services::FindPath::Result>(
                self->_find_path_service)
              .observe_on(
                rxcpp::identity_same_worker(self->_context->worker()))
              .subscribe(
                [w = self->weak_from_this(), finished](
                  const services::FindPath::Result& result)
                {
                  const auto self = w.lock();
                  if (!self)
                    return;

                  if (self->_consider_plan_result(result))
                  {
                    // We have a matching plan so proceed
                    RCLCPP_INFO(
                      self->_context->node()->get_logger(),
                      "Finished locking mutexes %s for [%s] and plan is "
                      "unchanged after waiting",
                      self->_data.all_groups_str().c_str(),
                      self->_context->requester_id().c_str());

                    self->_schedule(*self->_data.resume_itinerary);
                    self->_apply_cumulative_delay();
                    self->_state->update_status(Status::Completed);
                    finished();
                    return;
                  }

                  // The new plan was not a match, so we should trigger a
                  // proper replan.
                  self->_state->update_status(Status::Completed);
                  self->_context->request_replan();
                });

              self->_find_path_timeout =
              self->_context->node()->try_create_wall_timer(
                std::chrono::seconds(10),
                [
                  weak_service = self->_find_path_service->weak_from_this()
                ]()
                {
                  if (const auto service = weak_service.lock())
                  {
                    service->interrupt();
                  }
                });
              return;
            }
          }

          // We locked the mutex quickly enough that we should proceed.
          RCLCPP_INFO(
            self->_context->node()->get_logger(),
            "Finished locking mutexes %s for [%s]",
            self->_data.all_groups_str().c_str(),
            self->_context->requester_id().c_str());

          self->_schedule(*self->_data.resume_itinerary);
          self->_apply_cumulative_delay();
          self->_state->update_status(Status::Completed);
          finished();
          return;
        }
      });
}

//==============================================================================
void LockMutexGroup::Active::_schedule(
  rmf_traffic::schedule::Itinerary itinerary) const
{
  _context->schedule_itinerary(_data.plan_id, std::move(itinerary));
}

//==============================================================================
void LockMutexGroup::Active::_apply_cumulative_delay()
{
  const auto cumulative_delay = _context->now() - _data.hold_time;
  _context->itinerary().cumulative_delay(*_data.plan_id, cumulative_delay);
}

//==============================================================================
namespace {
std::vector<std::size_t> filter_graph_indices(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints)
{
  std::vector<std::size_t> output;
  output.reserve(waypoints.size());
  for (const auto& wp : waypoints)
  {
    if (wp.graph_index().has_value())
    {
      if (*wp.graph_index() != output.back())
      {
        output.push_back(*wp.graph_index());
      }
    }
  }
  return output;
}
} // anonymous namespace

//==============================================================================
bool LockMutexGroup::Active::_consider_plan_result(
  services::FindPath::Result result)
{
  if (!result.success())
  {
    RCLCPP_WARN(
      _context->node()->get_logger(),
      "Replanning for [%s] after locking mutexes %s because the path to the "
      "goal has become blocked.",
      _context->requester_id().c_str(),
      _data.all_groups_str().c_str());
    return false;
  }

  const auto original_sequence = filter_graph_indices(_data.waypoints);
  const auto new_sequence = filter_graph_indices(result->get_waypoints());
  if (original_sequence != new_sequence)
  {
    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Replanning for [%s] after locking mutexes %s because the external "
      "traffic has substantially changed.",
      _context->requester_id().c_str(),
      _data.all_groups_str().c_str());
    return false;
  }

  return true;
}

} // namespace events
} // namespace rmf_fleet_adapter
