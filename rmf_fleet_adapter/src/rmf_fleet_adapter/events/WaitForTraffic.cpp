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
  PlanIdPtr plan_id,
  rmf_traffic::Dependencies dependencies,
  rmf_traffic::Time expected_time,
  const AssignIDPtr& id,
  std::function<void()> update) -> std::shared_ptr<Standby>
{
  auto standby = std::make_shared<Standby>();
  standby->_context = std::move(context);
  standby->_plan_id = plan_id;
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
  RCLCPP_INFO(
    _context->node()->get_logger(),
    "[%s] waiting for traffic",
    _context->requester_id().c_str());
  rmf_traffic::PlanId plan_id = 0;
  if (_plan_id)
  {
    plan_id = *_plan_id;
  }
  else
  {
    RCLCPP_ERROR(
      _context->node()->get_logger(),
      "No plan_id was provided for WaitForTraffic action for robot [%s]. This "
      "is a critical internal error, please report this bug to the RMF "
      "maintainers.",
      _context->requester_id().c_str());
  }

  return Active::make(
    _context,
    plan_id,
    _dependencies,
    _expected_time,
    _state,
    _update,
    std::move(finished));
}

//==============================================================================
auto WaitForTraffic::Active::make(
  agv::RobotContextPtr context,
  const rmf_traffic::PlanId plan_id,
  const rmf_traffic::Dependencies& dependencies,
  rmf_traffic::Time expected_time,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  using MutexGroupRequestPtr =
    std::shared_ptr<rmf_fleet_msgs::msg::MutexGroupRequest>;

  auto active = std::make_shared<Active>();
  active->_context = std::move(context);
  active->_plan_id = plan_id;
  active->_expected_time = expected_time;
  active->_state = std::move(state);
  active->_update = std::move(update);
  active->_finished = std::move(finished);

  active->_mutex_group_listener = active->_context->node()
    ->mutex_group_request_obs()
    .observe_on(rxcpp::identity_same_worker(active->_context->worker()))
    .subscribe([w = active->weak_from_this()](const MutexGroupRequestPtr& msg)
      {
        const auto self = w.lock();
        if (!self)
          return;

        if (msg->claimant == self->_context->participant_id())
        {
          // We can ignore our own mutex group requests
          return;
        }

        if (self->_context->locked_mutex_groups().count(msg->group) > 0)
        {
          // If another participant is waiting to lock a mutex that we have
          // already locked, then we must delete any dependencies related to
          // that participant.
          auto r_it = std::remove_if(
            self->_dependencies.begin(),
            self->_dependencies.end(),
            [&](const DependencySubscription& d)
            {
              return d.dependency().on_participant == msg->claimant;
            });
          self->_dependencies.erase(r_it, self->_dependencies.end());
        }
      });

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
  _decision_made = std::chrono::steady_clock::now();
  // WaitForTraffic is not designed to resume after interrupting.
  // That is handled by GoToPlace.
  return Resume::make([]() { /* do nothing */ });
}

//==============================================================================
void WaitForTraffic::Active::cancel()
{
  _decision_made = std::chrono::steady_clock::now();
  _state->update_log().info("Received signal to cancel");
  _state->update_status(Status::Canceled);
  _finished();
}

//==============================================================================
void WaitForTraffic::Active::kill()
{
  _decision_made = std::chrono::steady_clock::now();
  _state->update_log().info("Received signal to kill");
  _state->update_status(Status::Killed);
  _finished();
}

//==============================================================================
void WaitForTraffic::Active::_consider_going()
{
  if (_decision_made)
  {
    const auto time_lapse = std::chrono::steady_clock::now() - *_decision_made;
    if (time_lapse > std::chrono::seconds(10))
    {
      RCLCPP_WARN(
        _context->node()->get_logger(),
        "[WaitForTraffic] excessive time lapse of %fs after a decision should "
        "have been made. Triggering a replan to recover.",
        rmf_traffic::time::to_seconds(time_lapse));
      _replan();
    }

    return;
  }

  bool all_dependencies_reached = true;
  for (const auto& dep : _dependencies)
  {
    if (!dep.reached() && !dep.deprecated())
      all_dependencies_reached = false;
  }

  if (all_dependencies_reached)
  {
    _decision_made = std::chrono::steady_clock::now();
    _state->update_status(Status::Completed);
    _state->update_log().info("All traffic dependencies satisfied");
    RCLCPP_INFO(
      _context->node()->get_logger(),
      "[%s] done waiting for traffic",
      _context->requester_id().c_str());
    return _finished();
  }

  using namespace std::chrono_literals;
  const auto now = _context->now();
  const auto cumulative_delay = now - _expected_time;
  if (30s < cumulative_delay)
  {
    // TODO(MXG): Make the max waiting time configurable
    _state->update_status(Status::Delayed);
    _state->update_log().info(
      "Replanning because a traffic dependency is excessively delayed");
    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Replanning for [%s] because a traffic dependency is excessively delayed",
      _context->requester_id().c_str());
    return _replan();
  }

  const auto current_delay = _context->itinerary().cumulative_delay(_plan_id);
  if (current_delay.has_value() && *current_delay < cumulative_delay)
  {
    _context->itinerary().cumulative_delay(_plan_id, cumulative_delay, 500ms);
  }
}

//==============================================================================
void WaitForTraffic::Active::_replan()
{
  _decision_made = std::chrono::steady_clock::now();
  _context->request_replan();
}

} // namespace events
} // namespace rmf_fleet_adapter
