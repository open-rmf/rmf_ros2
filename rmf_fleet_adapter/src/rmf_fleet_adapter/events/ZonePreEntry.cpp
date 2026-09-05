/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "ZonePreEntry.hpp"
#include "ZonePostEntry.hpp"

#include "../phases/Utils.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>

namespace rmf_fleet_adapter {
namespace events {

// How often we re-send the entry request while waiting for an answer.
const auto WAITING_WARN_INTERVAL = std::chrono::seconds(10);

// How long before the warning points at the zone manager.
const auto SUSPECT_MANAGER_AFTER = std::chrono::seconds(45);

// How often we actually say we are still waiting.
const auto WAITING_LOG_INTERVAL = std::chrono::seconds(60);

//==============================================================================
auto ZonePreEntry::Standby::make(
  agv::RobotContextPtr context,
  const AssignIDPtr& id,
  Data data) -> std::shared_ptr<Standby>
{
  auto standby = std::shared_ptr<Standby>(new Standby(std::move(data)));
  standby->_context = std::move(context);
  standby->_assign_id = id;
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    "Enter zone [" + standby->_data.zone_name + "]",
    "Waiting for the zone manager to assign a waypoint",
    rmf_task::Event::Status::Standby, {}, standby->_context->clock());
  return standby;
}

//==============================================================================
auto ZonePreEntry::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration ZonePreEntry::Standby::duration_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto ZonePreEntry::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  return Active::make(
    _context, _assign_id, _state, std::move(finished), _data);
}

//==============================================================================
ZonePreEntry::Standby::Standby(Data data)
: _data(std::move(data))
{
  // Do nothing
}

//==============================================================================
auto ZonePreEntry::Active::make(
  agv::RobotContextPtr context,
  const AssignIDPtr& id,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> finished,
  Data data) -> std::shared_ptr<Active>
{
  auto active = std::shared_ptr<Active>(new Active(std::move(data)));
  active->_context = std::move(context);
  active->_assign_id = id;
  active->_state = std::move(state);
  active->_finished = std::move(finished);
  active->_initialize();
  return active;
}

//==============================================================================
auto ZonePreEntry::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration ZonePreEntry::Active::remaining_time_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto ZonePreEntry::Active::backup() const -> Backup
{
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto ZonePreEntry::Active::interrupt(
  std::function<void()> task_is_interrupted) -> Resume
{
  _context->worker().schedule([task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });
  return Resume::make([]() { /* do nothing */ });
}

//==============================================================================
ZonePreEntry::Active::Active(Data data)
: _data(std::move(data))
{
  // Do nothing
}

//==============================================================================
void ZonePreEntry::Active::_initialize()
{
  _state->update_status(Status::Underway);

  // Held for as long as this event runs, or the sweep would reclaim a
  // booking left behind by a cancelled GoToZone while we drive to it. On the
  // entry lane the robot is still outside the zone box, so our reference is
  // the only thing keeping the sweep off.
  _booking = _context->zone_booking(_data.zone_name);
  if (_booking)
    _entry_waypoint = _booking->waypoint_name;

  _state->update_log().info(
    "Finalizing our waypoint in zone [" + _data.zone_name + "]");

  const auto node = _context->node();

  auto qos = rclcpp::QoS(10).transient_local().reliable();
  _state_sub = node->create_subscription<rmf_zone_msgs::msg::ZoneState>(
    ZoneStateTopicName, qos,
    [w = weak_from_this()](const rmf_zone_msgs::msg::ZoneState::SharedPtr msg)
    {
      const auto self = w.lock();
      if (!self)
        return;

      const auto& zone_name = self->_data.zone_name;

      const auto result = phases::handle_zone_state(
        self->_context, *msg, zone_name, self->_current_request_id,
        "ZonePreEntry");

      if (result.status != phases::ZoneStateResult::Status::NoMatch)
        self->_had_any_answer = true;

      switch (result.status)
      {
        case phases::ZoneStateResult::Status::Proceed:
        {
          // Nothing was booked, so there is nothing to drive to and nothing
          // to re-aim. Finish and let the plan we already have carry on.
          self->_state->update_log().info(
            "entering zone [" + zone_name + "] without a booking");

          self->_resume_plan();

          self->_state_sub.reset();
          self->_delay_timer.reset();
          self->_request_timer.reset();
          self->_complete(Status::Completed);
          return;
        }

        case phases::ZoneStateResult::Status::Granted:
        {
          self->_state->update_log().info(
            "Assigned waypoint [" + result.waypoint_name + "]");

          // Re-seat in case this is the first booking we have been given.
          self->_booking = self->_context->zone_booking(zone_name);

          self->_state_sub.reset();
          self->_delay_timer.reset();
          self->_request_timer.reset();

          if (result.waypoint_name == self->_entry_waypoint)
          {
            // Nothing was re-aimed, so the plan is already going there and
            // can finish the lane itself. Hopping would leave the robot on
            // this lane, and the replan would trigger this event again.
            self->_resume_plan();
            self->_complete(Status::Completed);
            return;
          }

          self->_begin_move(*result.goal, result.waypoint_name);
          return;
        }

        case phases::ZoneStateResult::Status::UnknownWaypoint:
        {
          self->_state->update_log().error(
            "assigned waypoint [" + result.waypoint_name
            + "] is not in the navigation graph");
          self->_state_sub.reset();
          self->_delay_timer.reset();
          self->_request_timer.reset();
          self->_complete(Status::Failed);
          return;
        }

        case phases::ZoneStateResult::Status::TicketMismatch:
        {
          self->_state->update_log().error(
            "zone manager granted [" + result.waypoint_name + "] backed by a "
            "ticket for a different vertex, so it was not adopted");
          self->_state_sub.reset();
          self->_delay_timer.reset();
          self->_request_timer.reset();
          self->_complete(Status::Failed);
          return;
        }

        case phases::ZoneStateResult::Status::UnknownZone:
        {
          self->_state->update_log().error(
            "zone [" + zone_name + "] is unknown to the zone manager");
          self->_state_sub.reset();
          self->_delay_timer.reset();
          self->_request_timer.reset();
          self->_complete(Status::Failed);
          return;
        }

        case phases::ZoneStateResult::Status::ZoneUnusable:
        {
          self->_state->update_log().error(
            "zone [" + zone_name + "] cannot assign any waypoint ("
            + result.reason + ")");
          self->_state_sub.reset();
          self->_delay_timer.reset();
          self->_request_timer.reset();
          self->_complete(Status::Failed);
          return;
        }

        case phases::ZoneStateResult::Status::Deferred:
        {
          self->_state->update_status(Status::Blocked);
          self->_state->update_log().info(
            "waiting for a free waypoint in zone [" + zone_name + "]: "
            + result.reason);
          self->_has_pending_request = false;
          return;
        }

        case phases::ZoneStateResult::Status::NoMatch:
          break;
      }

      // Not our booking
      if (self->_has_pending_request)
        return;

      self->_publish_finalize_request();
    });

  _requested_at = _context->now();
  _publish_finalize_request();

  _request_timer = node->try_create_wall_timer(
    WAITING_WARN_INTERVAL,
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (self)
        self->_on_request_timer();
    });

  _delay_timer = node->try_create_wall_timer(
    std::chrono::milliseconds(1000),
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      const auto delay = self->_context->now() - self->_data.expected_finish;
      if (delay > std::chrono::seconds(0))
      {
        self->_context->worker().schedule(
          [
            context = self->_context,
            plan_id = *self->_data.plan_id,
            delay
          ](const auto&)
          {
            context->itinerary().cumulative_delay(plan_id, delay);
          });
      }
    });
}

//==============================================================================
void ZonePreEntry::Active::_on_request_timer()
{
  if (!_finished || !_requested_at)
    return;

  const auto now = _context->now();
  const auto elapsed = now - *_requested_at;
  const auto waited =
    std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

  const bool manager_silent =
    elapsed >= SUSPECT_MANAGER_AFTER && !_had_any_answer;

  const bool due = !_last_warned
    || now - *_last_warned >= WAITING_LOG_INTERVAL;

  if (due || (manager_silent && !_warned_manager_silent))
  {
    _last_warned = now;

    if (manager_silent)
    {
      _warned_manager_silent = true;
      RCLCPP_WARN(
        _context->node()->get_logger(),
        "ZonePreEntry: [%s/%s] has waited %lds at the boundary of zone [%s] "
        "and the zone manager has never answered. Is it running?",
        _context->group().c_str(), _context->name().c_str(), waited,
        _data.zone_name.c_str());
    }
    else
    {
      RCLCPP_WARN(
        _context->node()->get_logger(),
        "ZonePreEntry: [%s/%s] has been waiting %lds to enter zone [%s]",
        _context->group().c_str(), _context->name().c_str(), waited,
        _data.zone_name.c_str());
    }

    _state->update_log().info(
      "still waiting to enter zone [" + _data.zone_name + "] after "
      + std::to_string(waited) + "s");
  }

  if (!_has_pending_request)
    _publish_finalize_request();
}

//==============================================================================
void ZonePreEntry::Active::_begin_move(
  rmf_traffic::agv::Plan::Goal goal,
  const std::string& waypoint_name)
{
  const auto planner = _context->planner();
  if (!planner)
  {
    _state->update_log().error("No planner is available to enter the zone");
    _finish_with_replan();
    return;
  }

  const auto result = planner->plan(_context->location(), std::move(goal));

  if (!result.success())
  {
    _state->update_log().error(
      "No route to waypoint [" + waypoint_name + "]");

    _finish_with_replan();
    return;
  }

  if (result->get_waypoints().empty())
  {
    // Nothing to drive means we are standing on the vertex already
    _state->update_log().info(
      "Already on waypoint [" + waypoint_name + "]");

    _finish_at_waypoint();
    return;
  }

  auto hop_plan_id = std::make_shared<rmf_traffic::PlanId>(
    _context->itinerary().assign_plan_id());

  // update this new itinerary
  _context->schedule_itinerary(hop_plan_id, result->get_itinerary());

  _state->update_log().info("Moving to waypoint [" + waypoint_name + "]");

  _move = std::make_shared<phases::MoveRobot::ActivePhase>(
    _context, result->get_waypoints(), *hop_plan_id, std::nullopt);

  _move_sub = _move->observe()
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
    [w = weak_from_this()](const LegacyTask::StatusMsg& msg)
    {
      const auto self = w.lock();
      if (!self)
        return;

      if (msg.state == LegacyTask::StatusMsg::STATE_FAILED)
        self->_state->update_log().error(msg.status);
    },
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_finish_at_waypoint();
    });
}

//==============================================================================
void ZonePreEntry::Active::_finish_at_waypoint()
{
  if (!_finished)
    return;

  _post_entry = ZonePostEntry::Standby::make(
    _context, _assign_id, ZonePostEntry::Data{_data.zone_name})
    ->begin(
    []() {},
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_finish_with_replan();
    });
}

//==============================================================================
void ZonePreEntry::Active::_resume_plan()
{
  // ExecutePlan truncates the published itinerary at the boundary so the
  // robot holds nothing beyond it while it waits. The re-aimed path does not
  // need this back, since its hop and replan overwrite the itinerary anyway.
  if (_data.resume_itinerary)
    _context->schedule_itinerary(_data.plan_id, *_data.resume_itinerary);
}

//==============================================================================
void ZonePreEntry::Active::_finish_with_replan()
{
  if (!_finished)
    return;

  _context->request_replan();
  _complete(Status::Completed);
}

//==============================================================================
void ZonePreEntry::Active::_publish_finalize_request()
{
  _current_request_id = phases::generate_zone_request_id(
    _context->group(), _context->name(), _data.zone_name);

  rmf_zone_msgs::msg::ZoneModifiers modifiers;
  const auto preference = _context->zone_preference(_data.zone_name);
  if (preference)
    modifiers = *preference;

  using Context = rmf_zone_msgs::msg::ZoneEntryContext;
  auto entry_context = Context();
  entry_context.task_type =
    preference ? Context::TASK_ZONE : Context::TASK_OTHER;

  // Stopping here or crossing, decided by the zone the plan ends in.
  if (!_data.plan_end_zone.has_value())
    entry_context.zone_relation = Context::RELATION_UNKNOWN;
  else if (*_data.plan_end_zone == _data.zone_name)
    entry_context.zone_relation = Context::RELATION_DESTINATION;
  else
    entry_context.zone_relation = Context::RELATION_PASSING_THROUGH;

  if (!_announced_request)
  {
    _announced_request = true;

    const char* task = entry_context.task_type == Context::TASK_ZONE
      ? "on a zone task" : "on another task";

    const char* relation = "destination unknown";
    if (entry_context.zone_relation == Context::RELATION_DESTINATION)
      relation = "stopping here";
    else if (entry_context.zone_relation == Context::RELATION_PASSING_THROUGH)
      relation = "passing through";

    RCLCPP_INFO(
      _context->node()->get_logger(),
      "ZonePreEntry: [%s/%s] is at the boundary of zone [%s] and going to "
      "request a booking, %s, %s",
      _context->group().c_str(), _context->name().c_str(),
      _data.zone_name.c_str(), task, relation);
  }

  _context->node()->zone_request()->publish(
    phases::make_zone_entry_request(
      _context->group(),
      _context->name(),
      _data.zone_name,
      _current_request_id,
      std::move(entry_context),
      std::move(modifiers)));

  _has_pending_request = true;
}

//==============================================================================
void ZonePreEntry::Active::cancel()
{
  _state_sub.reset();
  _delay_timer.reset();
  _request_timer.reset();
  if (_move_sub.get().is_subscribed())
    _move_sub.get().unsubscribe();

  if (_move)
  {
    _move->cancel();
    _move = nullptr;
  }

  _booking = nullptr;

  // Only for a request still in flight. The manager may answer it after we
  // have stopped listening, and nothing else would ever release that.
  // Skipped when we hold a booking, which is a vertex the robot may be on.
  if (_has_pending_request
    && !_context->zone_booking(_data.zone_name))
  {
    _context->node()->zone_request()->publish(
      phases::make_zone_exit_request(
        _context->group(), _context->name(), _data.zone_name));
  }

  _complete(Status::Canceled);
}

//==============================================================================
void ZonePreEntry::Active::kill()
{
  cancel();
}

//==============================================================================
void ZonePreEntry::Active::_complete(Status status)
{
  if (!_finished)
    return;

  _state->update_status(status);

  const auto finished = _finished;
  _finished = nullptr;

  _context->worker().schedule([finished](const auto&)
    {
      finished();
    });
}

} // namespace events
} // namespace rmf_fleet_adapter
