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

      switch (result.status)
      {
        case phases::ZoneStateResult::Status::Granted:
        {
          self->_state->update_log().info(
            "Assigned waypoint [" + result.waypoint_name + "]");

          // Re-seat in case this is the first booking we have been given.
          self->_booking = self->_context->zone_booking(zone_name);

          self->_state_sub.reset();
          self->_delay_timer.reset();

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
          self->_complete(Status::Failed);
          return;
        }

        case phases::ZoneStateResult::Status::UnknownZone:
        {
          self->_state->update_log().error(
            "zone [" + zone_name + "] is unknown to the zone manager");
          self->_state_sub.reset();
          self->_delay_timer.reset();
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

  _publish_finalize_request();

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

  _context->node()->zone_request()->publish(
    phases::make_zone_entry_request(
      _context->group(),
      _context->name(),
      _data.zone_name,
      _current_request_id,
      std::move(modifiers)));

  _has_pending_request = true;
}

//==============================================================================
void ZonePreEntry::Active::cancel()
{
  _state_sub.reset();
  _delay_timer.reset();
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
