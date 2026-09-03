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

#include "ZonePostExit.hpp"

#include "../phases/Utils.hpp"

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
auto ZonePostExit::Standby::make(
  agv::RobotContextPtr context,
  const AssignIDPtr& id,
  Data data) -> std::shared_ptr<Standby>
{
  auto standby = std::shared_ptr<Standby>(new Standby(std::move(data)));
  standby->_context = std::move(context);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    "Leave zone [" + standby->_data.zone_name + "]",
    "Releasing the booking held in this zone",
    rmf_task::Event::Status::Standby, {}, standby->_context->clock());
  return standby;
}

//==============================================================================
auto ZonePostExit::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration ZonePostExit::Standby::duration_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto ZonePostExit::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  return Active::make(_context, _state, std::move(finished), _data);
}

//==============================================================================
ZonePostExit::Standby::Standby(Data data)
: _data(std::move(data))
{
  // Do nothing
}

//==============================================================================
auto ZonePostExit::Active::make(
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
auto ZonePostExit::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration ZonePostExit::Active::remaining_time_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto ZonePostExit::Active::backup() const -> Backup
{
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto ZonePostExit::Active::interrupt(
  std::function<void()> task_is_interrupted) -> Resume
{
  _context->worker().schedule([task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });
  return Resume::make([]() { /* do nothing */ });
}

//==============================================================================
void ZonePostExit::Active::cancel()
{
  _complete(Status::Canceled);
}

//==============================================================================
void ZonePostExit::Active::kill()
{
  cancel();
}

//==============================================================================
ZonePostExit::Active::Active(Data data)
: _data(std::move(data))
{
  // Do nothing
}

//==============================================================================
void ZonePostExit::Active::_initialize()
{
  _state->update_status(Status::Underway);

  const auto node = _context->node();
  const auto booking = _context->zone_booking(_data.zone_name);
  const std::string released =
    booking ? booking->waypoint_name : std::string();

  // The local clear is authoritative and happens first.
  _context->clear_zone_booking(
    _data.zone_name, agv::RobotContext::ZoneTicketDisposal::HandBack);

  if (released.empty())
  {
    // Nothing was held for this zone.
    RCLCPP_DEBUG(
      node->get_logger(),
      "ZonePostExit for zone [%s]: robot [%s] held no booking",
      _data.zone_name.c_str(), _context->requester_id().c_str());

    _state->update_log().info(
      "no booking was held in zone [" + _data.zone_name + "]");
  }
  else
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Released zone booking [%s] in zone [%s] for robot [%s]",
      released.c_str(), _data.zone_name.c_str(),
      _context->requester_id().c_str());

    _state->update_log().info("released waypoint [" + released + "]");
  }

  node->zone_request()->publish(
    phases::make_zone_exit_request(
      _context->group(), _context->name(), _data.zone_name));

  _complete(Status::Completed);
}

//==============================================================================
void ZonePostExit::Active::_complete(Status status)
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
