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

#include "ZonePostEntry.hpp"

#include "../phases/Utils.hpp"

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
auto ZonePostEntry::Standby::make(
  agv::RobotContextPtr context,
  const AssignIDPtr& id,
  Data data) -> std::shared_ptr<Standby>
{
  auto standby = std::shared_ptr<Standby>(new Standby(std::move(data)));
  standby->_context = std::move(context);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    "Reach zone [" + standby->_data.zone_name + "]",
    "Telling the zone manager we have arrived",
    rmf_task::Event::Status::Standby, {}, standby->_context->clock());
  return standby;
}

//==============================================================================
auto ZonePostEntry::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration ZonePostEntry::Standby::duration_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto ZonePostEntry::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  return Active::make(_context, _state, std::move(finished), _data);
}

//==============================================================================
ZonePostEntry::Standby::Standby(Data data)
: _data(std::move(data))
{
  // Do nothing
}

//==============================================================================
auto ZonePostEntry::Active::make(
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
auto ZonePostEntry::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration ZonePostEntry::Active::remaining_time_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto ZonePostEntry::Active::backup() const -> Backup
{
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto ZonePostEntry::Active::interrupt(
  std::function<void()> task_is_interrupted) -> Resume
{
  _context->worker().schedule([task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });
  return Resume::make([]() { /* do nothing */ });
}

//==============================================================================
void ZonePostEntry::Active::cancel()
{
  _complete(Status::Canceled);
}

//==============================================================================
void ZonePostEntry::Active::kill()
{
  cancel();
}

//==============================================================================
ZonePostEntry::Active::Active(Data data)
: _data(std::move(data))
{
  // Do nothing
}

//==============================================================================
void ZonePostEntry::Active::_initialize()
{
  _state->update_status(Status::Underway);

  const auto node = _context->node();
  const auto booking = _context->zone_booking(_data.zone_name);

  // A grant precedes every path into this event, so a booking is expected.
  // A revocation or the release sweep could still have cleared it, and
  // reporting arrival would name a booking the manager no longer holds.
  if (booking)
  {
    node->zone_request()->publish(
      phases::make_zone_arrived_request(
        _context->group(), _context->name(), _data.zone_name));

    _state->update_log().info(
      "arrived at waypoint [" + booking->waypoint_name + "] in zone ["
      + _data.zone_name + "]");
  }
  else
  {
    RCLCPP_INFO(
      node->get_logger(),
      "ZonePostEntry for zone [%s]: robot [%s] holds no booking, so the zone "
      "manager is not told it arrived",
      _data.zone_name.c_str(), _context->requester_id().c_str());
  }

  _complete(Status::Completed);
}

//==============================================================================
void ZonePostEntry::Active::_complete(Status status)
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
