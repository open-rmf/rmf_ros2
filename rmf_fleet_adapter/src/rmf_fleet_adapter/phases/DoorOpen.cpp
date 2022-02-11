/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "DoorOpen.hpp"
#include "RxOperators.hpp"
#include "SupervisorHasSession.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

#include <utility>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
std::shared_ptr<DoorOpen::ActivePhase> DoorOpen::ActivePhase::make(
  agv::RobotContextPtr context,
  std::string door_name,
  std::string request_id,
  rmf_traffic::Time expected_finish)
{
  auto inst = std::shared_ptr<ActivePhase>(
    new ActivePhase(
      std::move(context),
      std::move(door_name),
      std::move(request_id),
      std::move(expected_finish)
  ));
  inst->_init_obs();
  return inst;
}

//==============================================================================
DoorOpen::ActivePhase::ActivePhase(
  agv::RobotContextPtr context,
  std::string door_name,
  std::string request_id,
  rmf_traffic::Time expected_finish)
:  _context(std::move(context)),
  _door_name(std::move(door_name)),
  _request_id(std::move(request_id)),
  _expected_finish(std::move(expected_finish))
{
  _description = "Opening [door:" + _door_name + "]";
}

//==============================================================================
const rxcpp::observable<LegacyTask::StatusMsg>&
DoorOpen::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration DoorOpen::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DoorOpen::ActivePhase::emergency_alarm(bool /*on*/)
{
  // TODO: implement
}

//==============================================================================
void DoorOpen::ActivePhase::cancel()
{
  _cancelled.get_subscriber().on_next(true);
}

//==============================================================================
const std::string& DoorOpen::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
void DoorOpen::ActivePhase::_init_obs()
{
  auto transport = _context->node();

  using rmf_door_msgs::msg::DoorRequest;
  using rmf_door_msgs::msg::DoorState;
  using rmf_door_msgs::msg::SupervisorHeartbeat;
  using CombinedType = std::tuple<DoorState::SharedPtr,
      SupervisorHeartbeat::SharedPtr>;
  _obs = transport->door_state().combine_latest(
    rxcpp::observe_on_event_loop(),
    transport->door_supervisor())
    .lift<CombinedType>(on_subscribe([weak = weak_from_this(), transport]()
      {
        auto me = weak.lock();
        if (!me)
          return;

        me->_status.state = LegacyTask::StatusMsg::STATE_ACTIVE;
        me->_publish_open_door();
        me->_timer =
        transport->try_create_wall_timer(std::chrono::milliseconds(1000),
        [weak, transport]()
        {
          auto me = weak.lock();
          if (!me)
            return;

          // TODO(MXG): We can stop publishing the door request once the
          // supervisor sees our request.
          me->_publish_open_door();

          const auto current_expected_finish =
          me->_expected_finish + me->_context->itinerary().delay();

          const auto delay = me->_context->now() - current_expected_finish;
          if (delay > std::chrono::seconds(0))
          {
            me->_context->worker().schedule(
              [context = me->_context, delay](const auto&)
              {
                context->itinerary().delay(delay);
              });
          }
        });
      }))
    .map([weak = weak_from_this()](const auto& v)
      {
        auto me = weak.lock();
        if (!me)
          return LegacyTask::StatusMsg();

        me->_update_status(std::get<0>(v), std::get<1>(v));
        return me->_status;
      })
    .lift<LegacyTask::StatusMsg>(grab_while_active())
    .finally([weak = weak_from_this()]()
      {
        auto me = weak.lock();
        if (!me)
          return;

        me->_timer.reset();
      })
    // When the phase is cancelled, queue a door close phase to make sure that there is no hanging
    // open doors
    .take_until(_cancelled.get_observable().filter([](auto b) { return b; }))
    .concat(rxcpp::observable<>::create<LegacyTask::StatusMsg>(
        [weak = weak_from_this()](const auto& s)
        {
          auto me = weak.lock();
          if (!me)
            return;

          // FIXME: is this thread-safe?
          if (!me->_cancelled.get_value())
            s.on_completed();
          else
          {
            auto transport = me->_context->node();
            me->_door_close_phase = DoorClose::ActivePhase::make(
              me->_context,
              me->_door_name,
              me->_request_id
            );
            me->_door_close_phase->observe().subscribe(s);
          }
        }));
}

//==============================================================================
void DoorOpen::ActivePhase::_publish_open_door()
{
  rmf_door_msgs::msg::DoorRequest msg;
  msg.door_name = _door_name;
  msg.request_time = _context->node()->now();
  msg.requested_mode.value = rmf_door_msgs::msg::DoorMode::MODE_OPEN;
  msg.requester_id = _request_id;
  _context->node()->door_request()->publish(msg);
}

//==============================================================================
void DoorOpen::ActivePhase::_update_status(
  const rmf_door_msgs::msg::DoorState::SharedPtr& door_state,
  const rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr& heartbeat)
{
  using rmf_door_msgs::msg::DoorMode;
  if (door_state->door_name == _door_name &&
    door_state->current_mode.value == DoorMode::MODE_OPEN
    && supervisor_has_session(*heartbeat, _request_id, _door_name))
  {
    _status.status = "success";
    _status.state = LegacyTask::StatusMsg::STATE_COMPLETED;
  }
  else
  {
    _status.status = "Waiting for [door:" + _door_name + "] to open";
  }
}

//==============================================================================
DoorOpen::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string  door_name,
  std::string request_id,
  rmf_traffic::Time expected_finish)
:  _context(std::move(context)),
  _door_name(std::move(door_name)),
  _request_id(std::move(request_id)),
  _expected_finish(std::move(expected_finish))
{
  _description = "Open [door:" + _door_name + "]";
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> DoorOpen::PendingPhase::begin()
{
  return ActivePhase::make(_context, _door_name, _request_id, _expected_finish);
}

//==============================================================================
rmf_traffic::Duration DoorOpen::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& DoorOpen::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
