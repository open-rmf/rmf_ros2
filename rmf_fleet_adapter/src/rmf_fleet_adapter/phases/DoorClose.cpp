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

#include "DoorClose.hpp"
#include "RxOperators.hpp"
#include "SupervisorHasSession.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
std::shared_ptr<DoorClose::ActivePhase> DoorClose::ActivePhase::make(
  agv::RobotContextPtr context,
  std::string door_name,
  std::string request_id)
{
  auto inst = std::shared_ptr<ActivePhase>(new ActivePhase(
        std::move(context),
        std::move(door_name),
        std::move(request_id)
  ));
  inst->_init_obs();
  return inst;
}

//==============================================================================
const rxcpp::observable<LegacyTask::StatusMsg>&
DoorClose::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration DoorClose::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DoorClose::ActivePhase::emergency_alarm(bool /*on*/)
{
  // TODO: implement
}

//==============================================================================
void DoorClose::ActivePhase::cancel()
{
  // Don't actually cancel anything here, we don't want to leave hanging opened doors.
  // no op
}

//==============================================================================
const std::string& DoorClose::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
void DoorClose::ActivePhase::_init_obs()
{
  using rmf_door_msgs::msg::DoorRequest;
  using rmf_door_msgs::msg::SupervisorHeartbeat;
  _obs = _context->node()->door_supervisor()
    .lift<SupervisorHeartbeat::SharedPtr>(on_subscribe([weak =
      weak_from_this()]()
      {
        auto me = weak.lock();
        if (!me)
          return;

        me->_status.state = LegacyTask::StatusMsg::STATE_ACTIVE;
        me->_publish_close_door();
        me->_timer = me->_context->node()->try_create_wall_timer(
          std::chrono::milliseconds(1000),
          [weak]()
          {
            auto me = weak.lock();
            if (!me)
              return;

            me->_publish_close_door();
          });
      }))
    .map([weak = weak_from_this()](const auto& heartbeat)
      {
        auto me = weak.lock();
        if (!me)
          return LegacyTask::StatusMsg();

        me->_update_status(heartbeat);
        return me->_status;
      })
    .lift<LegacyTask::StatusMsg>(grab_while_active())
    .finally([weak = weak_from_this()]()
      {
        auto me = weak.lock();
        if (!me)
          return;

        me->_timer.reset();
      });
}

//==============================================================================
void DoorClose::ActivePhase::_publish_close_door()
{
  rmf_door_msgs::msg::DoorRequest msg{};
  msg.door_name = _door_name;
  msg.request_time = _context->node()->now();
  msg.requested_mode.value = rmf_door_msgs::msg::DoorMode::MODE_CLOSED;
  msg.requester_id = _request_id;
  _context->node()->door_request()->publish(msg);
}

//==============================================================================
void DoorClose::ActivePhase::_update_status(
  const rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr& heartbeat)
{
  if (!supervisor_has_session(*heartbeat, _request_id, _door_name))
  {
    _status.status = "success";
    _status.state = LegacyTask::StatusMsg::STATE_COMPLETED;
  }
  else
  {
    _status.status = "Waiting for [door:" + _door_name + "] to close";
  }
}

//==============================================================================
DoorClose::ActivePhase::ActivePhase(
  agv::RobotContextPtr context,
  std::string door_name,
  std::string request_id)
: _context(std::move(context)),
  _door_name(std::move(door_name)),
  _request_id(std::move(request_id))
{
  _description = "Closing [door:" + _door_name + "]";
}

//==============================================================================
DoorClose::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string door_name,
  std::string request_id)
: _context(std::move(context)),
  _door_name(std::move(door_name)),
  _request_id(std::move(request_id))
{
  _description = "Close [door:" + _door_name + "]";
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> DoorClose::PendingPhase::begin()
{
  return DoorClose::ActivePhase::make(
    _context,
    _door_name,
    _request_id);
}

//==============================================================================
rmf_traffic::Duration DoorClose::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& DoorClose::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
