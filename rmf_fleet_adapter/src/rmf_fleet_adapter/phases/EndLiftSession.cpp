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

#include "EndLiftSession.hpp"
#include "RxOperators.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
std::shared_ptr<EndLiftSession::Active> EndLiftSession::Active::make(
  agv::RobotContextPtr context,
  std::string lift_name,
  std::string destination)
{
  auto inst = std::shared_ptr<Active>(
    new Active(
      std::move(context),
      std::move(lift_name),
      std::move(destination)));
  inst->_init_obs();
  return inst;
}

//==============================================================================
EndLiftSession::Active::Active(
  agv::RobotContextPtr context,
  std::string lift_name,
  std::string destination)
: _context(std::move(context)),
  _lift_name(std::move(lift_name)),
  _destination(std::move(destination))
{
  _description = "Ending session with lift [" + lift_name + "]";
}

//==============================================================================
const rxcpp::observable<LegacyTask::StatusMsg>&
EndLiftSession::Active::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration EndLiftSession::Active::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void EndLiftSession::Active::emergency_alarm(bool)
{
  // Do nothing.
}

//==============================================================================
void EndLiftSession::Active::cancel()
{
  // Do nothing. This is not interruptable.
}

//==============================================================================
const std::string& EndLiftSession::Active::description() const
{
  return _description;
}

//==============================================================================
void EndLiftSession::Active::_init_obs()
{
  using rmf_lift_msgs::msg::LiftRequest;
  using rmf_lift_msgs::msg::LiftState;
  _obs = _context->node()->lift_state()
    .lift<LiftState::SharedPtr>(on_subscribe([weak = weak_from_this()]()
      {
        const auto me = weak.lock();
        if (!me)
          return;

        me->_publish_session_end();
        me->_timer = me->_context->node()->try_create_wall_timer(
          std::chrono::milliseconds(1000),
          [weak]()
          {
            const auto me = weak.lock();
            if (!me)
              return;

            me->_publish_session_end();
          });
      }))
    .map([weak = weak_from_this()](const LiftState::SharedPtr& state)
      {
        const auto me = weak.lock();
        if (!me)
          return LegacyTask::StatusMsg();

        LegacyTask::StatusMsg msg;
        msg.state = LegacyTask::StatusMsg::STATE_ACTIVE;

        if (state->lift_name != me->_lift_name)
          return msg;

        if (state->session_id != me->_context->requester_id())
        {
          msg.status = "success";
          msg.state = LegacyTask::StatusMsg::STATE_COMPLETED;
        }

        return msg;
      })
    .lift<LegacyTask::StatusMsg>(grab_while_active())
    .finally([weak = weak_from_this()]()
      {
        const auto me = weak.lock();
        if (!me)
          return;

        me->_timer.reset();
      });
}

//==============================================================================
void EndLiftSession::Active::_publish_session_end()
{
  rmf_lift_msgs::msg::LiftRequest msg;
  msg.lift_name = _lift_name;
  msg.destination_floor = _destination;
  msg.request_type = rmf_lift_msgs::msg::LiftRequest::REQUEST_END_SESSION;
  msg.session_id = _context->requester_id();

  _context->node()->lift_request()->publish(msg);
}

//==============================================================================
EndLiftSession::Pending::Pending(
  agv::RobotContextPtr context,
  std::string lift_name,
  std::string destination)
: _context(std::move(context)),
  _lift_name(std::move(lift_name)),
  _destination(std::move(destination))
{
  _description = "End session with lift [" + lift_name + "]";
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> EndLiftSession::Pending::begin()
{
  return Active::make(
    std::move(_context),
    std::move(_lift_name),
    std::move(_destination));
}

//==============================================================================
rmf_traffic::Duration EndLiftSession::Pending::estimate_phase_duration() const
{
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& EndLiftSession::Pending::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
