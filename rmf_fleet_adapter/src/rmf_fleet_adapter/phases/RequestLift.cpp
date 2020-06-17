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

#include "RequestLift.hpp"
#include "RxOperators.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
std::shared_ptr<RequestLift::ActivePhase> RequestLift::ActivePhase::make(
  agv::RobotContextPtr context,
  std::string lift_name,
  std::string destination,
  rmf_traffic::Time expected_finish)
{
  auto inst = std::shared_ptr<ActivePhase>(
    new ActivePhase(
      std::move(context),
      std::move(lift_name),
      std::move(destination),
      std::move(expected_finish)
  ));
  inst->_init_obs();
  return inst;
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& RequestLift::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration RequestLift::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void RequestLift::ActivePhase::emergency_alarm(bool on)
{
  // TODO: implement
}

//==============================================================================
void RequestLift::ActivePhase::cancel()
{
  // TODO: implement
}

//==============================================================================
const std::string& RequestLift::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
RequestLift::ActivePhase::ActivePhase(
  agv::RobotContextPtr context,
  std::string lift_name,
  std::string destination,
  rmf_traffic::Time expected_finish)
  : _context(std::move(context)),
    _lift_name(std::move(lift_name)),
    _destination(std::move(destination)),
    _expected_finish(std::move(expected_finish))
{
  std::ostringstream oss;
  oss << "Requesting lift \"" << lift_name << "\" to \"" << destination << "\"";

  _description = oss.str();
}

//==============================================================================
void RequestLift::ActivePhase::_init_obs()
{
  using rmf_lift_msgs::msg::LiftState;

  _obs = _context->node()->lift_state()
    .lift<LiftState::SharedPtr>(on_subscribe([weak = weak_from_this()]()
    {
      auto me = weak.lock();
      if (!me)
        return;

      me->_do_publish();
      me->_timer = me->_context->node()->create_wall_timer(
        std::chrono::milliseconds(1000),
        [weak]()
        {
          auto me = weak.lock();
          if (!me)
            return;

          // TODO(MXG): We can stop publishing the door request once the
          // supervisor sees our request.
          me->_do_publish();

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
        return Task::StatusMsg();

      return me->_get_status(v);
    })
    .lift<Task::StatusMsg>(grab_while([weak = weak_from_this()](const Task::StatusMsg& status)
    {
      auto me = weak.lock();
      if (!me)
        return false;

      if (
        status.state == Task::StatusMsg::STATE_COMPLETED ||
        status.state == Task::StatusMsg::STATE_FAILED)
      {
        me->_timer.reset();
        return false;
      }
      return true;
    }));
}

//==============================================================================
Task::StatusMsg RequestLift::ActivePhase::_get_status(
  const rmf_lift_msgs::msg::LiftState::SharedPtr& lift_state)
{
  using rmf_lift_msgs::msg::LiftState;
  Task::StatusMsg status{};
  status.state = Task::StatusMsg::STATE_ACTIVE;
  if (lift_state->current_floor == _destination && lift_state->door_state == LiftState::DOOR_OPEN)
  {
    status.state = Task::StatusMsg::STATE_COMPLETED;
    status.status = "success";
    _timer.reset();
  }
  return status;
}

//==============================================================================
void RequestLift::ActivePhase::_do_publish()
{
  rmf_lift_msgs::msg::LiftRequest msg{};
  msg.lift_name = _lift_name;
  msg.destination_floor = _destination;
  msg.session_id = _context->requester_id();
  msg.request_time = _context->node()->now();
  msg.request_type = rmf_lift_msgs::msg::LiftRequest::REQUEST_AGV_MODE;
  msg.door_state = rmf_lift_msgs::msg::LiftRequest::DOOR_OPEN;
  _context->node()->lift_request()->publish(msg);
}

//==============================================================================
RequestLift::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string lift_name,
  std::string destination,
  rmf_traffic::Time expected_finish)
  : _context(std::move(context)),
    _lift_name(std::move(lift_name)),
    _destination(std::move(destination)),
    _expected_finish(std::move(expected_finish))
{
  std::ostringstream oss;
  oss << "Requesting lift \"" << lift_name << "\" to \"" << destination << "\"";

  _description = oss.str();
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> RequestLift::PendingPhase::begin()
{
  return ActivePhase::make(
    _context,
    _lift_name,
    _destination,
    _expected_finish);
}

//==============================================================================
rmf_traffic::Duration RequestLift::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& RequestLift::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
