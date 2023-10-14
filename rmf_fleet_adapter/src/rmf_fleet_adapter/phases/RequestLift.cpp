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
#include "EndLiftSession.hpp"
#include "RxOperators.hpp"
#include "../agv/internal_RobotUpdateHandle.hpp"
#include "../agv/internal_EasyFullControl.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
std::shared_ptr<RequestLift::ActivePhase> RequestLift::ActivePhase::make(
  agv::RobotContextPtr context,
  std::string lift_name,
  std::string destination,
  rmf_traffic::Time expected_finish,
  const Located located,
  rmf_traffic::PlanId plan_id,
  std::optional<agv::Destination> localize)
{
  auto inst = std::shared_ptr<ActivePhase>(
    new ActivePhase(
      std::move(context),
      std::move(lift_name),
      std::move(destination),
      std::move(expected_finish),
      located,
      plan_id,
      std::move(localize)
  ));
  inst->_init_obs();
  return inst;
}

//==============================================================================
const rxcpp::observable<LegacyTask::StatusMsg>&
RequestLift::ActivePhase::observe() const
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
void RequestLift::ActivePhase::emergency_alarm(bool /*on*/)
{
  // TODO: implement
}

//==============================================================================
void RequestLift::ActivePhase::cancel()
{
  // GoToPlace / ExecutePlan don't call the cancel function anyway
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
  rmf_traffic::Time expected_finish,
  Located located,
  rmf_traffic::PlanId plan_id,
  std::optional<agv::Destination> localize)
: _context(std::move(context)),
  _lift_name(std::move(lift_name)),
  _destination(std::move(destination)),
  _expected_finish(std::move(expected_finish)),
  _located(located),
  _plan_id(plan_id),
  _localize_after(std::move(localize))
{
  std::ostringstream oss;
  oss << "Requesting lift [" << lift_name << "] to [" << destination << "]";

  _description = oss.str();
}

//==============================================================================
void RequestLift::ActivePhase::_init_obs()
{
  using rmf_lift_msgs::msg::LiftState;

  _obs = _context->node()->lift_state()
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .lift<LiftState::SharedPtr>(
    on_subscribe(
      [weak = weak_from_this()]()
      {
        auto me = weak.lock();
        if (!me)
          return;

        me->_do_publish();
        me->_timer = me->_context->node()->try_create_wall_timer(
          std::chrono::milliseconds(1000),
          [weak]()
          {
            auto me = weak.lock();
            if (!me)
              return;

            // TODO(MXG): We can stop publishing the door request once the
            // supervisor sees our request.
            me->_do_publish();
            const auto delay = me->_context->now() - me->_expected_finish;
            if (delay > std::chrono::seconds(0))
            {
              me->_context->worker().schedule(
                [
                  context = me->_context,
                  plan_id = me->_plan_id,
                  delay
                ](const auto&)
                {
                  context->itinerary().cumulative_delay(plan_id, delay);
                });
            }
          });
      }))
    .map([weak = weak_from_this()](const auto& v)
      {
        auto me = weak.lock();
        if (!me)
          return LegacyTask::StatusMsg();

        return me->_get_status(v);
      })
    .lift<LegacyTask::StatusMsg>(grab_while([weak =
      weak_from_this()](const LegacyTask::StatusMsg& status)
      {
        auto me = weak.lock();
        if (!me)
          return false;

        if (
          status.state == LegacyTask::StatusMsg::STATE_COMPLETED ||
          status.state == LegacyTask::StatusMsg::STATE_FAILED)
        {
          me->_timer.reset();
          return false;
        }
        return true;
      }))
    .take_until(_cancelled.get_observable().filter([](auto b) { return b; }))
    .concat(rxcpp::observable<>::create<LegacyTask::StatusMsg>(
        [weak = weak_from_this()](const auto& s)
        {
          auto me = weak.lock();
          if (!me)
            return;

          if (me->_localize_after.has_value())
          {
            auto finish = [s, worker = me->_context->worker(), weak]()
              {
                worker.schedule([s, weak](const auto&)
                  {
                    if (const auto me = weak.lock())
                      me->_finish();

                    s.on_completed();
                  });
              };
            auto cmd = agv::EasyFullControl
              ::CommandExecution::Implementation::make_hold(
                me->_context,
                me->_expected_finish,
                me->_plan_id,
                std::move(finish));

            agv::Destination::Implementation::get(*me->_localize_after)
              .position = me->_context->position();

            if (me->_context->localize(*me->_localize_after, std::move(cmd)))
            {
              me->_rewait_timer = me->_context->node()->try_create_wall_timer(
                std::chrono::seconds(30),
                [weak, s]
                {
                  const auto me = weak.lock();
                  if (!me)
                    return;

                  RCLCPP_ERROR(
                    me->_context->node()->get_logger(),
                    "Waiting for robot [%s] to localize timed out. Please "
                    "ensure that your localization function triggers "
                    "execution.finished() when the robot's localization "
                    "process is finished.",
                    me->_context->requester_id().c_str());

                  me->_finish();
                  s.on_completed();
                });
              return;
            }
          }

          me->_finish();
          s.on_completed();
        }));
}

//==============================================================================
LegacyTask::StatusMsg RequestLift::ActivePhase::_get_status(
  const rmf_lift_msgs::msg::LiftState::SharedPtr& lift_state)
{
  using rmf_lift_msgs::msg::LiftState;
  using rmf_lift_msgs::msg::LiftRequest;
  LegacyTask::StatusMsg status{};
  status.state = LegacyTask::StatusMsg::STATE_ACTIVE;
  if (!_rewaiting &&
    lift_state->lift_name == _lift_name &&
    lift_state->current_floor == _destination &&
    lift_state->door_state == LiftState::DOOR_OPEN &&
    lift_state->session_id == _context->requester_id())
  {
    bool completed = false;
    const auto& watchdog = _context->get_lift_watchdog();

    if (_watchdog_info)
    {
      std::lock_guard<std::mutex> lock(_watchdog_info->mutex);
      if (_watchdog_info->decision.has_value())
      {
        switch (*_watchdog_info->decision)
        {
          case agv::RobotUpdateHandle::Unstable::Decision::Clear:
          {
            completed = true;
            break;
          }
          case agv::RobotUpdateHandle::Unstable::Decision::Undefined:
          {
            RCLCPP_ERROR(
              _context->node()->get_logger(),
              "Received undefined decision for lift watchdog of [%s]. "
              "Defaulting to a Crowded decision.",
              _context->name().c_str());
            // Intentionally drop to the next case
            [[fallthrough]];
          }
          case agv::RobotUpdateHandle::Unstable::Decision::Crowded:
          {
            _rewaiting = true;

            _lift_end_phase = EndLiftSession::Active::make(
              _context, _lift_name, _destination);
            _reset_session_subscription = _lift_end_phase->observe()
              .subscribe([](const auto&)
                {
                  // Do nothing
                });

            _rewait_timer = _context->node()->try_create_wall_timer(
              _context->get_lift_rewait_duration(),
              [w = weak_from_this()]()
              {
                if (const auto& me = w.lock())
                {
                  me->_rewaiting = false;
                  me->_rewait_timer.reset();
                  me->_reset_session_subscription =
                  rmf_rxcpp::subscription_guard();
                }
              });
            break;
          }
        }

        _watchdog_info.reset();
      }
    }
    else if (_located == Located::Outside && watchdog)
    {
      _watchdog_info = std::make_shared<WatchdogInfo>();
      watchdog(
        _lift_name,
        [info = _watchdog_info](
          agv::RobotUpdateHandle::Unstable::Decision decision)
        {
          std::lock_guard<std::mutex> lock(info->mutex);
          info->decision = decision;
        });
    }
    else
    {
      completed = true;
    }

    if (completed)
    {
      status.state = LegacyTask::StatusMsg::STATE_COMPLETED;
      status.status = "success";
      _timer.reset();
    }
  }
  else if (_rewaiting)
  {
    status.status = "[" + _context->name() + "] is waiting for lift ["
      + _lift_name + "] to clear out";
  }
  else if (lift_state->lift_name == _lift_name)
  {
    // TODO(MXG): Make this a more human-friendly message
    status.status = "[" + _context->name() + "] still waiting for lift ["
      + _lift_name + "]  current state: "
      + lift_state->current_floor + " vs " + _destination + " | "
      + std::to_string(static_cast<int>(lift_state->door_state))
      + " vs " + std::to_string(static_cast<int>(LiftState::DOOR_OPEN))
      + " | " + lift_state->session_id + " vs " + _context->requester_id();
  }

  return status;
}

//==============================================================================
void RequestLift::ActivePhase::_do_publish()
{
  if (_rewaiting)
    return;

  if (!_destination_handle)
  {
    _destination_handle = _context->set_lift_destination(
      _lift_name, _destination, _located == Located::Inside);
  }
}

//==============================================================================
void RequestLift::ActivePhase::_finish()
{
  if (_located == Located::Outside)
  {
    // The robot is going to start moving into the lift now, so we should lock
    // the destination in.
    _context->set_lift_destination(_lift_name, _destination, true);
  }
}

//==============================================================================
RequestLift::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string lift_name,
  std::string destination,
  rmf_traffic::Time expected_finish,
  Located located,
  rmf_traffic::PlanId plan_id,
  std::optional<agv::Destination> localize)
: _context(std::move(context)),
  _lift_name(std::move(lift_name)),
  _destination(std::move(destination)),
  _expected_finish(std::move(expected_finish)),
  _located(located),
  _plan_id(plan_id),
  _localize_after(std::move(localize))
{
  std::ostringstream oss;
  oss << "Requesting lift \"" << lift_name << "\" to \"" << destination << "\"";

  _description = oss.str();
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> RequestLift::PendingPhase::begin()
{
  return ActivePhase::make(
    _context,
    _lift_name,
    _destination,
    _expected_finish,
    _located,
    _plan_id,
    _localize_after);
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
