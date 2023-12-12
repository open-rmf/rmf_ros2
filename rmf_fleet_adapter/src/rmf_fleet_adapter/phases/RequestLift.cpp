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
  Data data)
{
  auto inst = std::shared_ptr<ActivePhase>(
    new ActivePhase(
      std::move(context),
      std::move(lift_name),
      std::move(destination),
      std::move(data)
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
  // GoToPlace and ExecutePlan don't call the cancel function anyway
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
  Data data)
: _context(std::move(context)),
  _lift_name(std::move(lift_name)),
  _destination(std::move(destination)),
  _data(std::move(data))
{
  std::ostringstream oss;
  oss << "Requesting lift [" << lift_name << "] to [" << destination << "]";

  _description = oss.str();
}

//==============================================================================
void RequestLift::ActivePhase::_init_obs()
{
  using rmf_lift_msgs::msg::LiftState;

  if (_data.located == Located::Outside && _context->current_lift_destination())
  {
    // Check if the current destination is the one we want and also has arrived.
    // If so, we can skip the rest of this process and just make an observable
    // that says it's completed right away.
    if (_context->current_lift_destination()->matches(_lift_name, _destination))
    {
      _obs = rxcpp::observable<>::create<LegacyTask::StatusMsg>(
        [w = weak_from_this()](rxcpp::subscriber<LegacyTask::StatusMsg> s)
        {
          const auto self = w.lock();
          if (!self)
            return;

          if (self->_data.resume_itinerary)
          {
            self->_context->schedule_itinerary(
              self->_data.plan_id, *self->_data.resume_itinerary);
            const auto delay =
            self->_context->now() - self->_data.expected_finish;
            self->_context->itinerary().cumulative_delay(
              *self->_data.plan_id, delay);
          }

          s.on_completed();
        });
      return;
    }
  }

  if (_data.hold_point.has_value())
  {
    *_data.plan_id = _context->itinerary().assign_plan_id();
    _context->schedule_hold(
      _data.plan_id,
      _data.hold_point->time(),
      std::chrono::seconds(10),
      _data.hold_point->position(),
      _destination);
  }

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
            const auto delay = me->_context->now() - me->_data.expected_finish;
            if (delay > std::chrono::seconds(0))
            {
              me->_context->worker().schedule(
                [
                  context = me->_context,
                  plan_id = *me->_data.plan_id,
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

          if (me->_data.localize_after.has_value())
          {
            auto finish = [s, worker = me->_context->worker(), weak]()
            {
              worker.schedule([s, weak](const auto&)
              {
                if (const auto me = weak.lock())
                {
                  if (!me->_finish())
                  {
                    return;
                  }
                }

                s.on_completed();
              });
            };

            auto cmd = agv::EasyFullControl
            ::CommandExecution::Implementation::make_hold(
              me->_context,
              me->_data.expected_finish,
              *me->_data.plan_id,
              std::move(finish));

            agv::Destination::Implementation::get(*me->_data.localize_after)
            .position = me->_context->position();

            if (me->_context->localize(*me->_data.localize_after,
            std::move(cmd)))
            {
              me->_rewait_timer = me->_context->node()->try_create_wall_timer(
                std::chrono::seconds(300),
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

                  if (me->_finish())
                    s.on_completed();
                });
              return;
            }
          }

          if (me->_finish())
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
    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Lift has arrived on floor [%s] and opened its doors for robot [%s]",
      lift_state->current_floor.c_str(),
      lift_state->session_id.c_str());
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
    else if (_data.located == Located::Outside && watchdog)
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
      _lift_name, _destination, _data.located == Located::Inside);
  }
}

//==============================================================================
bool RequestLift::ActivePhase::_finish()
{
  // The return value of _finish tells us whether we should have the observable
  // proceed to trigger on_completed(). If we have already finished before then
  // _finished will be true, so we should return false to indicate that the
  // observable should not proceed to trigger on_completed().
  if (_finished)
    return false;

  _finished = true;

  if (_data.located == Located::Outside)
  {
    // The robot is going to start moving into the lift now, so we should lock
    // the destination in.
    _context->set_lift_destination(_lift_name, _destination, true);

    // We should replan to make sure there are no traffic issues that came up
    // in the time that we were waiting for the lift.
    if (_data.hold_point.has_value())
    {
      if (_data.hold_point->graph_index().has_value())
      {
        auto start = rmf_traffic::agv::Plan::Start(
          _context->now(),
          _data.hold_point->graph_index().value(),
          _data.hold_point->position()[2]);
        _context->set_location({std::move(start)});
      }
    }

    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Requesting replan for [%s] because it has finished waiting lift [%s] "
      "to arrive at [%s]",
      _context->requester_id().c_str(),
      _lift_name.c_str(),
      _destination.c_str());
    _context->request_replan();
    return false;
  }

  return true;
}

//==============================================================================
RequestLift::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string lift_name,
  std::string destination,
  Data data)
: _context(std::move(context)),
  _lift_name(std::move(lift_name)),
  _destination(std::move(destination)),
  _data(std::move(data))
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
    _data);
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
