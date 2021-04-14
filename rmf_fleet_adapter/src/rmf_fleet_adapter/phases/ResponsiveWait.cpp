/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "ResponsiveWait.hpp"

#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
auto ResponsiveWait::Active::observe() const
-> const rxcpp::observable<StatusMsg>&
{
  return _status_obs;
}

//==============================================================================
rmf_traffic::Duration ResponsiveWait::Active::estimate_remaining_time() const
{
  if (_info.finish_time.has_value())
  {
    const auto now = rmf_traffic_ros2::convert(_info.context->node()->now());
    const auto remaining = *_info.finish_time - now;
    if (remaining.count() > 0)
      return remaining;
  }

  return std::chrono::seconds(0);
}

//==============================================================================
void ResponsiveWait::Active::emergency_alarm(bool on)
{
  if (!_movement)
  {
    throw std::runtime_error(
      "[rmf_fleet_adapter::phases::ResponsiveWait::Active::emergency_alarm] "
      "_movement field is null. This is a critical bug in rmf_fleet_adapter. "
      "Please report this to the maintainers.");
  }

  _movement->emergency_alarm(on);
}

//==============================================================================
void ResponsiveWait::Active::cancel()
{
  // We reset the period to ensure that the phase ends the next time GoToPlace
  // issues that it is finished.
  _info.period.reset();

  // Now we cancel the movement and wait for the underlying GoToPlace phase to
  // report that it is finished.
  _movement->cancel();
}

//==============================================================================
const std::string& ResponsiveWait::Active::description() const
{
  return _info.description;
}

//==============================================================================
ResponsiveWait::Active::Active(PhaseInfo info)
: _info(std::move(info))
{
  _status_obs = _status_publisher.get_observable();
}

//==============================================================================
void ResponsiveWait::Active::_begin_movement()
{
  const rmf_traffic::Time now = _info.context->now();
  rmf_traffic::agv::Plan::Start start_estimate(now, _info.waiting_point, 0.0);
  rmf_traffic::agv::Plan::Goal goal(_info.waiting_point);
  if (_info.finish_time.has_value())
  {
    goal.minimum_time(*_info.finish_time);
  }
  else if (_info.period.has_value())
  {
    goal.minimum_time(now + *_info.period);
  }
  else
  {
    throw std::runtime_error(
      "[rmf_fleet_adapter::phases::ResponsiveWait::Active::constructor] "
      "PhaseInfo is missing both finish_time and period. This is a critical "
      "bug in rmf_fleet_adapter. Please report it to the maintainers.");
  }

  std::optional<rmf_traffic::Duration> tail;
  if (_info.period.has_value())
    tail = std::chrono::seconds(1);

  _movement = GoToPlace::make(
    _info.context, start_estimate, goal, tail)->begin();

  _movement_subscription =
    _movement->observe()
    .observe_on(rxcpp::identity_same_worker(_info.context->worker()))
    .subscribe(
      [w = weak_from_this()](const StatusMsg& msg)
      {
        const auto me = w.lock();
        if (!me)
          return;

        me->_status_publisher.get_subscriber().on_next(msg);
      },
      [w = weak_from_this()](std::exception_ptr e)
      {
        const auto me = w.lock();
        if (!me)
          return;

        me->_status_publisher.get_subscriber().on_error(std::move(e));
      },
      [w = weak_from_this()]()
      {
        const auto me = w.lock();
        if (!me)
          return;

        if (me->_info.period.has_value())
        {
          // If this is an uncanceled indefinite wait, then we will begin the
          // movement again.
          me->_begin_movement();
          return;
        }

        me->_status_publisher.get_subscriber().on_completed();
      });
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> ResponsiveWait::Pending::begin()
{
  auto active = std::shared_ptr<Active>(new Active(_info));
  active->_begin_movement();

  return active;
}

//==============================================================================
rmf_traffic::Duration ResponsiveWait::Pending::estimate_phase_duration() const
{
  return std::chrono::seconds(0);
}

//==============================================================================
const std::string& ResponsiveWait::Pending::description() const
{
  return _info.description;
}

//==============================================================================
ResponsiveWait::Pending::Pending(PhaseInfo info)
: _info(std::move(info))
{
  // Do nothing
}

//==============================================================================
auto ResponsiveWait::make_until(
  agv::RobotContextPtr context,
  std::size_t waiting_point,
  rmf_traffic::Time finish_time) -> std::unique_ptr<Pending>
{
  PhaseInfo info{
    std::move(context),
    waiting_point,
    finish_time,
    std::nullopt,
    ""
  };

  info.description = "[" + info.context->requester_id() + "] waiting at ["
      + std::to_string(info.waiting_point) + "]";

  return std::unique_ptr<Pending>(new Pending(std::move(info)));
}

//==============================================================================
auto ResponsiveWait::make_indefinite(
  agv::RobotContextPtr context,
  std::size_t waiting_point,
  rmf_traffic::Duration update_period) -> std::unique_ptr<Pending>
{
  PhaseInfo info{
    std::move(context),
    waiting_point,
    std::nullopt,
    update_period,
    ""
  };

  info.description = "[" + info.context->requester_id() + "] waiting at ["
      + std::to_string(info.waiting_point) + "]";

  return std::unique_ptr<Pending>(new Pending(std::move(info)));
}

} // namespace phases
} // namespace rmf_fleet_adapter
