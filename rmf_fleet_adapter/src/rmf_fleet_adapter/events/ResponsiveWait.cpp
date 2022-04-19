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
#include "GoToPlace.hpp"

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>

#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
void ResponsiveWait::add(rmf_task_sequence::Event::Initializer& initializer)
{
  initializer.add<Description>(
    [](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      std::function<void()> update) -> StandbyPtr
    {
      return Standby::make(
        id, get_state, parameters, description, std::move(update));
    },
    [](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      const nlohmann::json&,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished) -> ActivePtr
    {
      return Standby::make(
        id, get_state, parameters, description, std::move(update))
      ->begin(std::move(checkpoint), std::move(finished));
    });
}

//==============================================================================
rmf_task::Task::ActivePtr ResponsiveWait::start(
  const std::string& task_id,
  agv::RobotContextPtr& context,
  std::size_t waiting_point,
  std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
  std::function<void()> finished)
{
  rmf_task_sequence::Task::Builder builder;
  builder.add_phase(
    rmf_task_sequence::phases::SimplePhase::Description::make(
      ResponsiveWait::Description::make_indefinite(waiting_point)), {});

  const auto desc = builder.build("Responsive Wait", "");
  const rmf_task::Request request(task_id, context->now(), nullptr, desc, true);

  return context->task_activator()->activate(
    context->make_get_state(),
    context->task_parameters(),
    request,
    std::move(update),
    [](const auto&) {},
    [](const auto&) {},
    std::move(finished));
}

//==============================================================================
auto ResponsiveWait::Description::make_indefinite(
  std::size_t waiting_point,
  rmf_traffic::Duration update_period) -> ConstDescriptionPtr
{
  return std::make_shared<Description>(
    Description(waiting_point, update_period));
}

//==============================================================================
ResponsiveWait::Description::Description(
  std::size_t waiting_point_,
  rmf_traffic::Duration period_)
: rmf_task_sequence::events::Placeholder::Description(
    "Responsive Wait", "Waiting at a location without blocking traffic"),
  // TODO(MXG): Make the description specific to the description parameters.
  waiting_point(waiting_point_),
  period(period_)
{
  // Do nothing
}

//==============================================================================
auto ResponsiveWait::Standby::make(
  const AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const rmf_task::ConstParametersPtr& parameters,
  const ResponsiveWait::Description& description,
  std::function<void()> update)
-> std::shared_ptr<Standby>
{
  const auto state = get_state();
  const auto context = state.get<agv::GetContext>()->value;
  const auto header = description.generate_header(state, *parameters);

  auto standby = std::make_shared<Standby>(Standby{description});
  standby->_assign_id = id;
  standby->_context = context;
  standby->_update = std::move(update);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    header.category(),
    header.detail(),
    rmf_task::Event::Status::Standby,
    {},
    context->clock());

  return standby;
}

//==============================================================================
auto ResponsiveWait::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration ResponsiveWait::Standby::duration_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto ResponsiveWait::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  if (!_active)
  {
    _active = Active::make(
      _assign_id,
      _context,
      _description,
      _state,
      _update,
      std::move(finished));
  }

  return _active;
}

//==============================================================================
ResponsiveWait::Standby::Standby(const Description& desc)
: _description(desc)
{
  // Do nothin
}

//==============================================================================
auto ResponsiveWait::Active::make(
  const AssignIDPtr& id,
  agv::RobotContextPtr context,
  Description description,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::make_shared<Active>(Active(std::move(description)));
  active->_assign_id = id;
  active->_context = std::move(context);
  active->_update = std::move(update);
  active->_finished = std::move(finished);
  active->_state = std::move(state);

  active->_begin_movement();
  return active;
}

//==============================================================================
auto ResponsiveWait::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration ResponsiveWait::Active::remaining_time_estimate() const
{
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto ResponsiveWait::Active::backup() const -> Backup
{
  // Backing up is not needed
  return Backup::make(0, {});
}

//==============================================================================
auto ResponsiveWait::Active::interrupt(
  std::function<void()> task_is_interrupted) -> Resume
{
  _state->update_status(Status::Standby);
  _state->update_log().info("Going into standby for an interruption");
  auto resume = Resume::make(
    [w = weak_from_this()]()
    {
      if (const auto self = w.lock())
      {
        self->_interrupted = false;
        self->_next_cycle();
      }
    });

  if (_interrupted || !_go_to_place)
  {
    _interrupted = true;
    task_is_interrupted();
    return resume;
  }

  _interrupted = true;
  _waiting_for_interruption = std::move(task_is_interrupted);
  _go_to_place->cancel();

  return resume;
}

//==============================================================================
void ResponsiveWait::Active::cancel()
{
  _state->update_status(Status::Canceled);
  _state->update_log().info("Received signal to cancel");
  _cancelled = true;
  if (_go_to_place)
    _go_to_place->cancel();
}

//==============================================================================
void ResponsiveWait::Active::kill()
{
  _state->update_status(Status::Killed);
  _state->update_log().info("Received signal to kill");
  _cancelled = true;
  if (_go_to_place)
    _go_to_place->kill();
}

//==============================================================================
ResponsiveWait::Active::Active(Description description)
: _description(description)
{
  // Do nothing
}

//==============================================================================
void ResponsiveWait::Active::_next_cycle()
{
  if (_cancelled)
  {
    _finished();
    return;
  }

  if (_interrupted)
  {
    if (_waiting_for_interruption)
    {
      const auto task_is_interrupted = _waiting_for_interruption;
      _waiting_for_interruption = nullptr;
      task_is_interrupted();
    }

    return;
  }

  _begin_movement();
}

//==============================================================================
void ResponsiveWait::Active::_begin_movement()
{
  auto goal = rmf_traffic::agv::Plan::Goal(
    _description.waiting_point,
    _context->now() + _description.period);

  _go_to_place = GoToPlace::Active::make(
    _assign_id,
    _context,
    std::move(goal),
    {},
    _description.period,
    _state,
    _update,
    [w = weak_from_this()]()
    {
      if (const auto self = w.lock())
        self->_next_cycle();
    });
}

} // namespace events
} // namespace rmf_fleet_adapter
