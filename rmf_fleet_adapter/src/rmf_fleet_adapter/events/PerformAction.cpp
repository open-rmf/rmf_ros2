/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "PerformAction.hpp"

#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
void PerformAction::add(rmf_task_sequence::Event::Initializer& initializer)
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
auto PerformAction::Standby::make(
  const AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const rmf_task::ConstParametersPtr& parameters,
  const rmf_task_sequence::events::PerformAction::Description& description,
  std::function<void()> update) -> std::shared_ptr<Standby>
{
  const auto state = get_state();
  const auto context = state.get<agv::GetContext>()->value;
  const auto header = description.generate_header(state, *parameters);

  auto standby = std::make_shared<Standby>(Standby{description});
  standby->_assign_id = id;
  standby->_context = context;
  standby->_time_estimate = header.original_duration_estimate();
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
PerformAction::Standby::Standby(
  rmf_task_sequence::events::PerformAction::Description description)
: _description(std::move(description))
{
  // Do nothing
}

//==============================================================================
auto PerformAction::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration PerformAction::Standby::duration_estimate() const
{
  return _time_estimate;
}

//==============================================================================
auto PerformAction::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  if (!_active)
  {
    _active = Active::make(
      _assign_id,
      _context,
      _description.category(),
      _description.description(),
      _time_estimate,
      _state,
      _update,
      std::move(finished));
  }

  return _active;
}

//==============================================================================
auto PerformAction::Active::make(
  const AssignIDPtr& id,
  agv::RobotContextPtr context,
  const std::string& category,
  nlohmann::json desc,
  rmf_traffic::Duration time_estimate,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::make_shared<Active>(
    Active(std::move(category), std::move(desc), time_estimate));
  active->_assign_id = id;
  active->_context = std::move(context);
  active->_update = std::move(update);
  active->_finished = std::move(finished);
  active->_state = std::move(state);
  active->_execute_action();
  active->_expected_finish_time =
    active->_context->now() + time_estimate;
  return active;
}

//==============================================================================
PerformAction::Active::Active(
  const std::string& category,
  nlohmann::json desc,
  rmf_traffic::Duration time_estimate)
: _action_category(std::move(category)),
  _action_description(std::move(desc)),
  _time_estimate(std::move(time_estimate))
{
  // Do nothing
}

//==============================================================================
auto PerformAction::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration PerformAction::Active::remaining_time_estimate() const
{
  auto execution = _execution_data.lock();
  if (!execution)
    return rmf_traffic::Duration(0);

  if (execution->remaining_time.has_value())
    return execution->remaining_time.value();

  // If an estimate is not provided we compute one based on the expected finish
  // time
  const auto estimate =
    std::max(rmf_traffic::Duration(0),
      _expected_finish_time - _context->now());
  return estimate;
}

//==============================================================================
auto PerformAction::Active::backup() const -> Backup
{
  // TODO: Consider adding a function to ActionExecution that allows the
  // system integrator to periodically send in backup messages. For now, we
  // do not generate backup messages.
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto PerformAction::Active::interrupt(
  std::function<void()> task_is_interrupted) -> Resume
{
  _state->update_status(Status::Standby);
  _state->update_log().info("Going into standby for an interruption");
  _state->update_dependencies({});

  _context->worker().schedule(
    [task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });

  // TODO(YV): Currently we will ask the robot to perform the action again when
  // resumed. Future work can be to receive a callback from RobotUpdateHandle
  // which can be used to ask the robot to resume from its interrupted state.
  // This callback can be accepted by RobotUpdateHandle::ActionExecution
  return Resume::make(
    [w = weak_from_this()]()
    {
      if (const auto self = w.lock())
      {
        self->_execute_action();
      }
    });
}

//==============================================================================
void PerformAction::Active::cancel()
{
  _state->update_status(Status::Canceled);
  _state->update_log().info("Received signal to cancel");
  _finished();
  if (auto data = _execution_data.lock())
    data->okay = false;
}

//==============================================================================
void PerformAction::Active::kill()
{
  _state->update_status(Status::Killed);
  _state->update_log().info("Received signal to kill");
  _finished();
  if (auto data = _execution_data.lock())
    data->okay = false;
}

//==============================================================================
void PerformAction::Active::_execute_action()
{
  auto action_executor = _context->action_executor();
  if (action_executor == nullptr)
  {
    // The action_executor has not been set by the user
    _state->update_status(Status::Error);
    const std::string msg = "ActionExecutor not set via RobotUpdateHandle. "
      "Unable to perform the requested action.";
    _state->update_log().error(msg);
    _finished();
    return;
  }

  auto finished = [state = _state, cb = _finished]()
    {
      state->update_status(Status::Completed);
      cb();
    };

  auto data = std::make_shared<ExecutionData>(
    std::move(finished), _state, std::nullopt);
  _execution_data = data;

  auto action_execution =
    agv::RobotUpdateHandle::ActionExecution::Implementation::make(data);

  _state->update_status(Status::Underway);
  action_executor(
    _action_category,
    _action_description,
    std::move(action_execution));
}

} // namespace events
} // namespace rmf_fleet_adapter
