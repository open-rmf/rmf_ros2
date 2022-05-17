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

#include "EmergencyPullover.hpp"

#include <rmf_traffic/schedule/StubbornNegotiator.hpp>

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/Placeholder.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
class EmergencyPulloverDescription
  : public rmf_task_sequence::events::Placeholder::Description
{
public:
  EmergencyPulloverDescription()
  : rmf_task_sequence::events::Placeholder::Description(
      "Emergency Pullover", "")
  {
    // Do nothing
  }
};

//==============================================================================
rmf_task::Task::ActivePtr EmergencyPullover::start(
  const std::string& task_id,
  agv::RobotContextPtr& context,
  std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
  std::function<void()> finished)
{
  static auto activator = _make_activator(context->clock());
  rmf_task_sequence::Task::Builder builder;
  builder.add_phase(
    rmf_task_sequence::phases::SimplePhase::Description::make(
      std::make_shared<EmergencyPulloverDescription>()), {});

  const auto desc = builder.build("Emergency Pullover", "");
  const rmf_task::Request request(task_id, context->now(), nullptr, desc, true);

  return activator.activate(
    context->make_get_state(),
    context->task_parameters(),
    request,
    std::move(update),
    [](const auto&) {},
    [](const auto&) {},
    std::move(finished));
}

//==============================================================================
auto EmergencyPullover::Standby::make(
  const AssignIDPtr& id,
  const agv::RobotContextPtr& context,
  std::function<void()> update) -> std::shared_ptr<Standby>
{
  auto standby = std::make_shared<Standby>();
  standby->_assign_id = id;
  standby->_context = context;
  standby->_update = std::move(update);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    "Emergency pullover",
    "",
    rmf_task::Event::Status::Standby,
    {},
    context->clock());

  return standby;
}

//==============================================================================
auto EmergencyPullover::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration EmergencyPullover::Standby::duration_estimate() const
{
  // An emergency pullover will last indefinitely until it gets cancelled, which
  // may happen at any time.
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto EmergencyPullover::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  if (!_active)
  {
    _active = Active::make(
      _assign_id,
      _context,
      _state,
      _update,
      std::move(finished));
  }

  return _active;
}

//==============================================================================
auto EmergencyPullover::Active::make(
  const AssignIDPtr& id,
  agv::RobotContextPtr context,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::make_shared<Active>();
  active->_assign_id = id;
  active->_context = std::move(context);
  active->_update = std::move(update);
  active->_finished = std::move(finished);
  active->_state = std::move(state);
  active->_negotiator =
    Negotiator::make(
    active->_context,
    [w = active->weak_from_this()](
      const auto& t, const auto& r) -> Negotiator::NegotiatePtr
    {
      if (const auto self = w.lock())
        return self->_respond(t, r);

      r->forfeit({});
      return nullptr;
    });

  active->_find_plan();
  return active;
}

//==============================================================================
auto EmergencyPullover::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration EmergencyPullover::Active::remaining_time_estimate() const
{
  // An emergency pullover will last indefinitely until it gets cancelled, which
  // may happen at any time.
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto EmergencyPullover::Active::backup() const -> Backup
{
  // EmergencyPullover doesn't need to be backed up
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto EmergencyPullover::Active::interrupt(
  std::function<void()> task_is_interrupted) -> Resume
{
  _negotiator->clear_license();
  _is_interrupted = true;
  _execution = std::nullopt;

  _state->update_status(Status::Standby);
  _state->update_log().info("Going into standby for an interruption");
  _state->update_dependencies({});

  if (const auto command = _context->command())
    command->stop();

  _context->itinerary().clear();

  _context->worker().schedule(
    [task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });

  return Resume::make(
    [w = weak_from_this()]()
    {
      if (const auto self = w.lock())
      {
        self->_negotiator->claim_license();
        self->_is_interrupted = false;
        self->_find_plan();
      }
    });
}

//==============================================================================
void EmergencyPullover::Active::cancel()
{
  _execution = std::nullopt;
  _state->update_status(Status::Canceled);
  _state->update_log().info("Received signal to cancel");
  _finished();
}

//==============================================================================
void EmergencyPullover::Active::kill()
{
  _execution = std::nullopt;
  _state->update_status(Status::Killed);
  _state->update_log().info("Received signal to kill");
  _finished();
}

//==============================================================================
void EmergencyPullover::Active::_find_plan()
{
  if (_is_interrupted)
    return;

  _state->update_status(Status::Underway);
  _state->update_log().info("Searching for an emergency pullover");

  _find_pullover_service = std::make_shared<services::FindEmergencyPullover>(
    _context->planner(), _context->location(), _context->schedule()->snapshot(),
    _context->itinerary().id(), _context->profile());

  _pullover_subscription =
    rmf_rxcpp::make_job<services::FindEmergencyPullover::Result>(
    _find_pullover_service)
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
    [w = weak_from_this()](
      const services::FindEmergencyPullover::Result& result)
    {
      const auto self = w.lock();
      if (!self)
        return;

      if (!result)
      {
        // The planner could not find any pullover
        self->_state->update_status(Status::Error);
        self->_state->update_log().error("Failed to find a pullover");

        self->_execution = std::nullopt;
        self->_schedule_retry();

        self->_context->worker().schedule(
          [update = self->_update](const auto&) { update(); });

        return;
      }

      self->_state->update_status(Status::Underway);
      self->_state->update_log().info("Found an emergency pullover");

      auto full_itinerary = result->get_itinerary();
      self->_execute_plan(
        self->_context->itinerary().assign_plan_id(),
        *std::move(result),
        std::move(full_itinerary));

      self->_find_pullover_service = nullptr;
      self->_retry_timer = nullptr;
    });

  _find_pullover_timeout = _context->node()->try_create_wall_timer(
    std::chrono::seconds(10),
    [
      weak_service = _find_pullover_service->weak_from_this(),
      weak_self = weak_from_this()
    ]()
    {
      if (const auto service = weak_service.lock())
        service->interrupt();

      if (const auto self = weak_self.lock())
        self->_find_pullover_timeout = nullptr;
    });

  _update();
}

//==============================================================================
void EmergencyPullover::Active::_schedule_retry()
{
  if (_retry_timer)
    return;

  // TODO(MXG): Make the retry timing configurable
  _retry_timer = _context->node()->try_create_wall_timer(
    std::chrono::seconds(5),
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_retry_timer = nullptr;
      if (self->_execution.has_value())
        return;

      self->_find_plan();
    });
}

//==============================================================================
void EmergencyPullover::Active::_execute_plan(
  const rmf_traffic::PlanId plan_id,
  rmf_traffic::agv::Plan plan,
  rmf_traffic::schedule::Itinerary full_itinerary)
{
  if (_is_interrupted)
    return;

  _execution = ExecutePlan::make(
    _context, plan_id, std::move(plan), std::move(full_itinerary), _assign_id,
    _state, _update, _finished, std::nullopt);

  if (!_execution.has_value())
  {
    _state->update_status(Status::Error);
    _state->update_log().error(
      "Invalid (empty) plan generated. Will retry soon. "
      "Please report this incident to the Open-RMF developers.");
    _schedule_retry();
  }
}

//==============================================================================
Negotiator::NegotiatePtr EmergencyPullover::Active::_respond(
  const Negotiator::TableViewerPtr& table_view,
  const Negotiator::ResponderPtr& responder)
{
  auto approval_cb = [w = weak_from_this()](
    const rmf_traffic::PlanId plan_id,
    const rmf_traffic::agv::Plan& plan,
    rmf_traffic::schedule::Itinerary full_itinerary)
    -> std::optional<rmf_traffic::schedule::ItineraryVersion>
    {
      if (auto self = w.lock())
      {
        self->_execute_plan(plan_id, plan, std::move(full_itinerary));
        return self->_context->itinerary().version();
      }

      return std::nullopt;
    };

  const auto evaluator = Negotiator::make_evaluator(table_view);
  return services::Negotiate::emergency_pullover(
    _context->itinerary().assign_plan_id(), _context->planner(),
    _context->location(), table_view,
    responder, std::move(approval_cb), std::move(evaluator));
}

//==============================================================================
rmf_task::Activator EmergencyPullover::_make_activator(
  std::function<rmf_traffic::Time()> clock)
{
  auto event_activator =
    std::make_shared<rmf_task_sequence::Event::Initializer>();
  event_activator->add<EmergencyPulloverDescription>(
    [](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr&,
      const Description&,
      std::function<void()> update) -> rmf_task_sequence::Event::StandbyPtr
    {
      return EmergencyPullover::Standby::make(
        id, get_state().get<agv::GetContext>()->value, std::move(update));
    },
    [](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr&,
      const EmergencyPulloverDescription&,
      const nlohmann::json&,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished) -> rmf_task_sequence::Event::ActivePtr
    {
      return EmergencyPullover::Standby::make(
        id, get_state().get<agv::GetContext>()->value, std::move(update))
      ->begin(std::move(checkpoint), std::move(finished));
    });

  auto phase_activator =
    std::make_shared<rmf_task_sequence::Phase::Activator>();
  rmf_task_sequence::phases::SimplePhase::add(
    *phase_activator, event_activator);

  rmf_task::Activator activator;
  rmf_task_sequence::Task::add(activator, phase_activator, std::move(clock));
  return activator;
}

} // namespace events
} // namespace rmf_fleet_adapter
