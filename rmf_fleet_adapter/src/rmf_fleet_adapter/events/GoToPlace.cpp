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

#include "GoToPlace.hpp"

#include <rmf_traffic/schedule/StubbornNegotiator.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
auto GoToPlace::Standby::make(
  const AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const rmf_task::ConstParametersPtr& parameters,
  const rmf_task_sequence::events::GoToPlace::Description& description,
  std::function<void()> update,
  std::optional<rmf_traffic::Duration> tail_period)
-> std::shared_ptr<Standby>
{
  const auto state = get_state();
  const auto context = state.get<agv::GetContext>()->value;
  const auto header = description.generate_header(state, *parameters);

  auto standby = std::make_shared<Standby>();
  standby->_assign_id = id;
  standby->_context = context;
  standby->_goal = description.destination();
  standby->_time_estimate = header.original_duration_estimate();
  standby->_tail_period = tail_period;
  standby->_update = std::move(update);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    header.category(),
    header.detail(),
    rmf_task::Event::Status::Standby);

  return standby;
}

//==============================================================================
auto GoToPlace::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration GoToPlace::Standby::duration_estimate() const
{
  return _time_estimate;
}

//==============================================================================
auto GoToPlace::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  if (!_active)
  {
    _active = Active::make(
      _assign_id,
      _context,
      _goal,
      _tail_period,
      _state,
      _update,
      std::move(finished));
  }

  return _active;
}

//==============================================================================
auto GoToPlace::Active::make(
  const AssignIDPtr& id,
  agv::RobotContextPtr context,
  rmf_traffic::agv::Plan::Goal goal,
  std::optional<rmf_traffic::Duration> tail_period,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::make_shared<Active>();
  active->_assign_id = id;
  active->_context = std::move(context);
  active->_goal = std::move(goal);
  active->_tail_period = tail_period;
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
Negotiator::NegotiatePtr GoToPlace::Active::_respond(
  const Negotiator::TableViewerPtr& table_view,
  const Negotiator::ResponderPtr& responder)
{
  if (_context->is_stubborn())
  {
    rmf_traffic::schedule::StubbornNegotiator(_context->itinerary())
    .respond(table_view, responder);
    return nullptr;
  }

  auto approval_cb = [w = weak_from_this()](
    const rmf_traffic::agv::Plan& plan)
    -> std::optional<rmf_traffic::schedule::ItineraryVersion>
    {
      if (auto self = w.lock())
      {
        self->_execute_plan(plan);
        return self->_context->itinerary().version();
      }

      return std::nullopt;
    };

  const auto evaluator = Negotiator::make_evaluator(table_view);
  return services::Negotiate::path(
    _context->planner(), _context->location(), _goal, table_view,
    responder, std::move(approval_cb), std::move(evaluator));
}

} // namespace events
} // namespace rmf_fleet_adapter
