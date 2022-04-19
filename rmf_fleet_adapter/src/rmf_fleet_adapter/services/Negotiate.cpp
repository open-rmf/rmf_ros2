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

#include "Negotiate.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
Negotiate::Negotiate(
  rmf_traffic::PlanId assigned_id,
  std::shared_ptr<const rmf_traffic::agv::Planner> planner,
  rmf_traffic::agv::Plan::StartSet starts,
  std::vector<rmf_traffic::agv::Plan::Goal> goals,
  std::vector<rmf_traffic::agv::Plan::Goal> followed_by,
  rmf_traffic::schedule::Negotiator::TableViewerPtr viewer,
  rmf_traffic::schedule::Negotiator::ResponderPtr responder,
  ApprovalCallback approval,
  const ProgressEvaluator evaluator,
  std::vector<rmf_traffic::Route> initial_itinerary)
// We add 1000 to the assigned ID to give room for other plan adjustments that
// could happen while the negotiation is ongoing.
: _plan_id(assigned_id + 1000),
  _planner(std::move(planner)),
  _starts(std::move(starts)),
  _goals(std::move(goals)),
  _followed_by(std::move(followed_by)),
  _viewer(std::move(viewer)),
  _responder(std::move(responder)),
  _approval(std::move(approval)),
  _initial_itinerary(std::move(initial_itinerary)),
  _evaluator(evaluator)
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<Negotiate> Negotiate::path(
  const rmf_traffic::PlanId plan_id,
  std::shared_ptr<const rmf_traffic::agv::Planner> planner,
  rmf_traffic::agv::Plan::StartSet starts,
  rmf_traffic::agv::Plan::Goal goal,
  std::vector<rmf_traffic::agv::Plan::Goal> followed_by,
  rmf_traffic::schedule::Negotiator::TableViewerPtr viewer,
  rmf_traffic::schedule::Negotiator::ResponderPtr responder,
  ApprovalCallback approval,
  const ProgressEvaluator evaluator,
  std::vector<rmf_traffic::Route> initial_itinerary)
{
  return std::make_shared<Negotiate>(
    plan_id,
    std::move(planner),
    std::move(starts),
    std::vector<rmf_traffic::agv::Plan::Goal>({std::move(goal)}),
    std::move(followed_by),
    std::move(viewer),
    std::move(responder),
    std::move(approval),
    evaluator,
    std::move(initial_itinerary));
}

//==============================================================================
std::shared_ptr<Negotiate> Negotiate::emergency_pullover(
  const rmf_traffic::PlanId plan_id,
  std::shared_ptr<const rmf_traffic::agv::Planner> planner,
  rmf_traffic::agv::Plan::StartSet starts,
  rmf_traffic::schedule::Negotiation::Table::ViewerPtr viewer,
  rmf_traffic::schedule::Negotiator::ResponderPtr responder,
  ApprovalCallback approval,
  const ProgressEvaluator evaluator)
{
  const auto& graph = planner->get_configuration().graph();
  const std::size_t N = graph.num_waypoints();

  std::vector<rmf_traffic::agv::Plan::Goal> goals;
  goals.reserve(N);
  for (std::size_t i = 0; i < N; ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    if (wp.is_parking_spot())
      goals.push_back(wp.index());
  }

  return std::make_shared<Negotiate>(
    plan_id,
    std::move(planner),
    std::move(starts),
    std::move(goals),
    std::vector<rmf_traffic::agv::Plan::Goal>(),
    std::move(viewer),
    std::move(responder),
    std::move(approval),
    evaluator);
}

//==============================================================================
void Negotiate::interrupt()
{
  *_interrupted = true;
}

//==============================================================================
void Negotiate::discard()
{
  _discarded = true;
  interrupt();
}

//==============================================================================
bool Negotiate::discarded() const
{
  return _discarded || _viewer->defunct();
}

//==============================================================================
const rmf_traffic::schedule::Negotiator::ResponderPtr&
Negotiate::responder() const
{
  return _responder;
}

//==============================================================================
Negotiate::~Negotiate()
{
  // We violate the Rule of Three/Five/Zero here because it's a good idea to
  // flip the interrupt flag just in case there are any dangling planning jobs
  // still running.
  //
  // TODO(MXG): Consider creating a simple Rule of Zero wrapper class for
  // _interrupted that will set its value to true upon destruction.
  *_interrupted = true;
}

//==============================================================================
void Negotiate::_resume_next()
{
  const auto top = _resume_jobs.top();
  _resume_jobs.pop();
  _current_jobs.insert(top);
  top->resume();
}

} // namespace services
} // namespace rmf_fleet_adapter
