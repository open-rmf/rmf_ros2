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
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    std::vector<rmf_traffic::agv::Plan::Goal> goals,
    rmf_traffic::schedule::Negotiator::TableViewerPtr viewer,
    rmf_traffic::schedule::Negotiator::ResponderPtr responder,
    ApprovalCallback approval,
    const ProgressEvaluator compliant_leeway)
  : _planner(std::move(planner)),
    _starts(std::move(starts)),
    _goals(std::move(goals)),
    _viewer(std::move(viewer)),
    _responder(std::move(responder)),
    _approval(std::move(approval)),
    _evaluator(compliant_leeway)
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<Negotiate> Negotiate::path(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    rmf_traffic::agv::Plan::Goal goal,
    rmf_traffic::schedule::Negotiator::TableViewerPtr viewer,
    rmf_traffic::schedule::Negotiator::ResponderPtr responder,
    ApprovalCallback approval,
    const ProgressEvaluator compliant_leeway)
{
  return std::make_shared<Negotiate>(
        std::move(planner),
        std::move(starts),
        std::vector<rmf_traffic::agv::Plan::Goal>({std::move(goal)}),
        std::move(viewer),
        std::move(responder),
        std::move(approval),
        compliant_leeway);
}

//==============================================================================
std::shared_ptr<Negotiate> Negotiate::emergency_pullover(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    rmf_traffic::schedule::Negotiation::Table::ViewerPtr viewer,
    rmf_traffic::schedule::Negotiator::ResponderPtr responder,
    ApprovalCallback approval,
    const ProgressEvaluator compliant_leeway)
{
  const auto& graph = planner->get_configuration().graph();
  const std::size_t N = graph.num_waypoints();

  std::vector<rmf_traffic::agv::Plan::Goal> goals;
  goals.reserve(N);
  for (std::size_t i=0; i < N; ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    if (wp.is_parking_spot())
      goals.push_back(wp.index());
  }

  return std::make_shared<Negotiate>(
        std::move(planner),
        std::move(starts),
        std::move(goals),
        std::move(viewer),
        std::move(responder),
        std::move(approval),
        compliant_leeway);
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
void Negotiate::_resume_next()
{
  const auto top = _resume_jobs.top();
  _resume_jobs.pop();
  _current_jobs.insert(top);
  top->resume();
}

} // namespace services
} // namespace rmf_fleet_adapter
