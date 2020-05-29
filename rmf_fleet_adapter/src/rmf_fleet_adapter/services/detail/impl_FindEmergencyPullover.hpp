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

#ifndef SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_FINDEMERGENCYPULLOVER_HPP
#define SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_FINDEMERGENCYPULLOVER_HPP

#include "../FindEmergencyPullover.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
template<typename Subscriber>
void FindEmergencyPullover::operator()(const Subscriber& s)
{
  const auto& graph = _planner->get_configuration().graph();
  const std::size_t N = graph.num_waypoints();
  _search_jobs.reserve(N);
  for (std::size_t i=0; i < N; ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    if (wp.is_parking_spot())
    {
      auto search = std::make_shared<jobs::SearchForPath>(
            _planner, _starts, wp.index(), _schedule, _participant_id);

      const bool keep =
          _greedy_evaluator.initialize(search->greedy().progress())
          || _compliant_evaluator.initialize(search->compliant().progress());

      if (keep)
        _search_jobs.emplace_back(std::move(search));
    }
  }

  const std::size_t N_jobs = _search_jobs.size();

  for (const auto& job : _search_jobs)
    job->set_cost_limit(estimate_leeway * _greedy_evaluator.best_estimate.cost);

  _search_sub = rmf_rxcpp::make_job_from_action_list(_search_jobs)
      .subscribe(
        [this, s, N_jobs](
          const jobs::SearchForPath::Result& progress)
  {
    auto* greedy = progress.greedy_job;
    auto* compliant = progress.compliant_job;

    bool resume_greedy = false;
    if (greedy)
    {
      if (_greedy_evaluator.evaluate(greedy->progress()))
        resume_greedy = true;
    }

    bool resume_compliant = false;
    if (compliant)
    {
      if (_compliant_evaluator.evaluate(compliant->progress()))
        resume_compliant = true;
    }

    if (greedy && compliant)
    {
      auto& compliant_progress = compliant->progress();
      auto& greedy_progress = greedy->progress();
      if (!compliant_progress.success())
      {
        if (!compliant_progress.cost_estimate())
        {
          resume_compliant = false;
          ++_compliant_evaluator.finished_count;
        }
        else if (greedy_progress.success())
        {
          if (greedy_progress->get_cost() * compliance_leeway <
              *compliant_progress.cost_estimate())
          {
            resume_compliant = false;
            ++_compliant_evaluator.finished_count;
          }
        }
      }
    }

    if (_compliant_evaluator.finished_count >= N_jobs
        && _greedy_evaluator.finished_count >= N_jobs)
    {
      if (_compliant_evaluator.best_result.progress)
        s.on_next(*_compliant_evaluator.best_result.progress);
      else if (_greedy_evaluator.best_result.progress)
        s.on_next(*_greedy_evaluator.best_result.progress);
      else
      {
        s.on_error(std::make_exception_ptr(
                     std::runtime_error(
                       "[FindEmergencyPullover] Unable to find a plan")));
      }

      s.on_completed();
      return;
    }

    if (resume_greedy)
      greedy->resume();

    if (resume_compliant)
      compliant->resume();
  });
}

} // namespace services
} // namespace rmf_fleet_adapter


#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_FINDEMERGENCYPULLOVER_HPP
