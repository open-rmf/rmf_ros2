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
  for (std::size_t i = 0; i < N; ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    if (wp.is_parking_spot())
    {
      auto search = std::make_shared<jobs::SearchForPath>(
        _planner, _starts, wp.index(), _schedule, _participant_id, _profile);

      // Be sure to initialize these individually and not in a single statement,
      // otherwise the logic might short-circuit one of the initialize() calls
      const bool keep_greedy =
        _greedy_evaluator.initialize(search->greedy().progress());

      const bool keep_compliant =
        _compliant_evaluator.initialize(search->compliant().progress());

      if (keep_greedy || keep_compliant)
        _search_jobs.emplace_back(std::move(search));
    }
  }

  const std::size_t N_jobs = _search_jobs.size();
  const double initial_max_cost =
    ProgressEvaluator::DefaultEstimateLeeway
    * _greedy_evaluator.best_estimate.cost;

  for (const auto& job : _search_jobs)
    job->set_cost_limit(initial_max_cost);

  _search_sub = rmf_rxcpp::make_job_from_action_list(_search_jobs)
    .subscribe(
    [weak = weak_from_this(), s, N_jobs](
      const jobs::SearchForPath::Result& progress)
    {
      const auto f = weak.lock();
      if (!f)
        return;

      const auto& greedy = progress.greedy_job;
      const auto& compliant = progress.compliant_job;

      bool resume_compliant = static_cast<bool>(compliant);
      if (compliant && f->_greedy_evaluator.best_result.progress)
      {
        auto& compliant_progress = compliant->progress();
        if (!compliant_progress.success())
        {
          if (!compliant_progress.cost_estimate())
          {
            resume_compliant = false;
            f->_compliant_evaluator.discard(compliant_progress);
          }
          else
          {
            const double best_greedy_cost =
            (*f->_greedy_evaluator.best_result.progress)->get_cost();
            const double compliant_cost = *compliant_progress.cost_estimate();

            if (best_greedy_cost * compliance_leeway < compliant_cost)
            {
              resume_compliant = false;
              f->_compliant_evaluator.discard(compliant_progress);
            }
          }
        }
      }

      bool resume_greedy = false;
      if (jobs::SearchForPath::Type::greedy == progress.type)
      {
        if (f->_greedy_evaluator.evaluate(greedy->progress()))
          resume_greedy = true;
      }

      if (jobs::SearchForPath::Type::compliant == progress.type
      && resume_compliant)
      {
        if (f->_compliant_evaluator.evaluate(compliant->progress()))
          resume_compliant = true;
      }

      if ( (f->_compliant_evaluator.finished_count >= N_jobs
      && f->_greedy_evaluator.finished_count >= N_jobs)
      || f->_interrupted)
      {
        if (f->_compliant_evaluator.best_result.progress)
          s.on_next(*f->_compliant_evaluator.best_result.progress);
        else if (f->_greedy_evaluator.best_result.progress)
          s.on_next(*f->_greedy_evaluator.best_result.progress);
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


#endif \
  // SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_FINDEMERGENCYPULLOVER_HPP
