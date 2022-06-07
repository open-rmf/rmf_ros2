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

#ifndef SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_NEGOTIATE_HPP
#define SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_NEGOTIATE_HPP

#include "../Negotiate.hpp"
#include "../../project_itinerary.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
template<typename Subscriber>
void Negotiate::operator()(const Subscriber& s)
{
  s.add([n = weak_from_this()]()
    {
      // This service will be discarded if it is unsubscribed from
      if (const auto negotiate = n.lock())
        negotiate->discard();
    });

  auto validators =
    rmf_traffic::agv::NegotiatingRouteValidator::Generator(_viewer).all();

  _queued_jobs.reserve(validators.size() * _goals.size());

  auto interrupter = [
    service_interrupted = _interrupted, viewer = _viewer]() -> bool
    {
      return *service_interrupted || viewer->defunct();
    };

  for (const auto& goal : _goals)
  {
    for (const auto& validator : validators)
    {
      auto job = std::make_shared<jobs::Planning>(
        _planner, _starts, goal,
        rmf_traffic::agv::Plan::Options(validator)
        .interrupter(interrupter));

      _evaluator.initialize(job->progress());

      _queued_jobs.emplace_back(std::move(job));
    }
  }

  const double initial_max_cost =
    _evaluator.best_estimate.cost * _evaluator.estimate_leeway;

  const std::size_t N_jobs = _queued_jobs.size();

  for (const auto& job : _queued_jobs)
    job->progress().options().maximum_cost_estimate(initial_max_cost);

  auto check_if_finished = [w = weak_from_this(), s, N_jobs]() -> bool
    {
      const auto self = w.lock();
      if (!self)
        return true;

      if (self->_finished)
        return true;

      if (self->_evaluator.finished_count >= N_jobs || *self->_interrupted)
      {
        if (self->_evaluator.best_result.progress
          && self->_evaluator.best_result.progress->success())
        {
          self->_finished = true;
          // This means we found a successful plan to submit to the negotiation.
          s.on_next(
            Result{
              self->shared_from_this(),
              [r = *self->_evaluator.best_result.progress,
              initial_itinerary = std::move(self->_initial_itinerary),
              followed_by = self->_followed_by,
              planner = self->_planner,
              approval = std::move(self->_approval),
              responder = self->_responder,
              viewer = self->_viewer,
              plan_id = self->_plan_id]()
              {
                std::vector<rmf_traffic::Route> final_itinerary;
                final_itinerary.reserve(
                  initial_itinerary.size() + r->get_itinerary().size());

                for (const auto& it : {initial_itinerary, r->get_itinerary()})
                {
                  for (const auto& route : it)
                  {
                    if (route.trajectory().size() > 1)
                      final_itinerary.push_back(route);
                  }
                }

                final_itinerary = project_itinerary(*r, followed_by, *planner);
                for (const auto& parent : viewer->base_proposals())
                {
                  // Make sure all parent dependencies are accounted for
                  // TODO(MXG): This is kind of a gross hack that we add to
                  // force the lookahead to work for patrols. This approach
                  // should be reworked in a future redesign of the traffic
                  // system.
                  for (auto& r : final_itinerary)
                  {
                    for (std::size_t i = 0; i < parent.itinerary.size(); ++i)
                    {
                      r.add_dependency(
                        r.trajectory().size(),
                        rmf_traffic::Dependency{
                          parent.participant,
                          parent.plan,
                          i,
                          parent.itinerary[i].trajectory().size()
                        });
                    }
                  }
                }

                responder->submit(
                  plan_id,
                  final_itinerary,
                  [
                    plan_id,
                    plan = *r,
                    approval = std::move(approval),
                    final_itinerary
                  ]()
                  -> UpdateVersion
                  {
                    if (approval)
                      return approval(plan_id, plan, final_itinerary);

                    return rmf_utils::nullopt;
                  });
              }
            });

          s.on_completed();
          self->interrupt();
          return true;
        }
        else if (self->_alternatives && !self->_alternatives->empty())
        {
          self->_finished = true;
          // This means we could not find a successful plan, but we have some
          // alternatives to offer the parent in the negotiation.
          s.on_next(
            Result{
              self->shared_from_this(),
              [alts = *self->_alternatives, responder = self->_responder]()
              {
                responder->reject(alts);
              }
            });

          s.on_completed();
          self->interrupt();
          return true;
        }
        else if (!self->_attempting_rollout)
        {
          self->_finished = true;
          // This means we could not find any plan or any alternatives to offer
          // the parent, so all we can do is forfeit.
          s.on_next(
            Result{
              self->shared_from_this(),
              [n = self->shared_from_this()]()
              {
                std::vector<rmf_traffic::schedule::ParticipantId> blockers(
                  n->_blockers.begin(), n->_blockers.end());

                n->_responder->forfeit(std::move(blockers));
              }
            });

          s.on_completed();
          self->interrupt();
          return true;
        }

        // If we land here, that means a rollout is still being calculated, and
        // we will consider the service finished when that rollout is ready
      }

      return false;
    };

  _search_sub = rmf_rxcpp::make_job_from_action_list(_queued_jobs)
    .observe_on(rxcpp::observe_on_event_loop())
    .subscribe(
    [n_weak = weak_from_this(), s,
    check_if_finished = std::move(check_if_finished)](
      const jobs::Planning::Result& result)
    {
      const auto n = n_weak.lock();
      if (!n)
      {
        s.on_completed();
        return;
      }

      if (n->discarded())
      {
        s.on_next(Result{n, []() {}});
        s.on_completed();
        return;
      }

      bool resume = false;
      if (n->_evaluator.evaluate(result.job->progress()))
      {
        resume = true;
      }
      else if (!n->_attempting_rollout && !n->_alternatives
      && n->_viewer->parent_id())
      {
        const auto parent_id = *n->_viewer->parent_id();
        for (const auto p : result.job->progress().blockers())
        {
          if (p == parent_id)
          {
            n->_attempting_rollout = true;
            auto rollout_source = result.job->progress();
            static_cast<rmf_traffic::agv::NegotiatingRouteValidator*>(
              rollout_source.options().validator().get())->mask(parent_id);

            n->_rollout_job = std::make_shared<jobs::Rollout>(
              std::move(rollout_source), parent_id,
              std::chrono::seconds(15), 200);

            n->_rollout_sub =
            rmf_rxcpp::make_job<jobs::Rollout::Result>(n->_rollout_job)
            .observe_on(rxcpp::observe_on_event_loop())
            .subscribe(
              [n, check_if_finished](const jobs::Rollout::Result& result)
              {
                n->_alternatives = result.alternatives;
                n->_attempting_rollout = false;
                check_if_finished();
              });
          }
        }
      }

      if (!check_if_finished())
      {
        const auto job = result.job;
        if (resume)
        {
          if (n->_current_jobs.find(job) != n->_current_jobs.end())
          {
            job->resume();
          }
          else
          {
            n->_resume_jobs.push(job);
          }
        }
        else
        {
          const auto& blockers = job->progress().blockers();
          for (const auto p : blockers)
            n->_blockers.insert(p);

          if (n->_evaluator.best_result.progress != &job->progress())
          {
            job->discard();
            const auto job_it = std::find(
              n->_queued_jobs.begin(), n->_queued_jobs.end(), job);
            assert(job_it != n->_queued_jobs.end());
            n->_queued_jobs.erase(job_it);
          }

          n->_current_jobs.erase(job);
        }

        while (n->_current_jobs.size() < max_concurrent_jobs
        && !n->_resume_jobs.empty())
        {
          n->_resume_next();
        }
      }
    });
}

} // namespace services
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_NEGOTIATE_HPP
