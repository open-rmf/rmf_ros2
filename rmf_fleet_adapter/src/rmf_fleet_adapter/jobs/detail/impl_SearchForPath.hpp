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

#ifndef SRC__RMF_FLEET_ADAPTER__JOBS__DETAIL__IMPL_SEARCHFORPATH_HPP
#define SRC__RMF_FLEET_ADAPTER__JOBS__DETAIL__IMPL_SEARCHFORPATH_HPP

#include "../SearchForPath.hpp"

namespace rmf_fleet_adapter {
namespace jobs {

//==============================================================================
template<typename Subscriber, typename Worker>
void SearchForPath::operator()(const Subscriber& s, const Worker&)
{
  s.add([weak = weak_from_this()]()
    {
      if (const auto search = weak.lock())
      {
        // This gets triggered if the subscription is discarded
        search->interrupt();
      }
    });

  if (!_greedy_job)
  {
    // This means the plan was infeasible from the start, so we will declare
    // that the job is completed without returning any result.
    s.on_error(std::make_exception_ptr(
        std::runtime_error(
          "[SearchForPath] Impossible path requested")));
    return;
  }

  if (_explicit_cost_limit)
  {
    _greedy_job->progress().options().maximum_cost_estimate(
      _explicit_cost_limit);

    _compliant_job->progress().options().maximum_cost_estimate(
      _explicit_cost_limit);
  }

  _greedy_sub = rmf_rxcpp::make_job<Planning::Result>(_greedy_job)
    .observe_on(rxcpp::identity_same_worker(_worker))
    .subscribe(
    [weak = weak_from_this(), s](const Planning::Result& result)
    {
      const auto search = weak.lock();
      if (!search)
        return;

      auto show_compliant = search->_compliant_finished ?
      search->_compliant_job : std::shared_ptr<Planning>(nullptr);

      Result next{search->_greedy_job, show_compliant, Type::greedy};

      const auto& r = result.job->progress();
      if (r.success())
      {
        if (search->_compliant_finished)
        {
          s.on_next(next);
          s.on_completed();
          return;
        }
        else if (search->_explicit_cost_limit)
        {
          s.on_next(next);

          search->_greedy_finished = true;
          return;
        }

        search->_greedy_finished = true;
        return;
      }

      if (!search->_explicit_cost_limit)
      {
        const double current_cost = r.cost_estimate() ?
        *r.cost_estimate() : std::numeric_limits<double>::infinity();
        const double maximum_cost = r.options().maximum_cost_estimate() ?
        *r.options().maximum_cost_estimate() :
        std::numeric_limits<double>::infinity();

        auto opt_to_str = [](const auto& v) -> std::string
        {
          if (v)
            return std::to_string(*v);

          return "null";
        };

        const auto to_string = [](const rmf_traffic::agv::Plan::Start& start)
        {
          std::ostringstream oss;
          oss << "[" << start.waypoint() << "] r:" << start.orientation();
          if (start.lane())
            oss << " | lane: " << *start.lane();
          else
            oss << " | no lane";

          if (start.location())
            oss << " | <" << start.location()->transpose() << ">";

          return oss.str();
        };

        const auto& desc = search->_schedule->get_participant(
          search->_participant_id);
        // If the job has not succeeded, then something very suspicious is
        // happening. The initial cost estimate must be very very bad for this to
        // occur, which would imply broader systemic issues in the planner.
        std::cerr << "[SearchForPath] CRITICAL ERROR: Failed to find an "
                  << "acceptable greedy solution. Participant [" << desc->name()
                  << "] owned by [" << desc->owner() << "] Requested path";
        for (const auto& start : search->_starts)
          std::cerr << " (" << to_string(start) << ")";
        std::cerr << " --> (" << search->_goal.waypoint() << "). Maximum cost: "
                  << maximum_cost << " | Leeway factor: "
                  << search->_greedy_leeway << " | Current cost: " << current_cost
                  << " | Saturated: " << r.saturated() << " (limit: "
                  << opt_to_str(r.options().saturation_limit())
                  << ") | interrupted: " << r.interrupted() << std::endl;
        const auto& v = r.get_configuration().vehicle_traits();
        std::cerr << "linear | v: " << v.linear().get_nominal_velocity()
                  << ", a: " << v.linear().get_nominal_acceleration()
                  << "\nangular | v: " << v.rotational().get_nominal_velocity()
                  << ", a: " << v.rotational().get_nominal_acceleration()
                  << std::endl;
        assert(false);

        // We'll return the failed plan, I guess. If the greedy planner fails,
        // then there is no hope of the compliant planner succeeding.
        s.on_next(next);
        s.on_completed();
        return;
      }

      s.on_next(next);
      // We do not automatically resume, because that should be the choice of
      // whoever we are reporting to
    });

  _compliant_sub = rmf_rxcpp::make_job<Planning::Result>(_compliant_job)
    .observe_on(rxcpp::identity_same_worker(_worker))
    .subscribe(
    [weak = weak_from_this(), s](const Planning::Result& result)
    {
      const auto search = weak.lock();
      if (!search)
        return;

      auto show_greedy = search->_greedy_finished ?
      search->_greedy_job : std::shared_ptr<Planning>(nullptr);

      Result next{show_greedy, search->_compliant_job, Type::compliant};

      auto& r = result.job->progress();
      if (r.success())
      {
        // Return the successful schedule-compliant plan
        if (search->_greedy_finished || search->_explicit_cost_limit)
        {
          s.on_next(next);
        }
        search->_compliant_finished = true;

        if (search->_greedy_finished)
          s.on_completed();

        return;
      }

      if (*search->_interrupt_flag || r.saturated() || !r.cost_estimate())
      {
        if (search->_greedy_finished)
        {
          s.on_next(next);
          s.on_completed();
        }
        else if (search->_explicit_cost_limit)
        {
          s.on_next(next);
        }

        search->_compliant_finished = true;
        return;
      }

      if (search->_explicit_cost_limit)
      {
        // An explicit cost limit means this is part of a Job, so we should
        // report an update whenever we get an update.
        s.on_next(next);
        // We do not automatically resume, because that should be the choice of
        // whoever we are reporting to.
        return;
      }

      if (search->_greedy_finished)
      {
        // We don't have an explicit cost limit, so we'll just check if the
        // greedy job search has granted us any more leeway.
        const double new_maximum =
        search->_compliant_leeway * search->_greedy_job->progress()->get_cost();

        if (*r.options().maximum_cost_estimate() < new_maximum)
        {
          // Push the maximum out a bit more and let the job try again.
          r.options().maximum_cost_estimate(new_maximum);
          result.job->resume();
          return;
        }

        // We shouldn't keep trying, because we have exceeded the cost limit, even
        // when accounting for the greedy plan cost.
        s.on_next(next);
        s.on_completed();
        return;
      }

      // Discard the job because it can no longer produce an acceptable result.
      // The SearchForPath will continue looking for a greedy plan.
      search->_compliant_finished = true;
    });
}

} // namespace jobs
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__JOBS__DETAIL__IMPL_SEARCHFORPATH_HPP
