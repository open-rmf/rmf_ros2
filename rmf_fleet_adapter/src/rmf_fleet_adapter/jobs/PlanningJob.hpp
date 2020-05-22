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

#ifndef SRC__RMF_FLEET_ADAPTER__JOBS__PLANNINGJOB_HPP
#define SRC__RMF_FLEET_ADAPTER__JOBS__PLANNINGJOB_HPP

#include <rmf_rxcpp/RxJobs.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/RouteValidator.hpp>
#include <rmf_traffic/schedule/Snapshot.hpp>

namespace rmf_fleet_adapter {
namespace jobs {

//==============================================================================
class Planning
{
public:
  struct Result
  {
    Planning& job;
  };

  Planning(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    const rmf_traffic::agv::Plan::StartSet& starts,
    rmf_traffic::agv::Plan::Goal goal,
    rmf_traffic::agv::Plan::Options options)
    : _current_result(
        planner->setup(starts, std::move(goal), std::move(options)))
  {
    // Do nothing
  }

  Planning(rmf_traffic::agv::Planner::Result _setup)
    : _current_result(std::move(_setup))
  {
    // Do nothing
  }

  template<typename Subscriber, typename Worker>
  void operator()(const Subscriber& s, const Worker& w)
  {
    if (_discarded)
    {
      s.on_completed();
      return;
    }

    _current_result.resume();
    s.on_next(Result{*this});
    if (_current_result.success() || !_current_result.cost_estimate())
    {
      // The plan is either finished or is guaranteed to never finish
      s.on_completed();
      return;
    }

    w.schedule([this, s, w](const auto&)
    {
      (*this)(s, w);
    });
  }

  void discard()
  {
    _discarded = true;
  }

  rmf_traffic::agv::Planner::Result& progress()
  {
    return _current_result;
  }

  const rmf_traffic::agv::Planner::Result& progress() const
  {
    return _current_result;
  }

private:
  rmf_traffic::agv::Planner::Result _current_result;
  bool _discarded = false;
};

//==============================================================================
class FindPath
{
public:

  FindPath(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    rmf_traffic::agv::Plan::Goal goal,
    std::shared_ptr<const rmf_traffic::schedule::Snapshot> schedule,
    rmf_traffic::schedule::ParticipantId participant_id)
    : _planner(std::move(planner)),
      _starts(std::move(starts)),
      _goal(std::move(goal)),
      _schedule(std::move(schedule)),
      _participant_id(participant_id),
      _event_loop(rxcpp::observe_on_event_loop())
  {
    // Do nothing
  }

  using Result = rmf_traffic::agv::Plan::Result;

  template<typename Subscriber>
  void operator()(const Subscriber& s)
  {
    s.add([this]()
    {
      // This gets triggered if the subscription is discarded
      if (_greedy_job)
        _greedy_job->discard();

      if (_compliant_job)
        _compliant_job->discard();
    });

    auto greedy_options = _planner->get_default_options();
    greedy_options.validator(nullptr);

    auto greedy_setup = _planner->setup(_starts, _goal, greedy_options);
    if (!greedy_setup.cost_estimate())
    {
      // If this ever happens, then there is a serious bug.
      const auto& desc = _schedule->get_participant(_participant_id);
      std::cerr << "[FindPathJob] CRITICAL ERROR: Impossible plan requested! "
                << "Participant [" << desc->name() << "] owned by ["
                << desc->owner() << "] Requested path";
      for (const auto& start : _starts)
        std::cerr << " (" << start.waypoint() << ")";
      std::cerr << " --> (" << _goal.waypoint() << ")" << std::endl;

      assert(false);
      s.on_completed();
      return;
    }

    const double base_cost = *greedy_setup.cost_estimate();
    greedy_setup.options().maximum_cost_estimate(greedy_leeway*base_cost);

    auto compliant_options = _planner->get_default_options();
    compliant_options.validator(
          rmf_traffic::agv::ScheduleRouteValidator::make(
            _schedule, _participant_id));
    compliant_options.maximum_cost_estimate(compliant_leeway*base_cost);
    auto compliant_setup = _planner->setup(_starts, _goal, compliant_options);

    _greedy_job = std::make_shared<Planning>(std::move(greedy_setup));
    _compliant_job = std::make_shared<Planning>(std::move(compliant_setup));

    _greedy_sub = rmf_rxcpp::make_job<Planning::Result>(_greedy_job)
        .observe_on(_event_loop)
        .subscribe(
          [this, s, base_cost](const Planning::Result& result)
    {
      const auto& r = result.job.progress();
      if (r.success())
      {
        if (_compliant_failed)
        {
          s.on_next(r);
          s.on_completed();
          return;
        }

        // If the job has succeeded, record its result
        _greedy_result = r;
        return;
      }

      const double current_cost = r.cost_estimate()?
            *r.cost_estimate() : std::numeric_limits<double>::infinity();

      const auto& desc = _schedule->get_participant(_participant_id);
      // If the job has not succeeded, then something very suspicious is
      // happening. The initial cost estimate must be very very bad for this to
      // occur, which would imply broader systemic issues in the planner.
      std::cerr << "[FindPathJob] CRITICAL ERROR: Failed to find an acceptable "
                << "greedy solution. Participant [" << desc->name() << "] "
                << "owned by [" << desc->owner() << "] Requested path";
      for (const auto& start : _starts)
        std::cerr << " (" << start.waypoint() << ")";
      std::cerr << " --> (" << _goal.waypoint() << "). Base cost: "
                << base_cost << " | Leeway factor: " << greedy_leeway
                << " | Current cost: " << current_cost << std::endl;
      assert(false);

      // We'll return the failed plan, I guess. If the greedy planner fails,
      // then there is no hope of the compliant planner succeeding.
      s.on_next(r);
      s.on_completed();
    });

    _compliant_sub = rmf_rxcpp::make_job<Planning::Result>(_compliant_job)
        .observe_on(_event_loop)
        .subscribe(
          [this, s](const Planning::Result& result)
    {
      auto& r = result.job.progress();
      if (r.success())
      {
        // Return the successful schedule-compliant plan
        s.on_next(r);
        s.on_completed();
        return;
      }

      // Check if it's worth giving this planner any more time
      if (_greedy_result)
      {
        const double new_maximum =
            compliant_leeway * (*_greedy_result)->get_cost();
        if (*r.options().maximum_cost_estimate() < new_maximum)
        {
          // Push the maximum out a bit more and let the job try again.
          r.options().maximum_cost_estimate(new_maximum);
          return;
        }

        // Return the greedy result
        s.on_next(*_greedy_result);
        s.on_completed();
        return;
      }

      // Discard the job because it can no longer produce an acceptable result
      result.job.discard();
      _compliant_failed = true;
    });
  }

private:
  std::shared_ptr<const rmf_traffic::agv::Planner> _planner;
  rmf_traffic::agv::Plan::StartSet _starts;
  rmf_traffic::agv::Plan::Goal _goal;
  std::shared_ptr<const rmf_traffic::schedule::Snapshot> _schedule;
  rmf_traffic::schedule::ParticipantId _participant_id;

  // The greedy job makes an optimal plan that ignores all other schedule
  // participants. It is used for two purposes:
  // 1. Provide a reference for what an acceptable cost is for the compliant job
  // 2. Provide a backup plan if a compliant job can't be found
  std::shared_ptr<Planning> _greedy_job;
  rmf_utils::optional<Result> _greedy_result;
  rxcpp::subscription _greedy_sub;

  // The compliant job makes the plan which is optimal without conflicting with
  // any other traffic currently on the schedule. In some cases, it might not
  // be feasible to find an acceptable compliant job, either because
  std::shared_ptr<Planning> _compliant_job;
  rmf_utils::optional<Result> _compliant_result;
  rxcpp::subscription _compliant_sub;
  bool _compliant_failed = false;

  decltype(rxcpp::observe_on_event_loop()) _event_loop;

  // TODO(MXG): Make these leeway factors configurable
  const double greedy_leeway = 20.0;
  const double compliant_leeway = 5.0;
};

//==============================================================================
class FindEmergencyPullover
{
public:

  FindEmergencyPullover()
  {
    // TODO
  }

  using Result = rmf_traffic::agv::Plan::Result;

  template<typename Subscriber>
  void operator()(const Subscriber& s)
  {
    // TODO
  }

};

} // namespace jobs
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__JOBS__PLANNINGJOB_HPP
