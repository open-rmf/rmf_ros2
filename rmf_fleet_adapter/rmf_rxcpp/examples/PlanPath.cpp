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

#include "TestMap.hpp"

#include <rmf_rxcpp/RxJobs.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/Rollout.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <iostream>
#include <memory>
#include <queue>
#include <atomic>

//==============================================================================
inline rmf_traffic::Time print_start(const rmf_traffic::Route& route)
{
  std::cout << "(start) --> ";
  std::cout << "(" << 0.0 << "; "
            << route.trajectory().front().position().transpose()
            << ") --> ";

  return *route.trajectory().start_time();
}

//==============================================================================
inline void print_route(
  const rmf_traffic::Route& route,
  const rmf_traffic::Time start_time)
{
  for (auto it = ++route.trajectory().begin(); it != route.trajectory().end();
    ++it)
  {
    const auto& wp = *it;
    if (wp.velocity().norm() > 1e-3)
      continue;

    const auto rel_time = wp.time() - start_time;
    std::cout << "(" << rmf_traffic::time::to_seconds(rel_time) << "; "
              << wp.position().transpose() << ") --> ";
  }
}

//==============================================================================
inline void print_itinerary(
  const rmf_traffic::schedule::Itinerary& itinerary)
{
  auto start_time = print_start(itinerary.front());
  for (const auto& r : itinerary)
    print_route(r, start_time);

  std::cout << "(end)\n" << std::endl;
}

//==============================================================================
struct PlannerAction
{
  struct Progress
  {
    PlannerAction& action;
    rmf_traffic::agv::Planner::Result& result;
  };

  using Result = Progress;

  PlannerAction(
    rmf_traffic::schedule::Database schedule,
    rmf_traffic::agv::Planner::Configuration config,
    rmf_traffic::agv::Planner::Start start,
    rmf_traffic::agv::Planner::Goal goal,
    std::atomic_int& sync_failure)
  : _database(std::move(schedule)),
    _current_result(
      initiate(
        _database,
        std::move(config),
        std::move(start),
        std::move(goal))),
    _sync_failure(sync_failure)
  {
    // Do nothing
  }

  template<typename Subscriber, typename Worker>
  void operator()(const Subscriber& s, const Worker& w)
  {
    if (_discarded)
    {
      std::cout << " !!!!!!!!!!!!!!!!!!!!!! " << std::endl;
      _sync_failure.fetch_add(1);
      return;
    }

    auto r = process();
    s.on_next(Progress{*this, r});
    if (r || _discarded)
    {
      s.on_completed();
      return;
    }

    if (!_discarded)
    {
      w.schedule([this, s, w](const auto&)
        {
          (*this)(s, w);
        });
    }
  }

  inline void discard()
  {
    _discarded = true;
  }

  const rmf_traffic::agv::Planner::Result& process()
  {
    _current_result.resume();
    return _current_result;
  }

  double cost_estimate() const
  {
    assert(_current_result.cost_estimate().has_value());
    return *_current_result.cost_estimate();
  }

  void set_maximum_cost_estimate(double cost)
  {
    _current_result.options().maximum_cost_estimate(cost);
  }

  struct Compare
  {
    bool operator()(
      const std::shared_ptr<PlannerAction>& a,
      const std::shared_ptr<PlannerAction>& b)
    {
      return b->cost_estimate() < a->cost_estimate();
    }
  };

private:
  rmf_traffic::schedule::Database _database;
  rmf_traffic::agv::Planner::Result _current_result;
  bool _discarded = false;
  std::atomic_int& _sync_failure;

  static rmf_traffic::agv::Planner::Result initiate(
    const rmf_traffic::schedule::Viewer& viewer,
    rmf_traffic::agv::Planner::Configuration config,
    rmf_traffic::agv::Planner::Start start,
    rmf_traffic::agv::Planner::Goal goal)
  {
    // Using this do_not_start flag with a value of true will make sure that the
    // planning problem gets started up but is immediately interrupted so that
    // it can be continued later.
    //
    // TODO(MXG): Should we create a Planner::setup() function that does this as
    // an alternative to Planner::plan()?
    rmf_traffic::agv::Planner::Options options(
      rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
        viewer, 10000, config.vehicle_traits().profile()),
      std::chrono::seconds(5));

    return rmf_traffic::agv::Planner(
      std::move(config), std::move(options))
      .setup(std::move(start), std::move(goal));
  }
};

//==============================================================================
struct MetaPlannerAction
{
  using Result = std::vector<rmf_traffic::Route>;

  struct JobResult
  {
    double cost;
    rmf_traffic::agv::Plan plan;
  };

  struct EstimateInfo
  {
    double cost = std::numeric_limits<double>::infinity();
    const PlannerAction* estimator = nullptr;
  };

  std::atomic_int sync_failure;

  std::vector<std::shared_ptr<PlannerAction>> planner_actions;
  rmf_utils::optional<JobResult> best_result;
  std::size_t finished_count = 0;
  EstimateInfo best_estimate;
  EstimateInfo second_best_estimate;
  std::chrono::steady_clock::time_point start_time =
    std::chrono::steady_clock::now();

  template<typename Subscriber>
  void operator()(const Subscriber& s)
  {
    std::atomic_init(&sync_failure, 0);

    //============================================================================
    // vvvvvvvvvvvvvvvvvvvvvvvv Setting up the problem vvvvvvvvvvvvvvvvvvvvvvvvvvv
    const auto graph = make_graph();

    const rmf_traffic::Profile profile(
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(1.0));

    const rmf_traffic::schedule::ParticipantDescription description{
      "participant_0",
      "rxcpp",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    };

    const rmf_traffic::agv::VehicleTraits traits(
      {0.7, 0.3}, {1.0, 0.45}, profile);

    const auto master_schedule =
      std::make_shared<rmf_traffic::schedule::Database>();

    auto p_A = rmf_traffic::schedule::make_participant(
      description, master_schedule);

    auto p_B = rmf_traffic::schedule::make_participant(
      description, master_schedule);

    rmf_traffic::agv::Planner::Configuration config(graph, traits);

    const auto start_B = rmf_traffic::agv::Plan::Start(start_time, 5, 0.0);
    const auto goal_B = rmf_traffic::agv::Plan::Goal(3);
    rmf_traffic::agv::Planner planner_B(
      config,
      {
        rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
          master_schedule, p_B.id(), profile)
      });

    const auto initial_plan = planner_B.plan(start_B, goal_B);
    assert(initial_plan);

    p_B.set(p_B.plan_id_assigner()->assign(), initial_plan->get_itinerary());

    rmf_traffic::agv::Planner planner_A(
      config,
      {
        rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
          master_schedule, p_A.id(), profile)
      });

    // This will fail because the other participant is blocking it.
    const auto start_A = rmf_traffic::agv::Plan::Start(start_time, 3, 0.0);
    const auto goal_A = rmf_traffic::agv::Plan::Goal(7);
    auto result_A = planner_A.plan(start_A, goal_A);
    assert(!result_A);

    rmf_traffic::agv::Rollout rollout_A(result_A);
    auto alternatives = rollout_A.expand(
      p_B.id(), std::chrono::seconds(30), {nullptr});

    std::cout << "Alternatives:" << std::endl;
    for (const auto& alternative : alternatives)
    {
//      print_itinerary(alternative);
      rmf_traffic::schedule::Database database;
      const auto p = database.register_participant(description);
      rmf_traffic::schedule::ItineraryVersion v = 0;
      for (const auto& route : alternative)
        database.extend(p.id(), {route}, v++);

      planner_actions.emplace_back(std::make_shared<PlannerAction>(
          std::move(database), config, start_B, goal_B, sync_failure));

      const double estimate = planner_actions.back()->cost_estimate();
      if (estimate < best_estimate.cost)
      {
        best_estimate.cost = estimate;
        best_estimate.estimator = planner_actions.back().get();
      }
    }
    second_best_estimate = best_estimate;

    //  const double estimate_leeway = 1.0; // 3.94903s | 3.51954s | 3.79843s
    const double estimate_leeway = 1.01; // 3.51758s | 3.49792s | 3.23114s
    //  const double estimate_leeway = 1.05; // 3.91029s | 3.53983s | 3.93007s
    //  const double estimate_leeway = 1.1; // 3.77644s | 3.4398s | 3.90458s
    //  const double estimate_leeway = 1.25; // 6.87893s | 4.04424s | 4.04946s
    //  const double estimate_leeway = 1.5; // 9.25227s | 6.06916s | 5.59134s
    //  const double estimate_leeway = 1.75; // 7.22795s | 10.3928s | 8.25479s

    for (auto& job : planner_actions)
      job->set_maximum_cost_estimate(estimate_leeway*best_estimate.cost);

    //^^^^^^^^^^^^^^^^^^^^ Done setting up the problem ^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //============================================================================


    //============================================================================
    //vvvvvvvvvvvvvvvvvvvvv TODO: Make this parallel vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    std::cout << "Jobs: " << planner_actions.size() << std::endl;

    auto meta_job = rmf_rxcpp::make_job_from_action_list(planner_actions);
    meta_job.subscribe(
      [this, s, estimate_leeway](
        const PlannerAction::Progress& progress)
      {
        auto& result = progress.result;

        if (!result.cost_estimate())
        {
          // The plan is impossible, so we should just move on
          finished_count++;
          std::cout << "(Fail) Finished: " << finished_count << std::endl;
          if (finished_count >= planner_actions.size())
          {
            s.on_next(best_result->plan.get_itinerary());
            s.on_completed();
            return;
          }
          progress.action.discard();
          return;
        }

        if (result) // This evaluates to true if a plan is ready
        {
          finished_count++;
          std::cout << "(Success) Finished: " << finished_count << std::endl;
          const auto finish_time =
          *result->get_itinerary().back().trajectory().finish_time();

          const double cost =
          rmf_traffic::time::to_seconds(finish_time - start_time);

          if (!best_result || cost < best_result->cost)
          {
            std::cout << "New best: " << cost << std::endl;
            // If no result exists yet, use this as the best result.
            best_result = JobResult{cost, *result};
            best_estimate.cost = cost;
          }

          if (finished_count >= planner_actions.size())
          {
            s.on_next(best_result->plan.get_itinerary());
            s.on_completed();
            return;
          }
        }
        else if (!best_result || *result.cost_estimate() < best_result->cost)
        {
          // This job could still produce a better plan than the current best, so
          // put it back in the job queue.

          // Update our best estimates based on the result of this job.
          const double cost_estimate = *result.cost_estimate();
          if (!best_result)
          {
            if (&progress.action == best_estimate.estimator)
            {
              best_estimate.cost = cost_estimate;
              if (second_best_estimate.cost < best_estimate.cost)
              {
                best_estimate = second_best_estimate;
                second_best_estimate = EstimateInfo();
              }
            }
            else
            {
              if (cost_estimate < second_best_estimate.cost)
              {
                second_best_estimate.cost = cost_estimate;
                second_best_estimate.estimator = &progress.action;
              }
            }
          }

          std::cout << "(Resume) " << "current cost: " << cost_estimate
                    << " best estimate: " << best_estimate.cost << " best result: ";
          if (best_result)
            std::cout << best_result->cost;
          else
            std::cout << "N/A";
          std::cout << std::endl;

          progress.action.set_maximum_cost_estimate(
            estimate_leeway*best_estimate.cost);
        }
        else
        {
          finished_count++;
          std::cout << "(Discarded) Finished: " << finished_count << std::endl;
          progress.action.discard();
          if (finished_count >= planner_actions.size())
          {
            s.on_next(best_result->plan.get_itinerary());
            s.on_completed();
            return;
          }
        }
        std::cout << "//////////" << std::endl;
      });
  }
};

//==============================================================================
int main()
{
  const auto benchmark_start = std::chrono::steady_clock::now();

  const auto meta_planner_obj = std::make_shared<MetaPlannerAction>();
  auto meta_planner_job = rmf_rxcpp::make_job<MetaPlannerAction::Result>(
    meta_planner_obj);

  std::promise<std::vector<rmf_traffic::Route>> itinerary_promise;
  auto itinerary_future = itinerary_promise.get_future();
  meta_planner_job
  .subscribe([&itinerary_promise](const auto& itinerary)
    {
      std::cout <<"\nBest plan for B:" << std::endl;
      itinerary_promise.set_value(itinerary);
    });

  itinerary_future.wait_for(std::chrono::seconds(5));
  print_itinerary(itinerary_future.get());

  const auto benchmark_finish = std::chrono::steady_clock::now();
  std::cout << "Benchmark: " << rmf_traffic::time::to_seconds(
      benchmark_finish - benchmark_start) << std::endl;

  std::cout << "Sync failures: " << meta_planner_obj->sync_failure << std::endl;
}
