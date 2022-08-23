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

#ifndef SRC__RMF_FLEET_ADAPTER__SERVICES__NEGOTIATEPATH_HPP
#define SRC__RMF_FLEET_ADAPTER__SERVICES__NEGOTIATEPATH_HPP

#include <rmf_traffic/schedule/Negotiator.hpp>
#include "../jobs/Planning.hpp"
#include "../jobs/Rollout.hpp"
#include "ProgressEvaluator.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
class Negotiate : public std::enable_shared_from_this<Negotiate>
{
public:

  using ItineraryVersion = rmf_traffic::schedule::ItineraryVersion;
  using UpdateVersion = rmf_utils::optional<ItineraryVersion>;
  using ApprovalCallback =
    std::function<UpdateVersion(
        rmf_traffic::PlanId,
        const rmf_traffic::agv::Plan&,
        rmf_traffic::schedule::Itinerary)
    >;

  Negotiate(
    rmf_traffic::PlanId assigned_id,
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    std::vector<rmf_traffic::agv::Plan::Goal> goals,
    std::vector<rmf_traffic::agv::Plan::Goal> followed_by,
    rmf_traffic::schedule::Negotiator::TableViewerPtr viewer,
    rmf_traffic::schedule::Negotiator::ResponderPtr responder,
    ApprovalCallback approval,
    ProgressEvaluator evaluator,
    std::vector<rmf_traffic::Route> initial_itinerary = {});

  static std::shared_ptr<Negotiate> path(
    rmf_traffic::PlanId plan_id,
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    rmf_traffic::agv::Plan::Goal goal,
    std::vector<rmf_traffic::agv::Plan::Goal> followed_by,
    rmf_traffic::schedule::Negotiator::TableViewerPtr viewer,
    rmf_traffic::schedule::Negotiator::ResponderPtr responder,
    ApprovalCallback approval,
    ProgressEvaluator evaluator,
    std::vector<rmf_traffic::Route> initial_itinerary = {});

  static std::shared_ptr<Negotiate> emergency_pullover(
    rmf_traffic::PlanId plan_id,
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    rmf_traffic::schedule::Negotiation::Table::ViewerPtr viewer,
    rmf_traffic::schedule::Negotiator::ResponderPtr responder,
    ApprovalCallback approval,
    ProgressEvaluator evaluator);

  struct Result
  {
    std::shared_ptr<Negotiate> service;
    std::function<void()> respond;
  };

  template<typename Subscriber>
  void operator()(const Subscriber& s);

  void interrupt();

  void discard();

  bool discarded() const;

  const rmf_traffic::schedule::Negotiator::ResponderPtr& responder() const;

  ~Negotiate();

private:

  void _resume_next();

  rmf_traffic::PlanId _plan_id;
  std::shared_ptr<const rmf_traffic::agv::Planner> _planner;
  rmf_traffic::agv::Plan::StartSet _starts;
  std::vector<rmf_traffic::agv::Plan::Goal> _goals;
  std::vector<rmf_traffic::agv::Plan::Goal> _followed_by;
  rmf_traffic::schedule::Negotiator::TableViewerPtr _viewer;
  rmf_traffic::schedule::Negotiator::ResponderPtr _responder;
  ApprovalCallback _approval;
  std::vector<rmf_traffic::Route> _initial_itinerary;

  using JobPtr = std::shared_ptr<jobs::Planning>;
  struct CompareJobs
  {
    bool operator()(const JobPtr& a, const JobPtr& b)
    {
      if (!b->progress().cost_estimate())
        return true;

      if (!a->progress().cost_estimate())
        return false;

      return *b->progress().cost_estimate() < *a->progress().cost_estimate();
    }
  };

  std::unordered_set<JobPtr> _current_jobs;
  std::priority_queue<JobPtr, std::vector<JobPtr>, CompareJobs> _resume_jobs;
  std::vector<JobPtr> _queued_jobs; // Used to keep the jobs alive
  JobPtr _best_job;
  rmf_rxcpp::subscription_guard _search_sub;
  std::shared_ptr<jobs::Rollout> _rollout_job;
  rmf_rxcpp::subscription_guard _rollout_sub;
  bool _finished = false;
  bool _attempting_rollout = false;

  using Alternatives = std::vector<rmf_traffic::schedule::Itinerary>;
  rmf_utils::optional<Alternatives> _alternatives;
  std::unordered_set<rmf_traffic::schedule::ParticipantId> _blockers;

  std::shared_ptr<std::atomic_bool> _interrupted =
    std::make_shared<std::atomic_bool>(false);
  bool _discarded = false;

  static constexpr std::size_t max_concurrent_jobs = 5;

  ProgressEvaluator _evaluator;
};

} // namespace services
} // namespace rmf_fleet_adapter

#include "detail/impl_Negotiate.hpp"

#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__NEGOTIATEPATH_HPP
