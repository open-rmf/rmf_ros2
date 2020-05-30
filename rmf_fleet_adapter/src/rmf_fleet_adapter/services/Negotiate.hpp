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

  Negotiate(
      std::shared_ptr<const rmf_traffic::agv::Planner> planner,
      rmf_traffic::agv::Plan::StartSet starts,
      std::vector<rmf_traffic::agv::Plan::Goal> goals,
      rmf_traffic::schedule::Negotiator::TableViewerPtr viewer,
      std::shared_ptr<rmf_traffic::schedule::Negotiator::Responder> responder,
      rmf_traffic::schedule::Negotiator::Responder::ApprovalCallback approval);

  static std::shared_ptr<Negotiate> path(
      std::shared_ptr<const rmf_traffic::agv::Planner> planner,
      rmf_traffic::agv::Plan::StartSet starts,
      rmf_traffic::agv::Plan::Goal goal,
      rmf_traffic::schedule::Negotiator::TableViewerPtr viewer,
      std::shared_ptr<rmf_traffic::schedule::Negotiator::Responder> responder,
      rmf_traffic::schedule::Negotiator::Responder::ApprovalCallback approval);

  static std::shared_ptr<Negotiate> emergency_pullover(
      std::shared_ptr<const rmf_traffic::agv::Planner> planner,
      rmf_traffic::agv::Plan::StartSet starts,
      rmf_traffic::schedule::Negotiation::Table::ViewerPtr viewer,
      std::shared_ptr<rmf_traffic::schedule::Negotiator::Responder> responder,
      rmf_traffic::schedule::Negotiator::Responder::ApprovalCallback approval);

  using Result = std::function<void()>;

  template<typename Subscriber>
  void operator()(const Subscriber& s);

  void interrupt();

  void discard();

private:

  std::shared_ptr<const rmf_traffic::agv::Planner> _planner;
  rmf_traffic::agv::Plan::StartSet _starts;
  std::vector<rmf_traffic::agv::Plan::Goal> _goals;
  rmf_traffic::schedule::Negotiator::TableViewerPtr _viewer;
  std::shared_ptr<rmf_traffic::schedule::Negotiator::Responder> _responder;
  rmf_traffic::schedule::Negotiator::Responder::ApprovalCallback _approval;

  std::vector<std::shared_ptr<jobs::Planning>> _search_jobs;
  rxcpp::subscription _search_sub;
  std::shared_ptr<jobs::Rollout> _rollout_job;
  rxcpp::subscription _rollout_sub;
  bool _attempting_rollout = false;

  using Alternatives = std::vector<rmf_traffic::schedule::Itinerary>;
  rmf_utils::optional<Alternatives> _alternatives;

  bool _interrupted = false;
  bool _discarded = false;

  ProgressEvaluator _evaluator;
};

} // namespace services
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__NEGOTIATEPATH_HPP
