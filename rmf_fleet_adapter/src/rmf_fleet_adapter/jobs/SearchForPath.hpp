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

#ifndef SRC__RMF_FLEET_ADAPTER__JOBS__SEARCHFORPATH_HPP
#define SRC__RMF_FLEET_ADAPTER__JOBS__SEARCHFORPATH_HPP

#include "Planning.hpp"

namespace rmf_fleet_adapter {
namespace jobs {

//==============================================================================
/// A job to advance the search for a path from a specific start to a specific
/// goal. This will manage two planning efforts simultaneously: One that
/// complies with the schedule, and one that ignores the schedule. The compliant
/// plan will generally be preferred, but if it is excessively blocked by other
/// vehicles on the schedule, then the greedy plan can be used to create an
/// opening in the traffic schedule.
class SearchForPath : public std::enable_shared_from_this<SearchForPath>
{
public:

  SearchForPath(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    rmf_traffic::agv::Plan::Goal goal,
    std::shared_ptr<const rmf_traffic::schedule::Snapshot> schedule,
    rmf_traffic::schedule::ParticipantId participant_id,
    const std::shared_ptr<const rmf_traffic::Profile>& profile);

  enum class Type
  {
    greedy,
    compliant
  };

  struct Result
  {
    std::shared_ptr<Planning> greedy_job;
    std::shared_ptr<Planning> compliant_job;
    Type type;
  };

  template<typename Subscriber, typename Worker>
  void operator()(const Subscriber& s, const Worker& w);

  // Stop the compliant planner, and return the result of the greedy planner as
  // soon as it's ready.
  void interrupt();

  void set_cost_limit(double cost);

  Planning& greedy();
  const Planning& greedy() const;

  Planning& compliant();
  const Planning& compliant() const;

private:
  std::shared_ptr<const rmf_traffic::agv::Planner> _planner;
  rmf_traffic::agv::Plan::StartSet _starts;
  rmf_traffic::agv::Plan::Goal _goal;
  std::shared_ptr<const rmf_traffic::schedule::Snapshot> _schedule;
  rmf_traffic::schedule::ParticipantId _participant_id;
  std::shared_ptr<std::atomic_bool> _interrupt_flag =
    std::make_shared<std::atomic_bool>(false);

  // The greedy job makes an optimal plan that ignores all other schedule
  // participants. It is used for two purposes:
  // 1. Provide a reference for what an acceptable cost is for the compliant job
  // 2. Provide a backup plan if a compliant job can't be found
  std::shared_ptr<Planning> _greedy_job;
  rmf_rxcpp::subscription_guard _greedy_sub;
  bool _greedy_finished = false;

  // The compliant job makes the plan which is optimal without conflicting with
  // any other traffic currently on the schedule. In some cases, it might not
  // be feasible to find an acceptable compliant job, either because
  std::shared_ptr<Planning> _compliant_job;
  rmf_rxcpp::subscription_guard _compliant_sub;
  bool _compliant_finished = false;

  rmf_utils::optional<double> _explicit_cost_limit;

  rxcpp::schedulers::worker _worker;

  // TODO(MXG): Make these leeway factors configurable
  const double _greedy_leeway = 10.0;
  const double _compliant_leeway = 3.0;
};

} // namespace jobs
} // namespace rmf_fleet_adapter

#include "detail/impl_SearchForPath.hpp"

#endif // SRC__RMF_FLEET_ADAPTER__JOBS__SEARCHFORPATH_HPP
