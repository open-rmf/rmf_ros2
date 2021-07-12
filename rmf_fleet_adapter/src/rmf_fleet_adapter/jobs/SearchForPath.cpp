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

#include "SearchForPath.hpp"

namespace rmf_fleet_adapter {
namespace jobs {

//==============================================================================
SearchForPath::SearchForPath(
  std::shared_ptr<const rmf_traffic::agv::Planner> planner,
  rmf_traffic::agv::Plan::StartSet starts,
  rmf_traffic::agv::Plan::Goal goal,
  std::shared_ptr<const rmf_traffic::schedule::Snapshot> schedule,
  rmf_traffic::schedule::ParticipantId participant_id,
  const std::shared_ptr<const rmf_traffic::Profile>& profile)
: _planner(std::move(planner)),
  _starts(std::move(starts)),
  _goal(std::move(goal)),
  _schedule(std::move(schedule)),
  _participant_id(participant_id),
  _worker(rxcpp::schedulers::make_event_loop().create_worker())
{
  auto greedy_options = _planner->get_default_options();
  greedy_options.validator(nullptr);

  // TODO(MXG): This is a gross hack to side-step the saturation issue that
  // happens when too many start conditions are given. That problem should be
  // fixed after we've improved the planner's heuristic.
  auto greedy_starts = _starts;
  if (greedy_starts.size() > 1)
    greedy_starts.erase(greedy_starts.begin()+1, greedy_starts.end());

  auto greedy_setup = _planner->setup(greedy_starts, _goal, greedy_options);
  if (!greedy_setup.cost_estimate())
  {
    // If this ever happens, then there is a serious bug.
    const auto& desc = _schedule->get_participant(_participant_id);
    std::string name = desc ?
      desc->name() + "] owned by [" + desc->owner() :
      std::to_string(_participant_id);

    std::cerr << "[SearchForPath] CRITICAL ERROR: Impossible plan requested! "
              << "Participant [" << name << "] Requested path";

    for (const auto& start : _starts)
      std::cerr << " (" << start.waypoint() << ")";
    std::cerr << " --> (" << _goal.waypoint() << ")" << std::endl;

    assert(false);
    return;
  }

  const double base_cost = *greedy_setup.cost_estimate();
  greedy_setup.options().maximum_cost_estimate(_greedy_leeway*base_cost);

  auto compliant_options = _planner->get_default_options();
  compliant_options.validator(
    rmf_traffic::agv::ScheduleRouteValidator::make(
      _schedule, _participant_id, *profile));
  compliant_options.maximum_cost_estimate(_compliant_leeway*base_cost);
  compliant_options.interrupt_flag(_interrupt_flag);
  auto compliant_setup = _planner->setup(_starts, _goal, compliant_options);

  _greedy_job = std::make_shared<Planning>(std::move(greedy_setup));
  _compliant_job = std::make_shared<Planning>(std::move(compliant_setup));
}

//==============================================================================
void SearchForPath::interrupt()
{
  *_interrupt_flag = true;
}

//==============================================================================
Planning& SearchForPath::greedy()
{
  return *_greedy_job;
}

//==============================================================================
const Planning& SearchForPath::greedy() const
{
  return *_greedy_job;
}

//==============================================================================
Planning& SearchForPath::compliant()
{
  return *_compliant_job;
}

//==============================================================================
const Planning& SearchForPath::compliant() const
{
  return *_compliant_job;
}

//==============================================================================
void SearchForPath::set_cost_limit(double cost)
{
  _explicit_cost_limit = cost;
}

} // namespace jobs
} // namespace rmf_fleet_adapter
