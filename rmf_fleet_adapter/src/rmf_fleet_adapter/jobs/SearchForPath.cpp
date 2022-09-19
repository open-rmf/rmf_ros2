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
std::function<bool()> make_interrupter(
  std::shared_ptr<std::atomic_bool> interrupt_flag,
  std::optional<rmf_traffic::Time> deadline)
{
  const auto counter = std::make_shared<uint32_t>(0);
  return [interrupt_flag, deadline, counter]()
    {
      ++*counter;
      if (*counter > 20)
      {
        *counter = 0;
        // Only check these once in a while to reduce planning overhead
        if (*interrupt_flag)
          return true;

        const auto now = std::chrono::steady_clock::now();
        if (deadline <= now)
          return true;
      }

      return false;
    };
}

//==============================================================================
SearchForPath::SearchForPath(
  std::shared_ptr<const rmf_traffic::agv::Planner> planner,
  rmf_traffic::agv::Plan::StartSet starts,
  rmf_traffic::agv::Plan::Goal goal,
  std::shared_ptr<const rmf_traffic::schedule::Snapshot> schedule,
  rmf_traffic::schedule::ParticipantId participant_id,
  const std::shared_ptr<const rmf_traffic::Profile>& profile,
  std::optional<rmf_traffic::Duration> planning_time_limit)
: _planner(std::move(planner)),
  _starts(std::move(starts)),
  _goal(std::move(goal)),
  _schedule(std::move(schedule)),
  _participant_id(participant_id),
  _worker(rxcpp::schedulers::make_event_loop().create_worker())
{
  if (planning_time_limit.has_value())
  {
    const auto start_time = std::chrono::steady_clock::now();
    _deadline = start_time + *planning_time_limit;
  }
  auto greedy_options = _planner->get_default_options();
  greedy_options.validator(nullptr);
  greedy_options.interrupter(make_interrupter(_interrupt_flag, _deadline));

  // TODO(MXG): This is a gross hack to side-step the saturation issue that
  // happens when too many start conditions are given. That problem should be
  // fixed after we've improved the planner's heuristic.
  auto greedy_starts = _starts;
  if (greedy_starts.size() > 1)
    greedy_starts.erase(greedy_starts.begin()+1, greedy_starts.end());

  auto greedy_setup = _planner->setup(greedy_starts, _goal, greedy_options);
  const double base_cost = *greedy_setup.cost_estimate();
  greedy_setup.options().maximum_cost_estimate(_greedy_leeway*base_cost);

  auto compliant_options = _planner->get_default_options();
  compliant_options.validator(
    rmf_traffic::agv::ScheduleRouteValidator::make(
      _schedule, _participant_id, *profile));
  compliant_options.maximum_cost_estimate(_compliant_leeway*base_cost);
  greedy_options.interrupter(make_interrupter(_interrupt_flag, _deadline));
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
