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

#include "FindEmergencyPullover.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
FindEmergencyPullover::FindEmergencyPullover(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    std::shared_ptr<const rmf_traffic::schedule::Snapshot> schedule,
    rmf_traffic::schedule::ParticipantId participant_id)
  : _planner(std::move(planner)),
    _starts(std::move(starts)),
    _schedule(std::move(schedule)),
    _participant_id(participant_id)
{
  // Do nothing
}

//==============================================================================
bool FindEmergencyPullover::Evaluator::initialize(const Result& setup)
{
  if (!setup.cost_estimate())
    return false;

  const double cost = *setup.cost_estimate();
  if (cost < best_estimate.cost)
  {
    best_estimate = ProgressInfo{cost, &setup};
    second_best_estimate = best_estimate;
  }

  return true;
}

//==============================================================================
bool FindEmergencyPullover::Evaluator::evaluate(Result& progress)
{
  if (!progress.success() && !progress.cost_estimate())
    return false;

  const double cost = progress.success()?
        progress->get_cost() : *progress.cost_estimate();

  if (progress.success())
  {
    if (cost < best_result.cost)
      best_result = ProgressInfo{cost, &progress};
  }

  if (cost < second_best_estimate.cost)
    second_best_estimate = ProgressInfo{cost, &progress};

  if (best_estimate.progress == &progress)
  {
    best_estimate = second_best_estimate;
    second_best_estimate = ProgressInfo();
  }

  if (!progress.success())
  {
    if (!best_result.progress || cost < best_result.cost)
    {
      progress.options().maximum_cost_estimate(
            estimate_leeway * best_estimate.cost);
      return true;
    }

    ++finished_count;
    return false;
  }

  ++finished_count;
  return true;
}

} // namespace services
} // namespace rmf_fleet_adapter
