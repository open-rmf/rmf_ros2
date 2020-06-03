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

#include "ProgressEvaluator.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
bool ProgressEvaluator::initialize(const Result& setup)
{
  if (!setup.cost_estimate())
    return false;

  const double cost = *setup.cost_estimate();
  if (cost < best_estimate.cost)
    best_estimate = Info{cost, &setup};

  return true;
}

//==============================================================================
bool ProgressEvaluator::evaluate(Result& progress)
{
  if (!progress.success() && !progress.cost_estimate())
  {
    ++finished_count;
    return false;
  }

  const double cost = progress.success()?
        progress->get_cost() : *progress.cost_estimate();

  if (progress.success())
  {
    if (cost < best_result.cost)
      best_result = Info{cost, &progress};
  }

  if (cost < second_best_estimate.cost)
    second_best_estimate = Info{cost, &progress};

  if (best_estimate.progress == &progress)
  {
    best_estimate = second_best_estimate;
    second_best_estimate = Info();
  }

  const bool giveup = compliant_leeway*progress.initial_cost_estimate() < cost;
  if (!progress.success() && !giveup)
  {
    if (!best_result.progress)
    {
      progress.options().maximum_cost_estimate(estimate_leeway * cost);
      return true;
    }
    else if (cost < best_result.cost)
    {
      progress.options().maximum_cost_estimate(best_result.cost);
      return true;
    }
  }

  ++finished_count;
  return false;
}

//==============================================================================
void ProgressEvaluator::discard(Result& progress)
{
  if (best_estimate.progress == &progress)
  {
    best_estimate = second_best_estimate;
    second_best_estimate = Info();
  }

  const double cost = progress.cost_estimate()?
        *progress.cost_estimate() : std::numeric_limits<double>::infinity();
  if (best_discarded.progress || cost < best_discarded.cost)
    best_discarded = Info{cost, &progress};

  ++finished_count;
}

} // namespace services
} // namespace rmf_fleet_adapter
