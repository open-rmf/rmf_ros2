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
ProgressEvaluator::ProgressEvaluator(
  const double compliant_leeway_base_,
  const double compliant_leeway_multiplier_,
  const double estimate_leeway_,
  const double max_cost_threshold_)
: compliant_leeway_base(compliant_leeway_base_),
  compliant_leeway_multiplier(compliant_leeway_multiplier_),
  estimate_leeway(estimate_leeway_),
  max_cost_threshold(max_cost_threshold_)
{
  // Do nothing
}

//==============================================================================
bool ProgressEvaluator::initialize(std::shared_ptr<const Planning> setup)
{
  if (!best_estimate.planning)
  {
    // Ensure that we always have something for a best estimate
    best_estimate.planning = setup;
  }

  if (!setup->progress().cost_estimate())
  {
    return false;
  }

  const double cost = *setup->progress().cost_estimate();
  if (cost < best_estimate.cost)
  {
    best_estimate = Info{cost, setup};
  }

  return true;
}

//==============================================================================
bool ProgressEvaluator::evaluate(std::shared_ptr<Planning> planning)
{
  if (!planning->progress().success() && !planning->progress().cost_estimate())
  {
    ++finished_count;
    return false;
  }

  const double cost = planning->progress().success() ?
    planning->progress()->get_cost() : *planning->progress().cost_estimate();

  if (planning->progress().success())
  {
    if (cost < best_result.cost)
    {
      best_result = Info{cost, planning};
    }
  }

  if (cost < second_best_estimate.cost)
  {
    second_best_estimate = Info{cost, planning};
  }

  if (best_estimate.planning == planning)
  {
    best_estimate = second_best_estimate;
    second_best_estimate = Info();
  }

  if (planning->progress().saturated())
  {
    ++finished_count;
    return false;
  }

  if (planning->progress().disconnected())
  {
    ++finished_count;
    return false;
  }

  const double dropdead_cost =
    std::min(
    planning->progress().ideal_cost().value() + max_cost_threshold,
    compliant_leeway_multiplier* planning->progress().ideal_cost().value()
    + compliant_leeway_base);

  const bool giveup = dropdead_cost <= cost;
  if (!planning->progress().success() && !giveup)
  {
    if (!best_result.planning)
    {
      planning->progress().options().maximum_cost_estimate(
        std::min(estimate_leeway * cost, dropdead_cost));

      return true;
    }
    else if (cost < best_result.cost)
    {
      planning->progress().options().maximum_cost_estimate(
        std::min(best_result.cost, dropdead_cost));
      return true;
    }
  }

  ++finished_count;
  return false;
}

//==============================================================================
void ProgressEvaluator::discard(std::shared_ptr<Planning> planning)
{
  if (best_estimate.planning == planning)
  {
    best_estimate = second_best_estimate;
    second_best_estimate = Info();
  }

  const double cost = planning->progress().cost_estimate() ?
    *planning->progress().cost_estimate() : std::numeric_limits<double>::infinity();
  if (best_discarded.planning || cost < best_discarded.cost)
  {
    best_discarded = Info{cost, planning};
  }

  ++finished_count;
}

} // namespace services
} // namespace rmf_fleet_adapter
