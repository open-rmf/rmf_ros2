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

#ifndef SRC__RMF_FLEET_ADAPTER__SERVICES__PROGRESSEVALUATOR_HPP
#define SRC__RMF_FLEET_ADAPTER__SERVICES__PROGRESSEVALUATOR_HPP

#include <rmf_traffic/agv/Planner.hpp>

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
struct ProgressEvaluator
{
  static constexpr double DefaultCompliantLeewayBase = 30.0;
  static constexpr double DefaultCompliantLeewayMultiplier = 2.0;
  static constexpr double DefaultEstimateLeeway = 1.5;
  static constexpr double DefaultMaxCostThreshold = 120.0;

  ProgressEvaluator(
    double compliant_leeway_base_ = DefaultCompliantLeewayBase,
    double compliant_leeway_multiplier_ = DefaultCompliantLeewayMultiplier,
    double estimate_leeway_ = DefaultEstimateLeeway,
    double max_cost_threshold_ = DefaultMaxCostThreshold);

  using Result = rmf_traffic::agv::Plan::Result;

  struct Info
  {
    double cost = std::numeric_limits<double>::infinity();
    const Result* progress = nullptr;
  };

  bool initialize(const Result& setup);

  bool evaluate(Result& progress);

  void discard(Result& progress);

  Info best_estimate;
  Info second_best_estimate;
  Info best_result;
  Info best_discarded;
  std::size_t finished_count = 0;

  double compliant_leeway_base;

  // TODO(MXG): This is redundant with SearchForPath::compliant_leeway, so we
  // should refactor this.
  double compliant_leeway_multiplier;

  double estimate_leeway;

  double max_cost_threshold;
};


} // namespace services
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__PROGRESSEVALUATOR_HPP
