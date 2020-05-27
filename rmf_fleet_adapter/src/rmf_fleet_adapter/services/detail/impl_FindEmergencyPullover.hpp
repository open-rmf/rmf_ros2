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

#ifndef SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_FINDEMERGENCYPULLOVER_HPP
#define SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_FINDEMERGENCYPULLOVER_HPP

#include "../FindEmergencyPullover.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
template<typename Subscriber>
void FindEmergencyPullover::operator()(const Subscriber& s)
{
  const auto& graph = _planner->get_configuration().graph();
  const std::size_t N = graph.num_waypoints();
  _search_jobs.reserve(N);
  for (std::size_t i=0; i < N; ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    if (wp.is_parking_spot())
    {
      auto search = std::make_shared<jobs::SearchForPath>(
            _planner, _starts, wp.index(), _schedule, _participant_id);

      const double cost = search->current_estimate();
      if (cost < _best_estimate.cost)
        _best_estimate = {cost, search.get()};

      _search_jobs.emplace_back(std::move(search));
    }
  }
  _second_best_estimate = _best_estimate;

  const double estimate_leeway = 1.01;

  _search_sub = rmf_rxcpp::make_job_from_action_list(_search_jobs)
      .subscribe(
        [this, s, estimate_leeway](
          const jobs::SearchForPath::Result& progress)
  {

  });
}

} // namespace services
} // namespace rmf_fleet_adapter


#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_FINDEMERGENCYPULLOVER_HPP
