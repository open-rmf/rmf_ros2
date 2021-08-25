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

#ifndef SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__PLANNING_HPP
#define SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__PLANNING_HPP

#include "../FindPath.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
template<typename Subscriber>
void FindPath::operator()(const Subscriber& s)
{
  _search_sub = rmf_rxcpp::make_job<jobs::SearchForPath::Result>(_search_job)
    .observe_on(rxcpp::observe_on_event_loop())
    .subscribe(
    [s](const jobs::SearchForPath::Result& result)
    {
      // The first time we get a result back, it will be when the jobs is
      // completed.
      if (result.compliant_job && result.compliant_job->progress().success())
      {
        s.on_next(result.compliant_job->progress());
        s.on_completed();
      }
      else if (result.greedy_job && result.greedy_job->progress().success())
      {
        s.on_next(result.greedy_job->progress());
        s.on_completed();
      }
      else
      {
        s.on_error(std::make_exception_ptr(
          std::runtime_error("[FindPath] Unable to find path")));
      }
    },
    [s](std::exception_ptr e)
    {
      s.on_error(e);
    },
    [s]()
    {
      // If this is triggered without a result coming in, that implies that the
      // job was impossible.
      s.on_completed();
    });
}

}
}

#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__PLANNING_HPP
