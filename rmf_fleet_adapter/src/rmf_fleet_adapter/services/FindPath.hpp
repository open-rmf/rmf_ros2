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

#ifndef SRC__RMF_FLEET_ADAPTER__SERVICES__FINDPATH_HPP
#define SRC__RMF_FLEET_ADAPTER__SERVICES__FINDPATH_HPP

#include "../jobs/SearchForPath.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
/// Find a path that gets from the start to the goal. It might or might not
/// comply with the given schedule, depending on what is feasible.
class FindPath : public std::enable_shared_from_this<FindPath>
{
public:

  FindPath(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts,
    rmf_traffic::agv::Plan::Goal goal,
    std::shared_ptr<const rmf_traffic::schedule::Snapshot> schedule,
    rmf_traffic::schedule::ParticipantId participant_id,
    const std::shared_ptr<const rmf_traffic::Profile>& profile);

  using Result = rmf_traffic::agv::Plan::Result;

  template<typename Subscriber>
  void operator()(const Subscriber& s);

  void interrupt();

private:
  std::shared_ptr<jobs::SearchForPath> _search_job;
  rmf_rxcpp::subscription_guard _search_sub;
};

} // namespace services
} // namespace rmf_fleet_adapter

#include "detail/impl_FindPath.hpp"

#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__FINDPATH_HPP
