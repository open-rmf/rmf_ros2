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

#ifndef SRC__RMF_FLEET_ADAPTER__JOBS__ROLLOUT_HPP
#define SRC__RMF_FLEET_ADAPTER__JOBS__ROLLOUT_HPP

#include <rmf_traffic/agv/Rollout.hpp>

namespace rmf_fleet_adapter {
namespace jobs {

//==============================================================================
class Rollout
{
public:

  struct Result
  {
    std::vector<rmf_traffic::schedule::Itinerary> alternatives;
  };

  Rollout(
    rmf_traffic::agv::Planner::Result result,
    rmf_traffic::schedule::ParticipantId blocker,
    rmf_traffic::Duration span,
    rmf_utils::optional<std::size_t> max_rollouts = rmf_utils::nullopt);

  template<typename Subscriber, typename Worker>
  void operator()(const Subscriber& s, const Worker& w);

private:
  rmf_traffic::agv::Rollout _rollout;
  rmf_traffic::schedule::ParticipantId _blocker;
  rmf_traffic::Duration _span;
  rmf_utils::optional<std::size_t> _max_rollouts;
};

} // namespace jobs
} // namespace rmf_fleet_adapter

#include "detail/impl_Rollout.hpp"

#endif // SRC__RMF_FLEET_ADAPTER__JOBS__ROLLOUT_HPP
