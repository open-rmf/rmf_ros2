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
  rmf_traffic::schedule::ParticipantId participant_id,
  std::shared_ptr<const rmf_traffic::Profile> profile)
: _planner(std::move(planner)),
  _starts(std::move(starts)),
  _schedule(std::move(schedule)),
  _participant_id(participant_id),
  _profile(std::move(profile))
{
  // Do nothing
}

//==============================================================================
void FindEmergencyPullover::interrupt()
{
  _interrupted = true;
  for (const auto& s : _search_jobs)
    s->interrupt();
}

} // namespace services
} // namespace rmf_fleet_adapter
