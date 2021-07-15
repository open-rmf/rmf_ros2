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

#include "FindPath.hpp"

namespace rmf_fleet_adapter {
namespace services {

//==============================================================================
FindPath::FindPath(
  std::shared_ptr<const rmf_traffic::agv::Planner> planner,
  rmf_traffic::agv::Plan::StartSet starts,
  rmf_traffic::agv::Plan::Goal goal,
  std::shared_ptr<const rmf_traffic::schedule::Snapshot> schedule,
  rmf_traffic::schedule::ParticipantId participant_id,
  const std::shared_ptr<const rmf_traffic::Profile>& profile)
{
  _search_job = std::make_shared<jobs::SearchForPath>(
    std::move(planner),
    std::move(starts),
    std::move(goal),
    std::move(schedule),
    participant_id,
    profile);
}

//==============================================================================
void FindPath::interrupt()
{
  _search_job->interrupt();
}

} // namespace services
} // namespace rmf_fleet_adapter
