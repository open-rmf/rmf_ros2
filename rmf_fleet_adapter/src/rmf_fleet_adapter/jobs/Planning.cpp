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

#include "Planning.hpp"

namespace rmf_fleet_adapter {
namespace jobs {

//==============================================================================
Planning::Planning(
  std::shared_ptr<const rmf_traffic::agv::Planner> planner,
  const rmf_traffic::agv::Plan::StartSet& starts,
  rmf_traffic::agv::Plan::Goal goal,
  rmf_traffic::agv::Plan::Options options)
: _current_result(planner->setup(starts, std::move(goal), std::move(options)))
{
  _current_result->options().saturation_limit(10000);
}

//==============================================================================
Planning::Planning(rmf_traffic::agv::Planner::Result _setup)
: _current_result(std::move(_setup))
{
  _current_result->options().saturation_limit(10000);
}

//==============================================================================
void Planning::resume()
{
  _resume();
}

//==============================================================================
void Planning::discard()
{
  _current_result = rmf_utils::nullopt;
}

//==============================================================================
bool Planning::active() const
{
//  return _current_result.has_value();
  return static_cast<bool>(_current_result);
}

//==============================================================================
rmf_traffic::agv::Planner::Result& Planning::progress()
{
  return *_current_result;
}

//==============================================================================
const rmf_traffic::agv::Planner::Result& Planning::progress() const
{
  return *_current_result;
}

//==============================================================================
std::unique_lock<std::mutex> Planning::_lock_resume() const
{
  std::unique_lock<std::mutex> lock(_resume_mutex, std::defer_lock);
  while (!lock.try_lock())
  {
    // Intentionally busy wait to obtain the mutex as fast as possible
  }

  return lock;
}

} // namespace jobs
} // namespace rmf_fleet_adapter
