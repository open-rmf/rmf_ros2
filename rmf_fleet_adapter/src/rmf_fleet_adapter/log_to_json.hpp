/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__LOG_TO_JSON_HPP
#define SRC__RMF_FLEET_ADAPTER__LOG_TO_JSON_HPP

#include <rmf_task/Log.hpp>

#include <nlohmann/json.hpp>
#include <chrono>

namespace rmf_fleet_adapter {

//==============================================================================
inline std::chrono::milliseconds to_millis(std::chrono::nanoseconds duration)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
}

//==============================================================================
inline std::string tier_to_string(rmf_task::Log::Tier tier)
{
  using Tier = rmf_task::Log::Tier;
  switch (tier)
  {
    case Tier::Info:
      return "info";
    case Tier::Warning:
      return "warning";
    case Tier::Error:
      return "error";
    default:
      return "uninitialized";
  }
}

//==============================================================================
inline nlohmann::json log_to_json(const rmf_task::Log::Entry& entry)
{
  nlohmann::json output;
  output["seq"] = entry.seq();
  output["tier"] = tier_to_string(entry.tier());
  output["unix_millis_time"] =
    to_millis(entry.time().time_since_epoch()).count();
  output["text"] = entry.text();

  return output;
}

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__LOG_TO_JSON_HPP
