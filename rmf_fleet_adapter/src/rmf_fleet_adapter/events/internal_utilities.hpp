/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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
#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_UTILITIES_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_UTILITIES_HPP

#include "../agv/RobotContext.hpp"

namespace rmf_fleet_adapter {
namespace events {

/// Get waypoint namegiven goal
std::string wp_name(
  const agv::RobotContext& context,
  const rmf_traffic::agv::Plan::Goal& goal);

///Get robot's current waypoint name
std::string wp_name(const agv::RobotContext& context);
}
}

#endif
