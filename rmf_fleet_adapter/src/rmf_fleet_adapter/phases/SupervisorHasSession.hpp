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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__SUPERVISORHASSESSION_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__SUPERVISORHASSESSION_HPP

#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>

namespace rmf_fleet_adapter {
namespace phases {

bool supervisor_has_session(
  const rmf_door_msgs::msg::SupervisorHeartbeat& heartbeat,
  const std::string& session_id,
  const std::string& door_name);

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__SUPERVISORHASSESSION_HPP
