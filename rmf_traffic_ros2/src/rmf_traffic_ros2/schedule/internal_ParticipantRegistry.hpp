/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef SRC__RMF_TRAFFIC_ROS2__SCHEDULE__INTERNAL_PARTICIPANTREGISTRY_CPP
#define SRC__RMF_TRAFFIC_ROS2__SCHEDULE__INTERNAL_PARTICIPANTREGISTRY_CPP

#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {
namespace mock {

// This function is purely used for testing by the
// ChangedParticipantScheduleNode. It should never ever be used for any other
// purpose.
//
// This function returns false if there are no participants to modify. Otherwise
// it will modify the ID of the highest-ID participant and return true.
bool mockup_modify_last_participant_id(ParticipantRegistry& registry);

// This function is purely used for testing the
// ChangedParticipantScheduleNode. It should never be used for any other
// purpose.
//
// This function returns false if there are no participants to modify. Otherwise
// it will modify the description of the highest-ID participant and return true.
bool mockup_modify_last_participant_description(
  ParticipantRegistry& registry);

} // namespace mock
} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // SRC__RMF_TRAFFIC_ROS2__SCHEDULE__INTERNAL_PARTICIPANTREGISTRY_CPP
