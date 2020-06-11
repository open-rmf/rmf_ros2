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

#ifndef SRC__RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATIONROOM_HPP
#define SRC__RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATIONROOM_HPP

#include <rmf_traffic/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/Negotiator.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_proposal.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_rejection.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_forfeit.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_key.hpp>

#include <list>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::Negotiation::VersionedKeySequence convert(
  const std::vector<rmf_traffic_msgs::msg::ScheduleConflictKey>& from);

//==============================================================================
std::vector<rmf_traffic_msgs::msg::ScheduleConflictKey> convert(
  const rmf_traffic::schedule::Negotiation::VersionedKeySequence& from);

namespace schedule {

using ParticipantId = rmf_traffic::schedule::ParticipantId;
using NegotiatorPtr = std::unique_ptr<rmf_traffic::schedule::Negotiator>;
using NegotiatorMap = std::unordered_map<ParticipantId, NegotiatorPtr>;

//==============================================================================
// TODO(MXG): Refactor this class into something more broadly usable.
struct NegotiationRoom
{
  NegotiationRoom(rmf_traffic::schedule::Negotiation negotiation_);

  rmf_traffic::schedule::Negotiation negotiation;
  std::list<rmf_traffic_msgs::msg::ScheduleConflictProposal> cached_proposals;
  std::list<rmf_traffic_msgs::msg::ScheduleConflictRejection> cached_rejections;
  std::list<rmf_traffic_msgs::msg::ScheduleConflictForfeit> cached_forfeits;

  std::vector<rmf_traffic::schedule::Negotiation::TablePtr> check_cache(
    const NegotiatorMap& negotiators);
};

//==============================================================================
// TODO(MXG): Refactor this by putting it somewhere more meaningful.
void print_negotiation_status(
  rmf_traffic::schedule::Version conflict_version,
  const rmf_traffic::schedule::Negotiation& negotiation);

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // SRC__RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATIONROOM_HPP
