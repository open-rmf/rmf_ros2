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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__UTILS_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__UTILS_HPP

#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/publisher.hpp>

#include <rmf_zone_msgs/msg/zone_request.hpp>
#include <rmf_zone_msgs/msg/zone_state.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include <memory>
#include <optional>
#include <string>

namespace rmf_fleet_adapter {

namespace agv {
class RobotContext;
} // namespace agv

namespace phases {

bool is_newer(
  const builtin_interfaces::msg::Time& a,
  const builtin_interfaces::msg::Time& b);

// Generate a zone request id of the form
// {fleet}_{robot}_{zone}_{random_hex}.
std::string generate_zone_request_id(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone);

// Build a PREBOOKING request asking the zone manager to assign this robot a
// vertex inside a zone, before it sets off.
rmf_zone_msgs::msg::ZoneRequest make_zone_prebooking_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone,
  std::string request_id,
  rmf_zone_msgs::msg::ZoneModifiers modifiers =
    rmf_zone_msgs::msg::ZoneModifiers());

// Build an ENTRY request asking the zone manager to confirm the booking at
// the zone boundary, taking a better vertex if one has freed up.
//
// entry_context says what the fleet adapter knows about why the robot is
// here. The zone manager decides what it implies.
rmf_zone_msgs::msg::ZoneRequest make_zone_entry_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone,
  std::string request_id,
  rmf_zone_msgs::msg::ZoneEntryContext entry_context,
  rmf_zone_msgs::msg::ZoneModifiers modifiers =
    rmf_zone_msgs::msg::ZoneModifiers());

// Build the ARRIVED request that tells the zone manager this robot has
// crossed into the zone. Nothing answers it.
rmf_zone_msgs::msg::ZoneRequest make_zone_arrived_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone);

// Build the EXIT request that tells the zone manager a booking has been
// given up.
rmf_zone_msgs::msg::ZoneRequest make_zone_exit_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone);

// Build the HANDBACK request that gives a vertex back to the zone manager.
rmf_zone_msgs::msg::ZoneRequest make_zone_handback_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone,
  const std::string& released_waypoint);

// What a ZoneState message turned out to mean for this robot.
struct ZoneStateResult
{
  enum class Status
  {
    /// Nothing in the message concerned this request. Stay subscribed.
    NoMatch,

    /// The booking is seated on the RobotContext and its ticket adopted.
    /// Drive to `goal`.
    Granted,

    /// The zone manager named a vertex the navigation graph does not have.
    /// Nothing was seated. Terminal.
    UnknownWaypoint,

    /// The zone manager does not know this zone. Terminal, since retrying
    /// cannot help.
    UnknownZone,

    /// The zone is known but cannot serve anybody, and no amount of waiting
    /// will change that. Today that means a zone declared with no internal
    /// vertices. Terminal.
    ZoneUnusable,

    /// Refused for now, most likely a full zone. Stay subscribed and try
    /// again when the zone next changes.
    Deferred,

    /// The grant carried a ticket for a different vertex, so it was refused
    /// rather than adopted. Terminal.
    TicketMismatch,

    /// Let in without a waypoint. Nothing is booked or reserved, so carry on
    /// with the plan already in hand and do not reroute.
    Proceed
  };

  Status status = Status::NoMatch;

  /// Set only for Granted.
  std::optional<rmf_traffic::agv::Plan::Goal> goal;
  std::string waypoint_name;

  /// The zone manager's wording, for UnknownWaypoint / UnknownZone /
  /// Deferred.
  std::string reason;

  /// The vertex we stopped holding in order to take this one, if any. Its
  /// ticket was NOT released. The zone manager has been told to take it
  /// back.
  std::string handed_back_waypoint;
};

// Consume a ZoneState, and if it answers this robot's request, seat the
// booking and adopt its ticket.
ZoneStateResult handle_zone_state(
  const std::shared_ptr<agv::RobotContext>& context,
  const rmf_zone_msgs::msg::ZoneState& state,
  const std::string& zone_name,
  const std::string& request_id,
  const char* caller);

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__UTILS_HPP
