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

#include "Utils.hpp"

#include "../agv/RobotContext.hpp"

#include <random>
#include <sstream>

namespace rmf_fleet_adapter {
namespace phases {

bool is_newer(const builtin_interfaces::msg::Time& a,
  const builtin_interfaces::msg::Time& b)
{
  return a.sec > b.sec || (a.sec == b.sec && a.nanosec >= b.nanosec);
}

namespace {
// Produces a length*2 character zero-padded hex string from random bytes.
std::string generate_random_hex_string(const std::size_t length)
{
  static thread_local std::mt19937 gen{std::random_device{}()};
  static thread_local std::uniform_int_distribution<int> dis(0, 255);
  std::stringstream ss;
  for (std::size_t i = 0; i < length; ++i)
  {
    const auto random_char = dis(gen);
    std::stringstream hexstream;
    hexstream << std::hex << random_char;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}
} // anonymous namespace

std::string generate_zone_request_id(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone)
{
  return fleet + "_" + robot + "_" + zone + "_"
    + generate_random_hex_string(5);
}

namespace {
//==============================================================================
rmf_zone_msgs::msg::ZoneRequest make_zone_claim_request(
  const uint8_t request_type,
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone,
  std::string request_id,
  rmf_zone_msgs::msg::ZoneEntryContext entry_context,
  rmf_zone_msgs::msg::ZoneModifiers modifiers)
{
  auto request = rmf_zone_msgs::msg::ZoneRequest();
  request.robot_name = robot;
  request.fleet_name = fleet;
  request.request_id = std::move(request_id);
  request.zone_name = zone;
  request.request_type = request_type;
  request.entry_context = std::move(entry_context);
  request.modifiers = std::move(modifiers);
  return request;
}
} // anonymous namespace

//==============================================================================
rmf_zone_msgs::msg::ZoneRequest make_zone_prebooking_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone,
  std::string request_id,
  rmf_zone_msgs::msg::ZoneModifiers modifiers)
{
  // Only a zone task ever prebooks. The zone is its destination by
  // definition, and it is sent before there is a plan to cross anything.
  auto context = rmf_zone_msgs::msg::ZoneEntryContext();
  context.task_type = rmf_zone_msgs::msg::ZoneEntryContext::TASK_ZONE;
  context.zone_relation =
    rmf_zone_msgs::msg::ZoneEntryContext::RELATION_DESTINATION;

  return make_zone_claim_request(
    rmf_zone_msgs::msg::ZoneRequest::PREBOOKING,
    fleet, robot, zone, std::move(request_id), std::move(context),
    std::move(modifiers));
}

//==============================================================================
rmf_zone_msgs::msg::ZoneRequest make_zone_entry_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone,
  std::string request_id,
  rmf_zone_msgs::msg::ZoneEntryContext entry_context,
  rmf_zone_msgs::msg::ZoneModifiers modifiers)
{
  return make_zone_claim_request(
    rmf_zone_msgs::msg::ZoneRequest::ENTRY,
    fleet, robot, zone, std::move(request_id), std::move(entry_context),
    std::move(modifiers));
}

//==============================================================================
rmf_zone_msgs::msg::ZoneRequest make_zone_arrived_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone)
{
  auto request = rmf_zone_msgs::msg::ZoneRequest();
  request.robot_name = robot;
  request.fleet_name = fleet;
  request.request_id = generate_zone_request_id(fleet, robot, zone);
  request.zone_name = zone;
  request.request_type = rmf_zone_msgs::msg::ZoneRequest::ARRIVED;
  return request;
}

//==============================================================================
rmf_zone_msgs::msg::ZoneRequest make_zone_exit_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone)
{
  auto request = rmf_zone_msgs::msg::ZoneRequest();
  request.robot_name = robot;
  request.fleet_name = fleet;
  request.request_id = generate_zone_request_id(fleet, robot, zone);
  request.zone_name = zone;
  request.request_type = rmf_zone_msgs::msg::ZoneRequest::EXIT;
  return request;
}

//==============================================================================
rmf_zone_msgs::msg::ZoneRequest make_zone_handback_request(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone,
  const std::string& released_waypoint)
{
  auto request = rmf_zone_msgs::msg::ZoneRequest();
  request.robot_name = robot;
  request.fleet_name = fleet;
  request.request_id = generate_zone_request_id(fleet, robot, zone);
  request.zone_name = zone;
  request.request_type = rmf_zone_msgs::msg::ZoneRequest::HANDBACK;
  request.released_waypoint = released_waypoint;
  return request;
}

//==============================================================================
ZoneStateResult handle_zone_state(
  const std::shared_ptr<agv::RobotContext>& context,
  const rmf_zone_msgs::msg::ZoneState& state,
  const std::string& zone_name,
  const std::string& request_id,
  const char* caller)
{
  ZoneStateResult result;

  const auto& robot_name = context->name();
  const auto& fleet_name = context->group();
  const auto node = context->node();

  for (const auto& booking : state.bookings)
  {
    if (booking.robot_name != robot_name
      || booking.fleet_name != fleet_name
      || booking.zone_name != zone_name
      || booking.request_id != request_id)
      continue;

    const auto& graph = context->navigation_graph();
    const auto* wp = graph.find_waypoint(booking.assigned_waypoint_name);
    if (!wp)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "%s: manager assigned waypoint [%s] to [%s], which is not in the "
        "navigation graph",
        caller,
        booking.assigned_waypoint_name.c_str(),
        context->requester_id().c_str());

      result.status = ZoneStateResult::Status::UnknownWaypoint;
      result.waypoint_name = booking.assigned_waypoint_name;
      result.reason = "assigned_waypoint_not_in_graph";
      return result;
    }

    auto goal = rmf_traffic::agv::Plan::Goal(wp->index());
    if (booking.has_orientation)
      goal = rmf_traffic::agv::Plan::Goal(wp->index(), booking.orientation);

    RCLCPP_INFO(
      node->get_logger(),
      "%s: [%s] booked waypoint [%s] in zone [%s]",
      caller,
      context->requester_id().c_str(),
      booking.assigned_waypoint_name.c_str(),
      zone_name.c_str());

    // The zone and the vertex we are giving up, for the manager to take
    // back. Empty when there is nothing to hand back.
    std::optional<std::pair<std::string, std::string>> hand_back;
    const auto reserved = context->_get_reserved_location();
    if (booking.has_ticket
      && !reserved.empty() && reserved != booking.assigned_waypoint_name
      && context->_zone_manager_listening())
    {
      const auto zone = context->zone_holding_waypoint(reserved);
      if (zone)
        hand_back = std::make_pair(*zone, reserved);
    }

    context->set_zone_booking(
      zone_name, booking.assigned_waypoint_name, goal);

    if (booking.has_ticket)
    {
      if (booking.ticket_resource != booking.assigned_waypoint_name)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "%s: manager granted [%s] to [%s] but backed it with a ticket "
          "holding [%s], so it will not be adopted",
          caller,
          booking.assigned_waypoint_name.c_str(),
          context->requester_id().c_str(),
          booking.ticket_resource.c_str());

        result.status = ZoneStateResult::Status::TicketMismatch;
        result.waypoint_name = booking.assigned_waypoint_name;
        result.reason = "ticket_resource_mismatch";
        return result;
      }

      context->_adopt_zone_ticket(
        booking.ticket_id, booking.ticket_resource, !hand_back.has_value());

      if (hand_back)
      {
        node->zone_request()->publish(
          make_zone_handback_request(
            fleet_name, robot_name, hand_back->first, hand_back->second));

        result.handed_back_waypoint = hand_back->second;
      }

      if (context->_get_reserved_location() != booking.assigned_waypoint_name)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "%s: about to drive [%s] to [%s] without holding its reservation "
          "(holding [%s] instead). The reservation system will be engaged for "
          "a vertex the zone manager holds, and this robot will wait "
          "indefinitely.",
          caller,
          context->requester_id().c_str(),
          booking.assigned_waypoint_name.c_str(),
          context->_get_reserved_location().c_str());
      }
    }

    result.status = ZoneStateResult::Status::Granted;
    result.goal = std::move(goal);
    result.waypoint_name = booking.assigned_waypoint_name;
    return result;
  }

  for (const auto& proceed : state.proceed)
  {
    if (proceed.robot_name != robot_name
      || proceed.fleet_name != fleet_name
      || proceed.zone_name != zone_name
      || proceed.request_id != request_id)
      continue;

    RCLCPP_INFO(
      node->get_logger(),
      "%s: [%s] may enter zone [%s] without a booking, so it will carry on "
      "with the plan it already has",
      caller,
      context->requester_id().c_str(),
      zone_name.c_str());

    result.status = ZoneStateResult::Status::Proceed;
    return result;
  }

  for (const auto& rejection : state.rejected)
  {
    if (rejection.robot_name != robot_name
      || rejection.fleet_name != fleet_name
      || rejection.request_id != request_id)
      continue;

    result.reason = rejection.reason;

    if (rejection.reason == "unknown_zone")
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "%s: the zone manager does not know zone [%s]",
        caller, zone_name.c_str());

      result.status = ZoneStateResult::Status::UnknownZone;
      return result;
    }

    if (rejection.reason == "zone_has_no_waypoints")
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "%s: zone [%s] has no waypoints, so nothing can ever be assigned in "
        "it. Check the zone's vertices in the building map",
        caller, zone_name.c_str());

      result.status = ZoneStateResult::Status::ZoneUnusable;
      return result;
    }

    if (rejection.reason == "waypoints_not_reserved")
    {
      RCLCPP_INFO(
        node->get_logger(),
        "%s: no waypoint in zone [%s] is reserved to the zone manager, so "
        "none can be assigned to [%s] yet. Is the reservation node running, "
        "and is something else holding this zone's waypoints?",
        caller,
        zone_name.c_str(),
        context->requester_id().c_str());

      result.status = ZoneStateResult::Status::Deferred;
      return result;
    }

    // A full zone.
    RCLCPP_INFO(
      node->get_logger(),
      "%s: request for [%s] in zone [%s] rejected (%s). The zone is most "
      "likely full, so waiting for it to change",
      caller,
      context->requester_id().c_str(),
      zone_name.c_str(),
      rejection.reason.c_str());

    result.status = ZoneStateResult::Status::Deferred;
    return result;
  }

  return result;
}

} // namespace phases
} // namespace rmf_fleet_adapter

