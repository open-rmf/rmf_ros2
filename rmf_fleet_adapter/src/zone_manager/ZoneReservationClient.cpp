/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "ZoneReservationClient.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>

namespace rmf_fleet_adapter {
namespace zone_manager {

//==============================================================================
ZoneReservationClient::ZoneReservationClient(
  rclcpp::Node& node,
  std::function<void(const std::string&)> on_zone_changed)
: _node(node),
  _on_zone_changed(std::move(on_zone_changed))
{
  const auto qos = rclcpp::QoS(10).transient_local().reliable();

  _request_pub =
    _node.create_publisher<rmf_reservation_msgs::msg::FlexibleTimeRequest>(
    ReservationRequestTopicName, qos);

  _claim_pub =
    _node.create_publisher<rmf_reservation_msgs::msg::ClaimRequest>(
    ReservationClaimTopicName, qos);

  _release_pub =
    _node.create_publisher<rmf_reservation_msgs::msg::ReleaseRequest>(
    ReservationReleaseTopicName, qos);

  _ticket_sub =
    _node.create_subscription<rmf_reservation_msgs::msg::Ticket>(
    ReservationResponseTopicName, qos,
    [this](const rmf_reservation_msgs::msg::Ticket::SharedPtr msg)
    {
      _on_ticket(*msg);
    });

  _allocation_sub =
    _node.create_subscription<rmf_reservation_msgs::msg::ReservationAllocation>(
    ReservationAllocationTopicName, qos,
    [this](
      const rmf_reservation_msgs::msg::ReservationAllocation::SharedPtr msg)
    {
      _on_allocation(*msg);
    });
}

//==============================================================================
void ZoneReservationClient::acquire(
  const std::string& zone,
  const std::vector<std::string>& vertex_names)
{
  for (const auto& vertex : vertex_names)
  {
    const auto existing = _holdings.find(vertex);
    if (existing != _holdings.end())
    {
      if (existing->second.zone != zone)
      {
        // Two zones claiming one name means their maps disagree. Taking
        // the other's reservation would send a robot where it may not go.
        RCLCPP_ERROR(
          _node.get_logger(),
          "Vertex [%s] is claimed by both zone [%s] and zone [%s]. Zone "
          "waypoint names must be unique across every fleet's nav graph, so "
          "[%s] will not be assignable in zone [%s]",
          vertex.c_str(), existing->second.zone.c_str(), zone.c_str(),
          vertex.c_str(), zone.c_str());
      }
      continue;
    }

    Holding holding;
    holding.zone = zone;
    holding.vertex = vertex;
    holding.request_id = _next_request_id++;
    holding.state = Holding::State::Requested;

    rmf_reservation_msgs::msg::FlexibleTimeRequest request;
    request.header.robot_name = _robot_name;
    request.header.fleet_name = _fleet_name;
    request.header.request_id = holding.request_id;

    rmf_reservation_msgs::msg::FlexibleTimeReservationAlt alternative;
    alternative.resource_name = vertex;
    // Not ranked by travel time. The manager does not travel.
    alternative.cost = 0.0;
    alternative.has_end = false;

    rmf_reservation_msgs::msg::StartTimeRange start;
    start.earliest_start_time = _node.get_clock()->now();
    start.latest_start_time = start.earliest_start_time;
    start.has_earliest_start_time = true;
    start.has_latest_start_time = true;
    alternative.start_time = start;

    request.alternatives.push_back(std::move(alternative));

    _holdings.emplace(vertex, std::move(holding));
    _request_pub->publish(request);

    RCLCPP_DEBUG(
      _node.get_logger(),
      "Zone manager requested a reservation on [%s] for zone [%s]",
      vertex.c_str(), zone.c_str());
  }
}

//==============================================================================
void ZoneReservationClient::_on_ticket(
  const rmf_reservation_msgs::msg::Ticket& msg)
{
  if (msg.header.robot_name != _robot_name
    || msg.header.fleet_name != _fleet_name)
    return;

  auto* holding = _find_by_request_id(msg.header.request_id);
  if (!holding)
    return;

  holding->ticket_id = msg.ticket_id;

  rmf_reservation_msgs::msg::ClaimRequest claim;
  claim.ticket = msg;
  _claim_pub->publish(claim);
}

//==============================================================================
void ZoneReservationClient::_on_allocation(
  const rmf_reservation_msgs::msg::ReservationAllocation& msg)
{
  if (msg.ticket.header.robot_name != _robot_name
    || msg.ticket.header.fleet_name != _fleet_name)
    return;

  auto* holding = _find_by_ticket_id(msg.ticket.ticket_id);
  if (!holding)
  {
    // An allocation for a vertex we gave up. Only an allocated ticket can
    // be released, so ours did nothing at the time and has to go again.
    RCLCPP_INFO(
      _node.get_logger(),
      "Zone manager releasing [%s] on ticket %lu: it arrived after we had "
      "already given that vertex up",
      msg.resource.c_str(), msg.ticket.ticket_id);

    rmf_reservation_msgs::msg::ReleaseRequest release;
    release.ticket = msg.ticket;
    release.location = msg.resource;
    _release_pub->publish(release);
    return;
  }

  if (msg.resource != holding->vertex)
  {
    // We asked for exactly one vertex, so anything else was substituted,
    // and a reservation outside the zone is no use to us.
    RCLCPP_WARN(
      _node.get_logger(),
      "Zone manager asked for [%s] but was allocated [%s], releasing it",
      holding->vertex.c_str(), msg.resource.c_str());

    rmf_reservation_msgs::msg::ReleaseRequest release;
    release.ticket = msg.ticket;
    release.location = msg.resource;
    _release_pub->publish(release);

    const std::string vertex = holding->vertex;
    const std::string zone = holding->zone;
    _holdings.erase(vertex);
    if (_on_zone_changed)
      _on_zone_changed(zone);
    return;
  }

  if (holding->state != Holding::State::Requested)
    return;

  holding->state = Holding::State::Held;

  RCLCPP_INFO(
    _node.get_logger(),
    "Zone manager now holds [%s] in zone [%s] on ticket %lu",
    holding->vertex.c_str(), holding->zone.c_str(), msg.ticket.ticket_id);

  const std::string zone = holding->zone;
  if (_on_zone_changed)
    _on_zone_changed(zone);
}

//==============================================================================
bool ZoneReservationClient::is_available(
  const std::string& zone, const std::string& vertex) const
{
  const auto* holding = _find(zone, vertex);
  return holding && holding->state == Holding::State::Held;
}

//==============================================================================
std::optional<uint64_t> ZoneReservationClient::transferred_ticket_for(
  const std::string& zone, const std::string& vertex) const
{
  const auto* holding = _find(zone, vertex);
  if (!holding)
    return std::nullopt;

  if (holding->state != Holding::State::Transferred)
    return std::nullopt;

  return holding->ticket_id;
}

//==============================================================================
bool ZoneReservationClient::transfer(
  const std::string& zone,
  const std::string& vertex,
  const std::string& fleet,
  const std::string& robot)
{
  auto* holding = _find(zone, vertex);
  if (!holding || holding->state != Holding::State::Held)
    return false;

  holding->state = Holding::State::Transferred;
  holding->transferred_to = Holding::RobotId{fleet, robot};

  RCLCPP_DEBUG(
    _node.get_logger(),
    "Zone manager handed [%s] to [%s/%s], the ticket is theirs now",
    vertex.c_str(), fleet.c_str(), robot.c_str());

  return true;
}

//==============================================================================
bool ZoneReservationClient::confirm_handback(
  const std::string& zone,
  const std::string& vertex,
  const std::string& fleet,
  const std::string& robot,
  const HandbackDisposal disposal)
{
  auto* holding = _find(zone, vertex);
  if (!holding)
    return false;

  const bool frozen =
    holding->state == Holding::State::Frozen;

  // Nothing to take back. Letting go of the ticket and leaving the zone are
  // two messages, so the first freezes and the second thaws.
  if (holding->state != Holding::State::Transferred && !frozen)
    return false;

  if (frozen && disposal == HandbackDisposal::Freeze)
    return false;

  if (holding->transferred_to != Holding::RobotId{fleet, robot})
  {
    RCLCPP_WARN(
      _node.get_logger(),
      "Ignoring handback of [%s] from [%s/%s]: it is held by another robot",
      vertex.c_str(), fleet.c_str(), robot.c_str());
    return false;
  }

  if (disposal == HandbackDisposal::Freeze)
  {
    holding->state = Holding::State::Frozen;

    RCLCPP_INFO(
      _node.get_logger(),
      "Zone manager froze [%s], which [%s/%s] is still parked on",
      vertex.c_str(), fleet.c_str(), robot.c_str());

    return true;
  }

  holding->state = Holding::State::Held;
  holding->transferred_to = std::nullopt;

  RCLCPP_INFO(
    _node.get_logger(),
    "Zone manager took [%s] back from [%s/%s]",
    vertex.c_str(), fleet.c_str(), robot.c_str());

  return true;
}

//==============================================================================
void ZoneReservationClient::_disown(const std::string& vertex)
{
  const auto it = _holdings.find(vertex);
  if (it == _holdings.end())
    return;

  RCLCPP_DEBUG(
    _node.get_logger(),
    "Zone manager forgetting [%s]: it was transferred, so its robot owns "
    "the release",
    vertex.c_str());

  _holdings.erase(it);
}

//==============================================================================
void ZoneReservationClient::release(
  const std::string& zone, const std::string& vertex)
{
  const auto it = _holdings.find(vertex);
  if (it == _holdings.end() || !_find(zone, vertex))
    return;

  if (it->second.state == Holding::State::Transferred)
  {
    // Not ours to release. The robot owns it now.
    _disown(vertex);
    return;
  }

  // Only an allocated ticket can be released, and _on_allocation catches a
  // still queued one later. Frozen counts as ours, like Held.
  if ((it->second.state == Holding::State::Held
    || it->second.state == Holding::State::Frozen)
    && it->second.ticket_id.has_value())
  {
    rmf_reservation_msgs::msg::ReleaseRequest release_msg;
    release_msg.ticket.ticket_id = *it->second.ticket_id;
    release_msg.ticket.header.robot_name = _robot_name;
    release_msg.ticket.header.fleet_name = _fleet_name;
    release_msg.ticket.header.request_id = it->second.request_id;
    release_msg.location = vertex;
    _release_pub->publish(release_msg);

    RCLCPP_INFO(
      _node.get_logger(),
      "Zone manager released [%s] in zone [%s]",
      vertex.c_str(), zone.c_str());
  }

  _holdings.erase(it);
}

//==============================================================================
std::size_t ZoneReservationClient::release_zone(const std::string& zone)
{
  std::vector<std::string> to_release;
  for (const auto& [vertex, holding] : _holdings)
  {
    // Transferred belongs to its robot, and a frozen one has a robot still
    // on it and need to wait for it to exit zone. Releasing either takes a
    // vertex out from under a robot.
    if (holding.zone == zone
      && holding.state != Holding::State::Transferred
      && holding.state != Holding::State::Frozen)
      to_release.push_back(vertex);
  }

  for (const auto& vertex : to_release)
    release(zone, vertex);

  return to_release.size();
}

//==============================================================================
bool ZoneReservationClient::reservation_node_present() const
{
  return _request_pub->get_subscription_count() > 0;
}

//==============================================================================
bool ZoneReservationClient::has_holdings(const std::string& zone) const
{
  for (const auto& [vertex, holding] : _holdings)
  {
    if (holding.zone == zone)
      return true;
  }
  return false;
}

//==============================================================================
std::vector<std::string> ZoneReservationClient::transferred_vertices(
  const std::string& zone) const
{
  std::vector<std::string> vertices;
  for (const auto& [vertex, holding] : _holdings)
  {
    if (holding.zone == zone
      && holding.state == Holding::State::Transferred)
      vertices.push_back(vertex);
  }
  return vertices;
}

//==============================================================================
std::unordered_set<std::string> ZoneReservationClient::zones_with_holdings()
const
{
  std::unordered_set<std::string> zones;
  for (const auto& [vertex, holding] : _holdings)
    zones.insert(holding.zone);

  return zones;
}

//==============================================================================
bool ZoneReservationClient::acquisition_in_flight(
  const std::string& zone) const
{
  for (const auto& [vertex, holding] : _holdings)
  {
    if (holding.zone == zone && holding.state == Holding::State::Requested)
      return true;
  }
  return false;
}

//==============================================================================
auto ZoneReservationClient::_find(
  const std::string& zone, const std::string& vertex) -> Holding*
{
  const auto it = _holdings.find(vertex);
  if (it == _holdings.end())
    return nullptr;

  if (it->second.zone != zone)
  {
    RCLCPP_ERROR(
      _node.get_logger(),
      "Zone [%s] asked about vertex [%s], which is held for zone [%s]. "
      "Refusing to act on another zone's reservation",
      zone.c_str(), vertex.c_str(), it->second.zone.c_str());
    return nullptr;
  }

  return &it->second;
}

//==============================================================================
auto ZoneReservationClient::_find(
  const std::string& zone, const std::string& vertex) const -> const Holding*
{
  return const_cast<ZoneReservationClient&>(*this)._find(zone, vertex);
}

//==============================================================================
auto ZoneReservationClient::_find_by_request_id(uint64_t request_id) -> Holding*
{
  for (auto& [vertex, holding] : _holdings)
  {
    if (holding.request_id == request_id
      && holding.state == Holding::State::Requested)
      return &holding;
  }
  return nullptr;
}

//==============================================================================
auto ZoneReservationClient::_find_by_ticket_id(uint64_t ticket_id) -> Holding*
{
  for (auto& [vertex, holding] : _holdings)
  {
    if (holding.ticket_id.has_value() && *holding.ticket_id == ticket_id)
      return &holding;
  }
  return nullptr;
}

} // namespace zone_manager
} // namespace rmf_fleet_adapter
