#include "Node.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <algorithm>

namespace rmf_fleet_adapter {
namespace zone_manager {

//==============================================================================
Node::Node(const rclcpp::NodeOptions& options)
: rclcpp::Node("zone_manager", options)
{
  // Subscribers
  auto transient_qos = rclcpp::QoS(10).transient_local().reliable();
  auto request_qos = rclcpp::QoS(100).transient_local().reliable();

  _nav_graphs_sub =
    this->create_subscription<rmf_building_map_msgs::msg::Graph>(
      NavGraphTopicName, transient_qos,
      [this](const rmf_building_map_msgs::msg::Graph::SharedPtr msg)
      { _on_nav_graphs(msg); });

  _zone_request_sub =
    this->create_subscription<rmf_zone_msgs::msg::ZoneRequest>(
      ZoneRequestTopicName, request_qos,
      [this](const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg)
      { _on_zone_request(msg); });

  _manual_release_sub =
    this->create_subscription<rmf_zone_msgs::msg::ZoneManualRelease>(
      ZoneManualReleaseTopicName, transient_qos,
      [this](const rmf_zone_msgs::msg::ZoneManualRelease::SharedPtr msg)
      { _on_manual_release(msg); });

  // Publishers
  _state_pub = this->create_publisher<rmf_zone_msgs::msg::ZoneState>(
    ZoneStateTopicName, transient_qos);

  _booking_revoked_pub =
    this->create_publisher<rmf_zone_msgs::msg::ZoneBookingRevoked>(
      ZoneBookingRevokedTopicName, transient_qos);

  _reservation_client = std::make_unique<ZoneReservationClient>(
    *this,
    [this](const std::string& zone) { _on_zone_pool_changed(zone); });

  // How long a request waits for reservations before being refused. The
  // refusal is not final, since the robot reads it as "try again" and does.
  _entry_request_timeout = std::chrono::seconds(
    this->declare_parameter<int>("entry_request_timeout", 30));

  // How long to wait for a zone's reservations to arrive before selecting
  _reservation_settle_time = std::chrono::milliseconds(
    this->declare_parameter<int>("reservation_settle_time_ms", 1000));

  // How long a zone sits unwanted before its reservations are handed back.
  _zone_pool_idle_timeout = std::chrono::seconds(
    this->declare_parameter<int>("zone_pool_idle_timeout", 30));

  _pending_entry_timer = this->create_wall_timer(
    std::chrono::milliseconds(200),
    [this]() { _tick_pending_entries(); });

  RCLCPP_INFO(this->get_logger(), "Zone Manager started.");
}

//==============================================================================
Node::~Node()
{
  // Do nothing
}

//==============================================================================
bool Node::_same_vertices(const ZoneInfo& a, const ZoneInfo& b) const
{
  if (a.waypoints.size() != b.waypoints.size())
    return false;

  for (std::size_t i = 0; i < a.waypoints.size(); ++i)
  {
    if (a.waypoints[i].name != b.waypoints[i].name
      || a.waypoints[i].group != b.waypoints[i].group
      || a.waypoints[i].priority != b.waypoints[i].priority)
      return false;
  }

  return true;
}

//==============================================================================
void Node::_on_nav_graphs(
  const rmf_building_map_msgs::msg::Graph::SharedPtr msg)
{
  const std::string fleet_name = msg->name;

  std::string zone_list;
  for (const auto& zone : msg->zones)
  {
    ZoneInfo info;
    for (const auto& vtx : zone.vertices)
    {
      ZoneWaypointInfo wp_info;
      wp_info.name = vtx.name;
      wp_info.priority = vtx.priority;
      wp_info.group = vtx.group;
      info.waypoints.push_back(std::move(wp_info));
    }

    std::sort(info.waypoints.begin(), info.waypoints.end(),
      [](const auto& a, const auto& b) { return a.priority < b.priority; });

    const auto known = _zones.find(zone.name);
    if (known != _zones.end() && !_same_vertices(known->second, info))
    {
      RCLCPP_WARN(this->get_logger(),
        "Zone [%s] is being redefined by fleet [%s] with different vertices. "
        "Zone names must mean the same place to every fleet, so check that "
        "these fleets were generated from the same building map",
        zone.name.c_str(), fleet_name.c_str());
    }

    _zones[zone.name] = std::move(info);

    if (!zone_list.empty())
      zone_list += ", ";
    zone_list += zone.name;
  }

  RCLCPP_INFO(this->get_logger(),
    "Registered %zu zones from fleet [%s] nav_graph, the zones are: [%s]",
    msg->zones.size(), fleet_name.c_str(), zone_list.c_str());

  if (!_ready)
  {
    _ready = true;
    RCLCPP_INFO(this->get_logger(),
      "Zone Manager ready. Processing %zu queued requests.",
      _pending_requests.size());
    for (const auto& pending : _pending_requests)
      _on_zone_request(pending);
    _pending_requests.clear();
  }
}

//==============================================================================
void Node::_on_zone_request(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg)
{
  if (!_ready)
  {
    constexpr std::size_t max_pending = 100;
    if (_pending_requests.size() >= max_pending)
    {
      RCLCPP_WARN(this->get_logger(),
        "Zone Manager pending queue is full (%zu), dropping the oldest "
        "request to make room. Is the nav graph being published?",
        max_pending);
      _pending_requests.erase(_pending_requests.begin());
    }

    RCLCPP_WARN_ONCE(this->get_logger(),
      "Zone Manager not ready, queuing requests until a nav graph arrives.");

    _pending_requests.push_back(msg);
    return;
  }

  if (msg->request_type == rmf_zone_msgs::msg::ZoneRequest::PREBOOKING)
    _process_prebooking(msg);
  else if (msg->request_type == rmf_zone_msgs::msg::ZoneRequest::ENTRY)
    _process_entry(msg);
  else if (msg->request_type == rmf_zone_msgs::msg::ZoneRequest::EXIT)
    _process_exit(msg);
  else if (msg->request_type == rmf_zone_msgs::msg::ZoneRequest::HANDBACK)
    _process_handback(msg);
  else if (msg->request_type == rmf_zone_msgs::msg::ZoneRequest::ARRIVED)
    _process_arrived(msg);
}

//==============================================================================
void Node::_process_arrived(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg)
{
  const auto held =
    _find_booking_for(msg->robot_name, msg->fleet_name, msg->zone_name);
  if (held == _zone_log.end())
  {
    // since every robot entering a zone reports arrival so
    // there is no log entry to flag.
    RCLCPP_DEBUG(this->get_logger(),
      "ZoneRequest ARRIVED from robot [%s/%s] holding no booking in zone "
      "[%s], ignoring it",
      msg->fleet_name.c_str(), msg->robot_name.c_str(),
      msg->zone_name.c_str());
    return;
  }

  if (held->second.arrived)
    return;

  // purpose of this arrived flag is for idle-zone checking, arrived flag true
  // means robot is parking inside the zone, and it doesn't need other zone waypoints
  // anymore and we can declare zone idle so zone-sweep can release those free waypoint
  // back to reservation node
  held->second.arrived = true;

  RCLCPP_INFO(this->get_logger(),
    "Robot [%s/%s] has arrived at waypoint [%s] in zone [%s]",
    msg->fleet_name.c_str(), msg->robot_name.c_str(),
    held->first.c_str(), msg->zone_name.c_str());
}

//==============================================================================
void Node::_process_handback(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg)
{
  if (msg->released_waypoint.empty())
  {
    RCLCPP_WARN(this->get_logger(),
      "ZoneRequest HANDBACK from robot [%s] named no waypoint",
      msg->robot_name.c_str());
    return;
  }

  // Freeze is when there is a back-to-back GoToZone, which robot
  // hadnback the waypoint to zone manager but keep the booking in zone log,
  // (because it is still inside the original zone) the booking will be cleared
  // at zone exit 
  auto disposal = ZoneReservationClient::HandbackDisposal::Reclaim;
  const auto logged = _zone_log.find(msg->released_waypoint);
  if (logged != _zone_log.end()
    && logged->second.zone_name == msg->zone_name
    && logged->second.robot_name == msg->robot_name
    && logged->second.fleet_name == msg->fleet_name)
  {
    disposal = ZoneReservationClient::HandbackDisposal::Freeze;
  }

  if (!_reservation_client->confirm_handback(
      msg->zone_name, msg->released_waypoint,
      msg->fleet_name, msg->robot_name, disposal))
  {
    return;
  }

  // A reclaimed vertex just became assignable, so anything parked on this
  // zone may now be answerable.
  _retry_pending_entries(msg->zone_name);
  _publish_state();
}

//==============================================================================
void Node::_process_entry(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg)
{
  const auto held =
    _find_booking_for(msg->robot_name, msg->fleet_name, msg->zone_name);

  if (held == _zone_log.end())
  {
    using Context = rmf_zone_msgs::msg::ZoneEntryContext;
    const auto& entry = msg->entry_context;

    const bool zone_task = entry.task_type == Context::TASK_ZONE
      || entry.task_type == Context::TASK_UNKNOWN;

    const bool stopping_here =
      entry.zone_relation == Context::RELATION_DESTINATION
      || entry.zone_relation == Context::RELATION_UNKNOWN;

    // not a zone task or not a destination zone, we just let the robot proceed
    // without any waypoint selection (transition zone will be supported at next
    // phase)
    if (!zone_task || !stopping_here)
    {
      RCLCPP_DEBUG(this->get_logger(),
        "Robot [%s/%s] is entering zone [%s] %s, proceeding as planned",
        msg->fleet_name.c_str(), msg->robot_name.c_str(),
        msg->zone_name.c_str(),
        zone_task ? "on its way elsewhere" : "without expecting a booking");

      _publish_state_with_proceed(
        msg->robot_name, msg->fleet_name, msg->request_id, msg->zone_name);
      return;
    }

    // A zone task stopping here with no booking. The prebooking never
    // reached us, a restart lost it, or it was revoked while the robot
    // drove. Book now rather than refuse, or the robot waits at the boundary
    // for a grant that only its own prebooking could have produced.
    RCLCPP_WARN(this->get_logger(),
      "ZoneRequest ENTRY from robot [%s/%s] on a zone task in zone [%s] but "
      "holding no booking, treating it as a prebooking",
      msg->fleet_name.c_str(), msg->robot_name.c_str(),
      msg->zone_name.c_str());

    _process_prebooking(msg);
    return;
  }

  _finalize_entry(msg, held);
}

//==============================================================================
void Node::_process_prebooking(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg,
  std::optional<rclcpp::Time> deadline,
  std::optional<rclcpp::Time> settle_by)
{
  const auto held =
    _find_booking_for(msg->robot_name, msg->fleet_name, msg->zone_name);
  if (held != _zone_log.end())
  {
    held->second.request_id = msg->request_id;
    _publish_state();
    return;
  }

  auto zone_it = _zones.find(msg->zone_name);
  if (zone_it == _zones.end())
  {
    // unknown zone requested
    RCLCPP_WARN(this->get_logger(),
      "ZoneRequest PREBOOKING: unknown zone [%s] requested by robot [%s]",
      msg->zone_name.c_str(), msg->robot_name.c_str());
    _publish_state_with_rejection(
      msg->robot_name, msg->fleet_name,
      msg->request_id, msg->zone_name, "unknown_zone");
    return;
  }

  // try to reserve all waypoint in this zone from reservation node
  _reservation_client->acquire(
    msg->zone_name, _zone_vertex_names(msg->zone_name));

  const auto now = this->now();
  const auto settle_deadline =
    settle_by.value_or(now + _reservation_settle_time);

  // Do not choose from a half-arrived field. We give some time to the
  // zone reservation client to finish reserving waypoints from reservation node
  if (now < settle_deadline
    && _reservation_client->acquisition_in_flight(msg->zone_name))
  {
    _park_entry_request(
      msg,
      deadline.value_or(now + _entry_request_timeout),
      settle_deadline);
    return;
  }

  // perform waypoint selection
  const auto result = _select_waypoint(msg->zone_name, msg->modifiers);

  if (!result.success)
  {
    if (result.awaiting_reservations)
    {
      _park_entry_request(
        msg, deadline.value_or(now + _entry_request_timeout), settle_deadline);
      return;
    }

    // Rejection
    _drop_pending_entry(msg->robot_name, msg->fleet_name, msg->zone_name);

    _publish_state_with_rejection(
      msg->robot_name, msg->fleet_name,
      msg->request_id, msg->zone_name, result.rejection_reason);
    return;
  }

  // there is a valid waypoint selected, so we can drop any pending request from
  // this robot
  _drop_pending_entry(msg->robot_name, msg->fleet_name, msg->zone_name);

  ZoneLogEntry log_entry;
  log_entry.robot_name = msg->robot_name;
  log_entry.fleet_name = msg->fleet_name;
  log_entry.zone_name = msg->zone_name;
  log_entry.assigned_at = this->now();
  log_entry.request_id = msg->request_id;
  log_entry.has_orientation = msg->modifiers.has_orientation_hint;
  log_entry.orientation = msg->modifiers.orientation_hint;

  _zone_log[result.selected.name] = std::move(log_entry);

  RCLCPP_INFO(this->get_logger(),
    "Booked waypoint [%s] in zone [%s] for robot [%s/%s]",
    result.selected.name.c_str(), msg->zone_name.c_str(),
    msg->fleet_name.c_str(), msg->robot_name.c_str());

  // The state we are about to publish carries the ticket, so from here the
  // vertex belongs to the robot.
  if (!_reservation_client->transfer(
      msg->zone_name, result.selected.name,
      msg->fleet_name, msg->robot_name))
  {
    RCLCPP_DEBUG(this->get_logger(),
      "No reservation to hand over for [%s]", result.selected.name.c_str());
  }

  _publish_state();
}

//==============================================================================
void Node::_park_entry_request(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg,
  rclcpp::Time deadline,
  rclcpp::Time settle_by)
{
  auto& parked = _pending_entries[msg->zone_name];

  for (auto& entry : parked)
  {
    if (entry.request->robot_name == msg->robot_name
      && entry.request->fleet_name == msg->fleet_name)
    {
      entry.request = msg;
      return;
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "No vertex assignable in zone [%s] for robot [%s] yet, waiting for its "
    "reservations to arrive",
    msg->zone_name.c_str(), msg->robot_name.c_str());

  parked.push_back(PendingEntry{msg, deadline, settle_by});
}

//==============================================================================
void Node::_retry_pending_entries(const std::string& zone_name)
{
  const auto it = _pending_entries.find(zone_name);
  if (it == _pending_entries.end() || it->second.empty())
    return;

  const auto retrying = it->second;

  for (const auto& entry : retrying)
    _process_prebooking(entry.request, entry.deadline, entry.settle_by);
}

//==============================================================================
void Node::_tick_pending_entries()
{
  // to remove any expired pending request
  if (!_pending_entries.empty())
  {
    std::vector<std::string> zones;
    zones.reserve(_pending_entries.size());
    for (const auto& [zone, parked] : _pending_entries)
      zones.push_back(zone);

    for (const auto& zone : zones)
      _retry_pending_entries(zone);

    _expire_pending_entries();
  }

  // to release not-in-used zone waypoints (from idle-zone) back to
  // reservation node
  _sweep_idle_zone_pools();
}

//==============================================================================
void Node::_expire_pending_entries()
{
  const auto now = this->now();
  std::vector<PendingEntry> expired;

  for (auto it = _pending_entries.begin(); it != _pending_entries.end(); )
  {
    auto& parked = it->second;
    for (auto e = parked.begin(); e != parked.end(); )
    {
      if (now >= e->deadline)
      {
        expired.push_back(*e);
        e = parked.erase(e);
      }
      else
      {
        ++e;
      }
    }

    it = parked.empty() ? _pending_entries.erase(it) : std::next(it);
  }

  for (const auto& entry : expired)
  {
    const auto& msg = entry.request;
    RCLCPP_WARN(
      this->get_logger(),
      "No vertex has come free in zone [%s] for robot [%s] in %lds, so it is "
      "being refused and will ask again",
      msg->zone_name.c_str(), msg->robot_name.c_str(),
      static_cast<long>(_entry_request_timeout.count()));

    _publish_state_with_rejection(
      msg->robot_name, msg->fleet_name,
      msg->request_id, msg->zone_name, "waypoints_not_reserved");
  }
}

//==============================================================================
void Node::_drop_pending_entry(
  const std::string& robot_name,
  const std::string& fleet_name,
  const std::string& zone_name)
{
  const auto it = _pending_entries.find(zone_name);
  if (it == _pending_entries.end())
    return;

  auto& parked = it->second;
  parked.erase(
    std::remove_if(parked.begin(), parked.end(),
      [&](const PendingEntry& e)
      {
        return e.request->robot_name == robot_name
        && e.request->fleet_name == fleet_name;
      }),
    parked.end());

  if (parked.empty())
    _pending_entries.erase(it);
}

//==============================================================================
void Node::_finalize_entry(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg,
  std::unordered_map<std::string, ZoneLogEntry>::iterator held)
{
  const std::string held_vertex = held->first;

  _drop_pending_entry(msg->robot_name, msg->fleet_name, msg->zone_name);

  // Try to re-acquire all the waypoints in the zone
  _reservation_client->acquire(
    msg->zone_name, _zone_vertex_names(msg->zone_name));

  const bool no_preference =
    msg->modifiers.group_hint.empty()
    && !msg->modifiers.has_orientation_hint
    && msg->modifiers.preferred_waypoints.empty();

  const auto result = no_preference
    ? SelectionResult()
    : _select_waypoint(msg->zone_name, msg->modifiers, held_vertex);

  if (!result.success || result.selected.name == held_vertex)
  {
    // Nothing better available, so the robot keeps what it has.
    RCLCPP_INFO(this->get_logger(),
      "Robot [%s/%s] finalized at [%s] in zone [%s]",
      msg->fleet_name.c_str(), msg->robot_name.c_str(),
      held_vertex.c_str(), msg->zone_name.c_str());

    held->second.request_id = msg->request_id;
    _publish_state();
    return;
  }

  const std::string new_vertex = result.selected.name;

  RCLCPP_INFO(this->get_logger(),
    "Re-assigning robot [%s/%s] from [%s] to [%s] in zone [%s]",
    msg->fleet_name.c_str(), msg->robot_name.c_str(),
    held_vertex.c_str(), new_vertex.c_str(), msg->zone_name.c_str());

  ZoneLogEntry entry = std::move(held->second);
  entry.request_id = msg->request_id;
  entry.assigned_at = this->now();

  _zone_log.erase(held);
  _zone_log[new_vertex] = std::move(entry);

  if (!_reservation_client->transfer(
      msg->zone_name, new_vertex, msg->fleet_name, msg->robot_name))
  {
    RCLCPP_DEBUG(this->get_logger(),
      "No reservation to hand over for [%s]", new_vertex.c_str());
  }

  _publish_state();
}

//==============================================================================
void Node::_process_exit(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg)
{
  // Booking or not, the robot is done with this zone.
  _drop_pending_entry(msg->robot_name, msg->fleet_name, msg->zone_name);

  auto it = _find_booking_for(
    msg->robot_name, msg->fleet_name, msg->zone_name);
  if (it == _zone_log.end())
  {
    // Ordinary since every robot leaving a zone says so, booked or not.
    RCLCPP_DEBUG(this->get_logger(),
      "ZoneRequest EXIT: no booking found for robot [%s] in zone [%s]",
      msg->robot_name.c_str(), msg->zone_name.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "Released booking at [%s] in zone [%s] for robot [%s]",
    it->first.c_str(), msg->zone_name.c_str(), msg->robot_name.c_str());

  const std::string released_wp = it->first;
  _zone_log.erase(it);

  _reservation_client->confirm_handback(
    msg->zone_name, released_wp, msg->fleet_name, msg->robot_name,
    ZoneReservationClient::HandbackDisposal::Reclaim);
  _retry_pending_entries(msg->zone_name);
  _publish_state();
}

//==============================================================================
Node::SelectionResult Node::_select_waypoint(
  const std::string& zone_name,
  const rmf_zone_msgs::msg::ZoneModifiers& modifiers,
  const std::string& held_vertex)
{
  auto zone_it = _zones.find(zone_name);
  if (zone_it == _zones.end())
    return {false, {}, "unknown_zone"};

  // 1. Copy the waypoint list (already sorted by priority ascending)
  auto candidates = zone_it->second.waypoints;

  if (candidates.empty())
  {
    // A zone declared with no internal vertices. Nothing about the fleet or
    // the reservation node will ever change that, so report it specifically
    // rather than let a misconfigured zone read as a busy one and block a
    // robot indefinitely.
    return {false, {}, "zone_has_no_waypoints"};
  }

  // 2a. Drop waypoints that are already booked.
  candidates.erase(
    std::remove_if(candidates.begin(), candidates.end(),
      [this, &held_vertex](const auto& wp)
      { return wp.name != held_vertex && _zone_log.count(wp.name) > 0; }),
    candidates.end());

  const bool all_booked = candidates.empty();

  // 2b. Only hand out vertices we hold and have not already given away.
  if (_reservation_client->reservation_node_present())
  {
    candidates.erase(
      std::remove_if(candidates.begin(), candidates.end(),
        [this, &held_vertex, &zone_name](const auto& wp)
        {
          return wp.name != held_vertex
          && !_reservation_client->is_available(zone_name, wp.name);
        }),
      candidates.end());
  }
  else
  {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "No reservation node is listening, so zone waypoints are being assigned "
      "without reservations to back them. Assignments are advisory only and "
      "do not exclude other robots.");
  }

  // 3. Group filtering: apply group_hint if provided.
  if (!modifiers.group_hint.empty())
  {
    auto group_filtered = candidates;
    group_filtered.erase(
      std::remove_if(group_filtered.begin(), group_filtered.end(),
        [&](const auto& wp)
        { return wp.group != modifiers.group_hint; }),
      group_filtered.end());

    if (!group_filtered.empty())
      candidates = std::move(group_filtered);
    // No match for group filter, fall back to all candidates
  }

  // 4. Preferred waypoints filtering:
  // if specified, return the first available
  // match from the preference list.
  if (!modifiers.preferred_waypoints.empty())
  {
    for (const auto& pref : modifiers.preferred_waypoints)
    {
      for (const auto& wp : candidates)
      {
        if (wp.name == pref)
          return {true, wp, ""};
      }
    }
  }

  // 5. Priority selection: Select highest priority (first in sorted list).
  if (candidates.empty())
  {
    // Report which kind of empty.
    const bool pending =
      _reservation_client->acquisition_in_flight(zone_name);

    return {
      false, {},
      all_booked ? "zone_full" : "waypoints_not_reserved",
      pending};
  }

  return {true, candidates.front(), ""};
}

//==============================================================================
void Node::_on_manual_release(
  const rmf_zone_msgs::msg::ZoneManualRelease::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),
    "Manual release requested for robot [%s/%s] in zone [%s]",
    msg->fleet_name.c_str(), msg->robot_name.c_str(),
    msg->zone_name.c_str());

  _remove_booking_for_robot(
    msg->robot_name, msg->fleet_name, msg->zone_name, "manual_release");
}

//==============================================================================
void Node::_remove_booking(
  const std::string& wp_name, const std::string& reason)
{
  auto it = _zone_log.find(wp_name);
  if (it == _zone_log.end())
    return;

  const auto entry = it->second;
  _zone_log.erase(it);

  _reservation_client->release(entry.zone_name, wp_name);

  auto revoked = rmf_zone_msgs::msg::ZoneBookingRevoked();
  revoked.stamp = this->now();
  revoked.robot_name = entry.robot_name;
  revoked.fleet_name = entry.fleet_name;
  revoked.zone_name = entry.zone_name;
  revoked.assigned_waypoint_name = wp_name;
  revoked.reason = reason;
  _booking_revoked_pub->publish(revoked);

  _retry_pending_entries(entry.zone_name);
  _publish_state();
}

//==============================================================================
rmf_zone_msgs::msg::ZoneState Node::_build_state_msg()
{
  auto state = rmf_zone_msgs::msg::ZoneState();
  state.stamp = this->now();

  for (const auto& [wp_name, entry] : _zone_log)
  {
    auto booking = rmf_zone_msgs::msg::ZoneBooking();
    booking.stamp = entry.assigned_at;
    booking.robot_name = entry.robot_name;
    booking.fleet_name = entry.fleet_name;
    booking.zone_name = entry.zone_name;
    booking.assigned_waypoint_name = wp_name;
    booking.has_orientation = entry.has_orientation;
    booking.orientation = entry.orientation;
    booking.request_id = entry.request_id;

    _reservation_client->transfer(
      entry.zone_name, wp_name, entry.fleet_name, entry.robot_name);

    const auto ticket =
      _reservation_client->transferred_ticket_for(
        entry.zone_name, wp_name);
    if (ticket)
    {
      booking.has_ticket = true;
      booking.ticket_id = *ticket;
      booking.ticket_resource = wp_name;
    }
    else
    {
      booking.has_ticket = false;
      booking.ticket_id = 0;
    }

    state.bookings.push_back(std::move(booking));
  }

  return state;
}

//==============================================================================
void Node::_publish_state()
{
  _state_pub->publish(_build_state_msg());
}

//==============================================================================
void Node::_publish_state_with_proceed(
  const std::string& robot_name,
  const std::string& fleet_name,
  const std::string& request_id,
  const std::string& zone_name)
{
  auto state = _build_state_msg();

  auto proceed = rmf_zone_msgs::msg::ZoneProceed();
  proceed.robot_name = robot_name;
  proceed.fleet_name = fleet_name;
  proceed.request_id = request_id;
  proceed.zone_name = zone_name;
  state.proceed.push_back(std::move(proceed));

  _state_pub->publish(state);
}

//==============================================================================
void Node::_publish_state_with_rejection(
  const std::string& robot_name,
  const std::string& fleet_name,
  const std::string& request_id,
  const std::string& zone_name,
  const std::string& reason)
{
  auto state = _build_state_msg();

  auto rejection = rmf_zone_msgs::msg::ZoneRejection();
  rejection.robot_name = robot_name;
  rejection.fleet_name = fleet_name;
  rejection.request_id = request_id;
  rejection.zone_name = zone_name;
  rejection.reason = reason;
  state.rejected.push_back(std::move(rejection));

  _state_pub->publish(state);
}

//==============================================================================
void Node::_remove_booking_for_robot(
  const std::string& robot_name,
  const std::string& fleet_name,
  const std::string& zone_name,
  const std::string& reason)
{
  auto it = _find_booking_for(robot_name, fleet_name, zone_name);
  if (it == _zone_log.end())
  {
    RCLCPP_WARN(this->get_logger(),
      "No booking found for robot [%s] (fleet [%s]) in zone [%s] during %s",
      robot_name.c_str(), fleet_name.c_str(), zone_name.c_str(),
      reason.c_str());
    return;
  }

  const std::string wp_name = it->first;
  RCLCPP_INFO(this->get_logger(),
    "Removing booking for robot [%s] at waypoint [%s] in zone [%s] "
    "(reason: %s)",
    robot_name.c_str(), wp_name.c_str(), zone_name.c_str(), reason.c_str());
  _remove_booking(wp_name, reason);
}

//==============================================================================
std::vector<std::string> Node::_zone_vertex_names(
  const std::string& zone_name) const
{
  std::vector<std::string> names;
  const auto it = _zones.find(zone_name);
  if (it == _zones.end())
    return names;

  names.reserve(it->second.waypoints.size());
  for (const auto& wp : it->second.waypoints)
    names.push_back(wp.name);

  return names;
}

//==============================================================================
void Node::_on_zone_pool_changed(const std::string& zone_name)
{
  // the waypoint state changed, so we can request a retry on pending request
  RCLCPP_DEBUG(
    this->get_logger(),
    "Reservation pool for zone [%s] changed, re-advertising zone state",
    zone_name.c_str());

  _retry_pending_entries(zone_name);
  _publish_state();
}

//==============================================================================
bool Node::_zone_pool_is_idle(const std::string& zone_name) const
{
  for (const auto& [wp_name, entry] : _zone_log)
  {
    // if there is any booking related to this zone and the robot
    // hasn't parked on the zone waypoint, we consider this zone as
    // non-idle
    if (entry.zone_name == zone_name && !entry.arrived)
      return false;
  }

  // if there is pending request regardng this zone, which means some robot
  // still need thezone waypoint, so we don't relase to reservation node, and
  // marks this zoneas non-idle
  const auto pending = _pending_entries.find(zone_name);
  if (pending != _pending_entries.end() && !pending->second.empty())
    return false;

  return true;
}

//==============================================================================
void Node::_sweep_idle_zone_pools()
{
  const auto held_zones = _reservation_client->zones_with_holdings();
  const auto now = this->now();

  for (const auto& zone : held_zones)
  {
    if (!_zone_pool_is_idle(zone))
    {
      _zone_idle_since.erase(zone);
      continue;
    }

    // is an idle zone
    const auto it = _zone_idle_since.find(zone);
    if (it == _zone_idle_since.end())
    {
      _zone_idle_since[zone] = now;
      continue;
    }

    if (now - it->second < rclcpp::Duration(_zone_pool_idle_timeout))
      continue;

    // Nothing has wanted this zone for some time, so stop holding its
    // vertices, release the not-in-used waypoint back to reservation
    // node
    const auto given_up = _reservation_client->release_zone(zone);

    if (given_up.released > 0)
    {
      RCLCPP_INFO(this->get_logger(),
        "Released %lu reservation(s) for zone [%s] after %ld s idle",
        given_up.released, zone.c_str(),
        static_cast<long>(_zone_pool_idle_timeout.count()));
    }

    // Reported apart from the releases. A claim we never had granted means
    // something outside the zone system is sitting on that vertex.
    if (given_up.dropped_claims > 0)
    {
      RCLCPP_INFO(this->get_logger(),
        "Dropped %lu queued claim(s) in zone [%s], whose waypoints are held "
        "by somebody outside the zone system",
        given_up.dropped_claims, zone.c_str());
    }

    std::vector<std::string> orphaned;
    for (const auto& vertex : _reservation_client->transferred_vertices(zone))
    {
      // check if there is any transferred vertices but not in zone log
      if (_zone_log.count(vertex) == 0)
        orphaned.push_back(vertex);
    }

    if (!orphaned.empty())
    {
      std::string list;
      for (const auto& vertex : orphaned)
        list += (list.empty() ? "" : ", ") + vertex;

      RCLCPP_WARN(this->get_logger(),
        "Zone [%s] still holds [%s] for a robot with no booking, which "
        "cannot be reclaimed from here",
        zone.c_str(), list.c_str());
    }

    // Re-stamp while anything is still held, to acknowledge that we have
    // already checked now
    if (_reservation_client->has_holdings(zone))
    {
      _zone_idle_since[zone] = now;
      continue;
    }

    // if nothing is held, erase the stamp
    _zone_idle_since.erase(zone);
  }

  // Housekeeping: drop the stamps for zones we no longer hold. There is no
  // pool left to release, and stale entries would accumulate.
  for (auto it = _zone_idle_since.begin(); it != _zone_idle_since.end(); )
  {
    const auto& stamped_zone = it->first;
    if (held_zones.count(stamped_zone) > 0)
    {
      ++it;
    }
    else
    {
      it = _zone_idle_since.erase(it);
    }
  }
}

//==============================================================================
std::unordered_map<std::string, Node::ZoneLogEntry>::iterator
Node::_find_booking_for(
  const std::string& robot_name,
  const std::string& fleet_name,
  const std::string& zone_name)
{
  for (auto it = _zone_log.begin(); it != _zone_log.end(); ++it)
  {
    if (it->second.robot_name == robot_name
      && it->second.fleet_name == fleet_name
      && it->second.zone_name == zone_name)
    {
      return it;
    }
  }
  return _zone_log.end();
}

} // namespace zone_manager
} // namespace rmf_fleet_adapter
