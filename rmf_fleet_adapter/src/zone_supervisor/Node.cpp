#include "Node.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <algorithm>
#include <cmath>

namespace rmf_fleet_adapter {
namespace zone_supervisor {

//==============================================================================
Node::Node(const rclcpp::NodeOptions& options)
: rclcpp::Node("zone_supervisor", options)
{
  // Declare parameters
  _stale_booking_check_interval =
    this->declare_parameter("stale_booking_check_interval", 60.0);
  _stale_booking_distance_threshold =
    this->declare_parameter("stale_booking_distance_threshold", 1.0);
  _stale_booking_grace_period =
    this->declare_parameter("stale_booking_grace_period", 180.0);

  // All callbacks share a single MutuallyExclusive group so that they
  // serialize through one thread. This avoids data races on _zones,
  // _zone_log, and _robot_positions when running under MultiThreadedExecutor.
  _callback_group = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions cb_opts;
  cb_opts.callback_group = _callback_group;

  // Subscribers
  auto transient_qos = rclcpp::QoS(10).transient_local().reliable();

  _nav_graphs_sub =
    this->create_subscription<rmf_building_map_msgs::msg::Graph>(
      NavGraphTopicName, transient_qos,
      [this](const rmf_building_map_msgs::msg::Graph::SharedPtr msg)
      { _on_nav_graphs(msg); },
      cb_opts);

  _zone_request_sub =
    this->create_subscription<rmf_zone_msgs::msg::ZoneRequest>(
      ZoneRequestTopicName, rclcpp::QoS(10).reliable(),
      [this](const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg)
      { _on_zone_request(msg); },
      cb_opts);

  _manual_release_sub =
    this->create_subscription<rmf_zone_msgs::msg::ZoneManualRelease>(
      ZoneManualReleaseTopicName, 10,
      [this](const rmf_zone_msgs::msg::ZoneManualRelease::SharedPtr msg)
      { _on_manual_release(msg); },
      cb_opts);

  _fleet_states_sub =
    this->create_subscription<rmf_fleet_msgs::msg::FleetState>(
      FleetStateTopicName, 10,
      [this](const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
      { _on_fleet_state(msg); },
      cb_opts);

  // Publishers
  _state_pub = this->create_publisher<rmf_zone_msgs::msg::ZoneState>(
    ZoneStateTopicName, transient_qos);

  _booking_revoked_pub =
    this->create_publisher<rmf_zone_msgs::msg::ZoneBookingRevoked>(
      ZoneBookingRevokedTopicName, 10);

  // Stale booking check timer
  _stale_booking_timer = this->create_wall_timer(
    std::chrono::duration<double>(_stale_booking_check_interval),
    [this]() { _on_stale_booking_check(); },
    _callback_group);

  RCLCPP_INFO(this->get_logger(), "Zone Supervisor started.");
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
      wp_info.x = vtx.x;
      wp_info.y = vtx.y;
      wp_info.priority = vtx.priority;
      wp_info.group = vtx.group;
      info.waypoints.push_back(std::move(wp_info));
      info.waypoint_positions[vtx.name] = {vtx.x, vtx.y};
    }

    std::sort(info.waypoints.begin(), info.waypoints.end(),
      [](const auto& a, const auto& b) { return a.priority < b.priority; });

    if (_zones.find(zone.name) != _zones.end())
    {
      RCLCPP_DEBUG(this->get_logger(),
        "Zone [%s] already registered; updating from fleet [%s].",
        zone.name.c_str(), fleet_name.c_str());
    }
    // assuming that zone names are unique across fleets,
    // later insert will overwrite earlier one if there are duplicates
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
      "Zone Supervisor ready. Processing %zu queued requests.",
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
      RCLCPP_ERROR(this->get_logger(),
        "Zone Supervisor not ready and pending queue is full (%zu). "
        "Dropping request from [%s/%s] for zone [%s]. "
        "Is the nav graph being published?",
        max_pending,
        msg->fleet_name.c_str(), msg->robot_name.c_str(),
        msg->zone_name.c_str());
      return;
    }
    RCLCPP_WARN(this->get_logger(),
      "Zone Supervisor not ready. Queuing request from [%s/%s].",
      msg->fleet_name.c_str(), msg->robot_name.c_str());
    _pending_requests.push_back(msg);
    return;
  }

  if (msg->request_type == rmf_zone_msgs::msg::ZoneRequest::ENTRY)
    _process_entry(msg);
  else if (msg->request_type == rmf_zone_msgs::msg::ZoneRequest::EXIT)
    _process_exit(msg);
}

//==============================================================================
void Node::_process_entry(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg)
{
  // Already booked? Return existing booking.
  for (auto& [wp_name, entry] : _zone_log)
  {
    if (entry.robot_name == msg->robot_name
      && entry.fleet_name == msg->fleet_name
      && entry.zone_name == msg->zone_name)
    {
      RCLCPP_DEBUG(this->get_logger(),
        "Robot [%s] already has booking at [%s] in zone [%s]. "
        "Returning existing booking.",
        msg->robot_name.c_str(), wp_name.c_str(),
        msg->zone_name.c_str());
      // Refresh request_id so the new request can match the existing
      // booking via exact-equality in ZoneEntry.
      entry.request_id = msg->request_id;
      _publish_state();
      return;
    }
  }

  auto zone_it = _zones.find(msg->zone_name);
  if (zone_it == _zones.end())
  {
    // unknown zone requested
    RCLCPP_WARN(this->get_logger(),
      "ZoneRequest ENTRY: unknown zone [%s] requested by robot [%s]",
      msg->zone_name.c_str(), msg->robot_name.c_str());
    _publish_state_with_rejection(
      msg->robot_name, msg->fleet_name,
      msg->request_id, msg->zone_name, "unknown_zone");
    return;
  }

  const auto result = _select_waypoint(msg->zone_name, msg->modifiers);

  if (!result.success)
  {
    _publish_state_with_rejection(
      msg->robot_name, msg->fleet_name,
      msg->request_id, msg->zone_name, result.rejection_reason);
    return;
  }

  // Book in zone_log.
  ZoneLogEntry log_entry;
  log_entry.robot_name = msg->robot_name;
  log_entry.fleet_name = msg->fleet_name;
  log_entry.zone_name = msg->zone_name;
  log_entry.assigned_at = this->now();
  log_entry.request_id = msg->request_id;
  log_entry.has_orientation = msg->modifiers.has_orientation_hint;
  log_entry.orientation = msg->modifiers.orientation_hint;

  _insert_booking(result.selected.name, std::move(log_entry));

  RCLCPP_INFO(this->get_logger(),
    "Booked waypoint [%s] in zone [%s] for robot [%s/%s]",
    result.selected.name.c_str(), msg->zone_name.c_str(),
    msg->fleet_name.c_str(), msg->robot_name.c_str());

  _publish_state();
}

//==============================================================================
void Node::_process_exit(
  const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg)
{
  auto it = _find_booking_for(
    msg->robot_name, msg->fleet_name, msg->zone_name);
  if (it == _zone_log.end())
  {
    RCLCPP_WARN(this->get_logger(),
      "ZoneRequest EXIT: no booking found for robot [%s] in zone [%s]",
      msg->robot_name.c_str(), msg->zone_name.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "Released booking at [%s] in zone [%s] for robot [%s]",
    it->first.c_str(), msg->zone_name.c_str(), msg->robot_name.c_str());

  _suspect_bookings.erase(it->first);
  _erase_booking(it);
  _erase_robot_position(msg->fleet_name, msg->robot_name);
  _publish_state();
}

//==============================================================================
Node::SelectionResult Node::_select_waypoint(
  const std::string& zone_name,
  const rmf_zone_msgs::msg::ZoneModifiers& modifiers)
{
  auto zone_it = _zones.find(zone_name);
  if (zone_it == _zones.end())
    return {false, {}, "unknown_zone"};

  // 1. Copy the waypoint list (already sorted by priority ascending)
  auto candidates = zone_it->second.waypoints;

  // 2. Zone_log filtering: Filter out already-booked waypoints.
  candidates.erase(
    std::remove_if(candidates.begin(), candidates.end(),
      [this](const auto& wp) { return _zone_log.count(wp.name) > 0; }),
    candidates.end());

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
    return {false, {}, "waypoint_unavailable"};

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
void Node::_on_fleet_state(
  const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
{
  // Only track positions of robots that have active zone bookings.
  auto fleet_it = _booked_robots_by_fleet.find(msg->name);
  if (fleet_it == _booked_robots_by_fleet.end())
    return;

  const auto& booked = fleet_it->second;
  for (const auto& robot : msg->robots)
  {
    if (booked.count(robot.name))
    {
      _robot_positions[msg->name][robot.name] = {
        robot.location.x, robot.location.y
      };
    }
  }
}

//==============================================================================
void Node::_on_stale_booking_check()
{
  std::vector<std::string> to_remove;

  for (auto& [wp_name, entry] : _zone_log)
  {
    // robot name not found, or fleet name not found in the fleet state cache,
    // but we had previously seen this robot arrive at the zone waypoint,
    // so mark as suspect.
    auto fleet_it = _robot_positions.find(entry.fleet_name);
    if (fleet_it == _robot_positions.end())
    {
      if (entry.has_arrived)
        _mark_suspect(wp_name);
      continue;
    }

    auto robot_it = fleet_it->second.find(entry.robot_name);
    if (robot_it == fleet_it->second.end())
    {
      if (entry.has_arrived)
        _mark_suspect(wp_name);
      continue;
    }

    // Look up waypoint position
    const auto& pos = robot_it->second;
    double wp_x = 0, wp_y = 0;
    bool found = false;
    auto zone_it = _zones.find(entry.zone_name);
    if (zone_it != _zones.end())
    {
      auto pos_it = zone_it->second.waypoint_positions.find(wp_name);
      if (pos_it != zone_it->second.waypoint_positions.end())
      {
        wp_x = pos_it->second.first;
        wp_y = pos_it->second.second;
        found = true;
      }
    }

    if (!found)
    {
      // Waypoint not found in zone definition. This shouldn't happen,
      // but if it does, mark as suspect.
      if (entry.has_arrived)
        _mark_suspect(wp_name);
      continue;
    }

    const double dist = std::sqrt(
      std::pow(pos.x - wp_x, 2) + std::pow(pos.y - wp_y, 2));

    if (dist <= _stale_booking_distance_threshold)
    {
      entry.has_arrived = true;
      _suspect_bookings.erase(wp_name);
    }
    else if (entry.has_arrived)
    {
      // Was here, now gone
      _mark_suspect(wp_name);
    }
    // else: don't mark it stale yet, it hasn't had its chance to arrive

    if (entry.has_arrived)
    {
      auto suspect_it = _suspect_bookings.find(wp_name);
      if (suspect_it != _suspect_bookings.end())
      {
        const auto elapsed = (this->now() - suspect_it->second).seconds();
        if (elapsed >= _stale_booking_grace_period)
        {
          RCLCPP_WARN(this->get_logger(),
            "Stale booking detected: removing [%s] (robot [%s])",
            wp_name.c_str(), entry.robot_name.c_str());
          to_remove.push_back(wp_name);
        }
      }
    }
  }

  for (const auto& wp_name : to_remove)
    _remove_booking(wp_name, "stale_booking_check");
}

//==============================================================================
void Node::_remove_booking(
  const std::string& wp_name, const std::string& reason)
{
  auto it = _zone_log.find(wp_name);
  if (it == _zone_log.end())
    return;

  const auto entry = it->second;
  _erase_booking(it);
  _suspect_bookings.erase(wp_name);
  _erase_robot_position(entry.fleet_name, entry.robot_name);

  // Publish ZoneBookingRevoked.
  auto revoked = rmf_zone_msgs::msg::ZoneBookingRevoked();
  revoked.stamp = this->now();
  revoked.robot_name = entry.robot_name;
  revoked.fleet_name = entry.fleet_name;
  revoked.zone_name = entry.zone_name;
  revoked.assigned_waypoint_name = wp_name;
  revoked.reason = reason;
  _booking_revoked_pub->publish(revoked);

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
void Node::_mark_suspect(const std::string& wp_name)
{
  if (_suspect_bookings.find(wp_name) == _suspect_bookings.end())
  {
    _suspect_bookings[wp_name] = this->now();
    RCLCPP_WARN(this->get_logger(),
      "Booking for waypoint [%s] marked as suspect", wp_name.c_str());
  }
}

//==============================================================================
void Node::_erase_robot_position(
  const std::string& fleet_name, const std::string& robot_name)
{
  auto fleet_it = _robot_positions.find(fleet_name);
  if (fleet_it == _robot_positions.end())
    return;

  fleet_it->second.erase(robot_name);
  if (fleet_it->second.empty())
    _robot_positions.erase(fleet_it);
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

//==============================================================================
void Node::_insert_booking(const std::string& wp_name, ZoneLogEntry entry)
{
  _booked_robots_by_fleet[entry.fleet_name].insert(entry.robot_name);
  _zone_log[wp_name] = std::move(entry);
}

//==============================================================================
void Node::_erase_booking(
  std::unordered_map<std::string, ZoneLogEntry>::iterator it)
{
  const auto& entry = it->second;
  auto fleet_it = _booked_robots_by_fleet.find(entry.fleet_name);
  if (fleet_it != _booked_robots_by_fleet.end())
  {
    fleet_it->second.erase(entry.robot_name);
    if (fleet_it->second.empty())
      _booked_robots_by_fleet.erase(fleet_it);
  }
  _zone_log.erase(it);
}

} // namespace zone_supervisor
} // namespace rmf_fleet_adapter
