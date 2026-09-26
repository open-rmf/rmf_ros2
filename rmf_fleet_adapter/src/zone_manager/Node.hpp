#ifndef SRC__ZONE_MANAGER__NODE_HPP
#define SRC__ZONE_MANAGER__NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_zone_msgs/msg/zone_request.hpp>
#include <rmf_zone_msgs/msg/zone_state.hpp>
#include <rmf_zone_msgs/msg/zone_booking.hpp>
#include <rmf_zone_msgs/msg/zone_rejection.hpp>
#include <rmf_zone_msgs/msg/zone_booking_revoked.hpp>
#include <rmf_zone_msgs/msg/zone_manual_release.hpp>

#include <rmf_building_map_msgs/msg/graph.hpp>

#include "ZoneReservationClient.hpp"

#include <memory>

#include <optional>
#include <unordered_map>

namespace rmf_fleet_adapter {
namespace zone_manager {

//==============================================================================
class Node : public rclcpp::Node
{
public:

  Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  ~Node();

private:

  struct ZoneWaypointInfo
  {
    std::string name;
    uint8_t priority = 0;
    std::string group;
  };

  struct ZoneInfo
  {
    std::vector<ZoneWaypointInfo> waypoints;  // sorted by priority ascending
  };

  // Every zone the nav graph declares, keyed by zone name.
  std::unordered_map<std::string, ZoneInfo> _zones;

  bool _same_vertices(const ZoneInfo& a, const ZoneInfo& b) const;

  // Zone log
  struct ZoneLogEntry
  {
    std::string robot_name;
    std::string fleet_name;
    std::string zone_name;
    rclcpp::Time assigned_at;
    std::string request_id;
    bool has_orientation = false;
    double orientation = 0.0;

    // Set by an ARRIVED request. The robot is on its waypoint, so the pool
    // no longer has to stay whole on its account.
    bool arrived = false;

  };

  // Zone booking ledger, keyed by assigned waypoint name.
  std::unordered_map<std::string, ZoneLogEntry> _zone_log;

  rclcpp::Subscription<rmf_building_map_msgs::msg::Graph>::SharedPtr
    _nav_graphs_sub;
  rclcpp::Subscription<rmf_zone_msgs::msg::ZoneRequest>::SharedPtr
    _zone_request_sub;
  rclcpp::Subscription<rmf_zone_msgs::msg::ZoneManualRelease>::SharedPtr
    _manual_release_sub;

  rclcpp::Publisher<rmf_zone_msgs::msg::ZoneState>::SharedPtr _state_pub;
  rclcpp::Publisher<rmf_zone_msgs::msg::ZoneBookingRevoked>::SharedPtr
    _booking_revoked_pub;

  // Not ready until we receive a nav graph. Requests are queued until then.
  bool _ready = false;
  std::vector<rmf_zone_msgs::msg::ZoneRequest::SharedPtr> _pending_requests;

  struct PendingEntry
  {
    rmf_zone_msgs::msg::ZoneRequest::SharedPtr request;

    /// When to stop waiting and refuse.
    rclcpp::Time deadline;

    /// Do not select before this, while any vertex is still outstanding.
    /// Allocations arrive one at a time in no particular order, so selecting
    /// early lets arrival order decide instead of priority or the hints.
    rclcpp::Time settle_by;
  };

  // Parked ENTRY requests, keyed by zone name.
  std::unordered_map<std::string, std::vector<PendingEntry>> _pending_entries;

  rclcpp::TimerBase::SharedPtr _pending_entry_timer;

  // How long a parked ENTRY request waits before being refused.
  std::chrono::seconds _entry_request_timeout{30};

  // How long to let a zone's remaining reservations arrive before selecting
  // from the ones already in hand.
  std::chrono::milliseconds _reservation_settle_time{1000};

  // How long a zone must sit unwanted before its reservations go back.
  std::chrono::seconds _zone_pool_idle_timeout{30};

  // When each zone went idle, keyed by zone name. Absent means in demand.
  std::unordered_map<std::string, rclcpp::Time> _zone_idle_since;

  std::unique_ptr<ZoneReservationClient> _reservation_client;

  void _on_nav_graphs(
    const rmf_building_map_msgs::msg::Graph::SharedPtr msg);
  void _on_zone_request(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg);
  void _on_manual_release(
    const rmf_zone_msgs::msg::ZoneManualRelease::SharedPtr msg);

  /// Re-select for a robot arriving at the zone boundary, which may hand it
  /// a better vertex than the one it prebooked. Falls back to a prebooking
  /// if it turns out to hold nothing.
  void _process_entry(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg);

  /// Claim a vertex for a robot that is about to set off, or park the
  /// request if the zone cannot answer yet.
  ///
  /// \param[in] deadline
  ///   when to give up and refuse. Unset on first arrival.
  /// \param[in] settle_by
  ///   when selection may begin. Unset on first arrival.
  void _process_prebooking(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg,
    std::optional<rclcpp::Time> deadline = std::nullopt,
    std::optional<rclcpp::Time> settle_by = std::nullopt);
  void _process_exit(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg);

  /// Record that a robot has reached the waypoint it booked. Answers
  /// nothing and changes no booking.
  void _process_arrived(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg);

  /// A robot confirming it stopped holding a vertex's ticket without
  /// releasing it, after being re-assigned inside this zone.
  void _process_handback(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg);

  void _finalize_entry(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg,
    std::unordered_map<std::string, ZoneLogEntry>::iterator held);

  void _park_entry_request(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg,
    rclcpp::Time deadline,
    rclcpp::Time settle_by);

  /// Re-run every request parked against this zone.
  void _retry_pending_entries(const std::string& zone_name);

  /// Refuse the parked requests whose deadline has passed.
  void _expire_pending_entries();

  /// Retry the parked requests that can be answered,
  /// then expire the rest.
  void _tick_pending_entries();

  void _drop_pending_entry(
    const std::string& robot_name,
    const std::string& fleet_name,
    const std::string& zone_name);

  void _on_zone_pool_changed(const std::string& zone_name);

  bool _zone_pool_is_idle(const std::string& zone_name) const;

  /// Give back the reservations of every zone that has been idle long enough.
  void _sweep_idle_zone_pools();

  /// The zone's vertex names, for acquiring reservations over.
  std::vector<std::string> _zone_vertex_names(
    const std::string& zone_name) const;

  rmf_zone_msgs::msg::ZoneState _build_state_msg();
  void _publish_state();
  void _publish_state_with_rejection(
    const std::string& robot_name,
    const std::string& fleet_name,
    const std::string& request_id,
    const std::string& zone_name,
    const std::string& reason);

  /// Answer a robot that asked for nothing. Carried on one publish only,
  /// since no booking is made and there is nothing to keep advertising.
  void _publish_state_with_proceed(
    const std::string& robot_name,
    const std::string& fleet_name,
    const std::string& request_id,
    const std::string& zone_name);

  void _remove_booking(
    const std::string& waypoint_name,
    const std::string& reason);
  void _remove_booking_for_robot(
    const std::string& robot_name,
    const std::string& fleet_name,
    const std::string& zone_name,
    const std::string& reason);

  std::unordered_map<std::string, ZoneLogEntry>::iterator
  _find_booking_for(
    const std::string& robot_name,
    const std::string& fleet_name,
    const std::string& zone_name);

  struct SelectionResult
  {
    bool success = false;
    ZoneWaypointInfo selected;
    std::string rejection_reason;

    /// Nothing assignable yet, but reservations are still arriving.
    bool awaiting_reservations = false;
  };

  /// \param[in] held_vertex
  ///   The vertex this robot already holds, kept as a candidate so that
  ///   finding nothing better keeps it rather than refusing it.
  SelectionResult _select_waypoint(
    const std::string& zone_name,
    const rmf_zone_msgs::msg::ZoneModifiers& modifiers,
    const std::string& held_vertex = "");
};

} // namespace zone_manager
} // namespace rmf_fleet_adapter

#endif // SRC__ZONE_MANAGER__NODE_HPP
