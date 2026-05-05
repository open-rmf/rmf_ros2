#ifndef SRC__ZONE_SUPERVISOR__NODE_HPP
#define SRC__ZONE_SUPERVISOR__NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_zone_msgs/msg/zone_request.hpp>
#include <rmf_zone_msgs/msg/zone_state.hpp>
#include <rmf_zone_msgs/msg/zone_booking.hpp>
#include <rmf_zone_msgs/msg/zone_rejection.hpp>
#include <rmf_zone_msgs/msg/zone_booking_revoked.hpp>
#include <rmf_zone_msgs/msg/zone_manual_release.hpp>

#include <rmf_building_map_msgs/msg/graph.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <unordered_map>
#include <unordered_set>

namespace rmf_fleet_adapter {
namespace zone_supervisor {

//==============================================================================
class Node : public rclcpp::Node
{
public:

  Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:

  // Zones registry
  struct ZoneWaypointInfo
  {
    std::string name;
    float x;
    float y;
    uint8_t priority;
    std::string group;
  };

  struct ZoneInfo
  {
    std::vector<ZoneWaypointInfo> waypoints;  // sorted by priority ascending
    std::unordered_map<std::string, std::pair<float, float>>
      waypoint_positions; // wp_name -> (x, y)
  };

  std::unordered_map<std::string, ZoneInfo> _zones;

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

    // Set once robot is seen near the waypoint. Stale checks only apply
    // after arrival.
    bool has_arrived = false;
  };

  // Keyed by assigned waypoint name (presume zone waypoint name are
  // unique across fleets).
  std::unordered_map<std::string, ZoneLogEntry> _zone_log;

  // fleet_name -> set of robot_names with active bookings.
  std::unordered_map<std::string,
    std::unordered_set<std::string>> _booked_robots_by_fleet;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr _callback_group;

  // Subscribers
  rclcpp::Subscription<rmf_building_map_msgs::msg::Graph>::SharedPtr
    _nav_graphs_sub;
  rclcpp::Subscription<rmf_zone_msgs::msg::ZoneRequest>::SharedPtr
    _zone_request_sub;
  rclcpp::Subscription<rmf_zone_msgs::msg::ZoneManualRelease>::SharedPtr
    _manual_release_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
    _fleet_states_sub;

  // Publishers
  rclcpp::Publisher<rmf_zone_msgs::msg::ZoneState>::SharedPtr _state_pub;
  rclcpp::Publisher<rmf_zone_msgs::msg::ZoneBookingRevoked>::SharedPtr
    _booking_revoked_pub;

  // Timers
  rclcpp::TimerBase::SharedPtr _stale_booking_timer;

  // Fleet state cache
  struct RobotPosition
  {
    double x;
    double y;
  };

  std::unordered_map<std::string,
    std::unordered_map<std::string, RobotPosition>> _robot_positions;

  // Parameters
  double _stale_booking_check_interval;
  double _stale_booking_distance_threshold;
  double _stale_booking_grace_period;

  // Not ready until we receive a nav graph. Requests are queued until then.
  bool _ready = false;
  std::vector<rmf_zone_msgs::msg::ZoneRequest::SharedPtr> _pending_requests;

  // wp_name -> time first suspected stale. Revoked after grace period.
  std::unordered_map<std::string, rclcpp::Time> _suspect_bookings;

  // Callbacks
  void _on_nav_graphs(
    const rmf_building_map_msgs::msg::Graph::SharedPtr msg);
  void _on_zone_request(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg);
  void _on_manual_release(
    const rmf_zone_msgs::msg::ZoneManualRelease::SharedPtr msg);
  void _on_fleet_state(
    const rmf_fleet_msgs::msg::FleetState::SharedPtr msg);
  void _on_stale_booking_check();

  // Core logic
  void _process_entry(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg);
  void _process_exit(
    const rmf_zone_msgs::msg::ZoneRequest::SharedPtr msg);

  rmf_zone_msgs::msg::ZoneState _build_state_msg();
  void _publish_state();
  void _publish_state_with_rejection(
    const std::string& robot_name,
    const std::string& fleet_name,
    const std::string& request_id,
    const std::string& zone_name,
    const std::string& reason);

  void _remove_booking(
    const std::string& waypoint_name,
    const std::string& reason);
  void _remove_booking_for_robot(
    const std::string& robot_name,
    const std::string& fleet_name,
    const std::string& zone_name,
    const std::string& reason);

  void _mark_suspect(const std::string& wp_name);

  void _erase_robot_position(
    const std::string& fleet_name, const std::string& robot_name);

  void _insert_booking(const std::string& wp_name, ZoneLogEntry entry);
  void _erase_booking(
    std::unordered_map<std::string, ZoneLogEntry>::iterator it);

  // Finds an active booking
  std::unordered_map<std::string, ZoneLogEntry>::iterator
  _find_booking_for(
    const std::string& robot_name,
    const std::string& fleet_name,
    const std::string& zone_name);

  struct SelectionResult
  {
    bool success;
    ZoneWaypointInfo selected;
    std::string rejection_reason;
  };

  SelectionResult _select_waypoint(
    const std::string& zone_name,
    const rmf_zone_msgs::msg::ZoneModifiers& modifiers);
};

} // namespace zone_supervisor
} // namespace rmf_fleet_adapter

#endif // SRC__ZONE_SUPERVISOR__NODE_HPP
