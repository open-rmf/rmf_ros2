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

#ifndef SRC__ZONE_MANAGER__ZONERESERVATIONCLIENT_HPP
#define SRC__ZONE_MANAGER__ZONERESERVATIONCLIENT_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_reservation_msgs/msg/claim_request.hpp>
#include <rmf_reservation_msgs/msg/flexible_time_request.hpp>
#include <rmf_reservation_msgs/msg/release_request.hpp>
#include <rmf_reservation_msgs/msg/reservation_allocation.hpp>
#include <rmf_reservation_msgs/msg/ticket.hpp>

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rmf_fleet_adapter {
namespace zone_manager {

//==============================================================================
/// Holds reservations on a zone's vertices, which are deliberately not
/// parking spots, so nothing outside the zone system can take one. Handing
/// one to a robot releases nothing, so it is never free for an instant.
class ZoneReservationClient
{
public:

  /// on_zone_changed fires when a zone's set of held vertices changes, so
  /// the manager can retry requests refused for want of a free vertex.
  ZoneReservationClient(
    rclcpp::Node& node,
    std::function<void(const std::string& zone)> on_zone_changed);

  /// Reserve every listed vertex of a zone from reservation node.
  void acquire(
    const std::string& zone,
    const std::vector<std::string>& vertex_names);

  /// Is this vertex held by us and free to assign?
  bool is_available(const std::string& zone, const std::string& vertex) const;

  /// The ticket for a vertex we have handed to a robot.
  std::optional<uint64_t> transferred_ticket_for(
    const std::string& zone, const std::string& vertex) const;

  /// Transfer a held vertex to a robot.
  bool transfer(
    const std::string& zone,
    const std::string& vertex,
    const std::string& fleet,
    const std::string& robot);

  /// What a handed-back vertex becomes. A robot lets go of a ticket for two
  /// different reasons and they want opposite answers.
  enum class HandbackDisposal
  {
    /// The robot is done with this zone, so the vertex is a spare again.
    Reclaim,
    /// The robot moved its booking elsewhere but is still parked here, so
    /// hold the vertex for nobody until an exit says it has gone.
    Freeze
  };

  /// Take a vertex back from a robot.
  bool confirm_handback(
    const std::string& zone,
    const std::string& vertex,
    const std::string& fleet,
    const std::string& robot,
    HandbackDisposal disposal);

  /// Relinquish one vertex. True when a reservation was actually handed
  /// back, false for a vertex we only had a queued claim on.
  bool release(const std::string& zone, const std::string& vertex);

  /// What release_zone gave up. A dropped claim was never granted, so
  /// somebody outside the zone system is most likely on that vertex.
  struct ZonePoolRelease
  {
    std::size_t released = 0;
    std::size_t dropped_claims = 0;
  };

  /// Release a zone's free spares. A transferred or frozen vertex has a
  /// robot on it, so it stays.
  ZonePoolRelease release_zone(const std::string& zone);

  /// Is a reservation node present?
  bool reservation_node_present() const;

  /// Do we have any holding at all for this zone, in any state?
  bool has_holdings(const std::string& zone) const;

  /// Every vertex of this zone a robot has taken from us. These are the
  /// only ones we cannot get back on our own.
  std::vector<std::string> transferred_vertices(const std::string& zone) const;

  /// Every zone we hold something in, in any state.
  std::unordered_set<std::string> zones_with_holdings() const;

  /// Still waiting on the reservation node for any vertex of this zone?
  bool acquisition_in_flight(const std::string& zone) const;

private:

  /// Drop our record of a vertex whose ticket is not ours to release.
  void _disown(const std::string& vertex);

  struct Holding
  {
    std::string zone;
    std::string vertex;
    uint64_t request_id = 0;
    std::optional<uint64_t> ticket_id;

    enum class State
    {
      /// Request published, no ticket back yet.
      Requested,
      /// Ticket claimed and allocated to us. Ours to assign.
      Held,
      /// Ours again, but the robot that gave it back is still parked on it.
      /// Assignable to nobody, and the idle sweep leaves it alone, until an
      /// exit says the robot has gone.
      Frozen,
      /// The robot owns it now. Not ours to release.
      Transferred
    };

    State state = State::Requested;

    struct RobotId
    {
      std::string fleet;
      std::string name;

      bool operator==(const RobotId& other) const
      {
        return fleet == other.fleet && name == other.name;
      }

      bool operator!=(const RobotId& other) const
      {
        return !(*this == other);
      }
    };

    std::optional<RobotId> transferred_to;
  };

  void _on_ticket(const rmf_reservation_msgs::msg::Ticket& msg);
  void _on_allocation(
    const rmf_reservation_msgs::msg::ReservationAllocation& msg);

  Holding* _find(const std::string& zone, const std::string& vertex);
  const Holding* _find(
    const std::string& zone, const std::string& vertex) const;

  Holding* _find_by_request_id(uint64_t request_id);
  Holding* _find_by_ticket_id(uint64_t ticket_id);

  rclcpp::Node& _node;
  std::function<void(const std::string&)> _on_zone_changed;

  // the holding waypoints map, keyed by vertex name
  std::unordered_map<std::string, Holding> _holdings;

  // Identity used for our own bookings
  std::string _robot_name = "zone_manager";
  std::string _fleet_name = "rmf_zones";
  uint64_t _next_request_id = 0;

  rclcpp::Publisher<rmf_reservation_msgs::msg::FlexibleTimeRequest>::SharedPtr
    _request_pub;
  rclcpp::Publisher<rmf_reservation_msgs::msg::ClaimRequest>::SharedPtr
    _claim_pub;
  rclcpp::Publisher<rmf_reservation_msgs::msg::ReleaseRequest>::SharedPtr
    _release_pub;
  rclcpp::Subscription<rmf_reservation_msgs::msg::Ticket>::SharedPtr
    _ticket_sub;
  rclcpp::Subscription<rmf_reservation_msgs::msg::ReservationAllocation>::
  SharedPtr _allocation_sub;
};

} // namespace zone_manager
} // namespace rmf_fleet_adapter

#endif // SRC__ZONE_MANAGER__ZONERESERVATIONCLIENT_HPP
