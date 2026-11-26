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

#ifndef SRC__LANE_CLOSURE_SUPERVISOR__LANEHOLDS_HPP
#define SRC__LANE_CLOSURE_SUPERVISOR__LANEHOLDS_HPP

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace rmf_fleet_adapter {
namespace lane_closure_supervisor {

//==============================================================================
/// What a request covers: a fleet (empty robot_name) or one robot
struct RequestKey
{
  std::string fleet_name;
  std::string robot_name;

  bool operator<(const RequestKey& other) const
  {
    return std::tie(fleet_name, robot_name)
      < std::tie(other.fleet_name, other.robot_name);
  }
};

//==============================================================================
/// One robot: fleet and robot name (robot_name always set)
struct RobotId
{
  std::string fleet_name;
  std::string robot_name;

  bool operator<(const RobotId& other) const
  {
    return std::tie(fleet_name, robot_name)
      < std::tie(other.fleet_name, other.robot_name);
  }
};

//==============================================================================
/// "fleet [f]" or "robot [f/r]", for log messages
std::string to_string(const RequestKey& key);

/// "robot [f/r]", for log messages
std::string to_string(const RobotId& key);

//==============================================================================
/// Lane changes for one robot
struct Change
{
  /// Lanes now closed
  std::vector<uint64_t> newly_closed;

  /// Lanes now open
  std::vector<uint64_t> newly_opened;

  bool empty() const
  {
    return newly_closed.empty() && newly_opened.empty();
  }
};

//==============================================================================
/// Result of update()
struct Update
{
  /// False if ignored: older than the requester's latest message for the
  /// fleet, or the same time with a different key or lanes
  bool accepted = false;

  /// Changes per robot
  std::map<RobotId, Change> changes;
};

//==============================================================================
/// Lanes to send again
struct Resend
{
  /// Held but reported open
  std::vector<uint64_t> close;

  /// Released but reported closed
  std::vector<uint64_t> open;

  bool empty() const
  {
    return close.empty() && open.empty();
  }
};

//==============================================================================
struct Holder
{
  std::string requester_id;

  /// Time of its list that applies to this robot. Unique per requester and
  /// fleet (one timeline), so it also tells fleet-wide and robot lists apart.
  int64_t request_time_ns;

  bool operator==(const Holder& other) const
  {
    return std::tie(requester_id, request_time_ns)
      == std::tie(other.requester_id, other.request_time_ns);
  }
};

/// Lane -> holders
using LaneHolders = std::map<uint64_t, std::vector<Holder>>;

/// Robot -> lane -> holders
using Sessions = std::map<RobotId, LaneHolders>;

//==============================================================================
/// Each requester's latest lanes per request key, applied to known robots.
/// For each requester and robot, the later of its fleet-wide list and its list
/// for that robot applies. A lane is closed for a robot while any requester's
/// applied list holds it. No ROS code.
class LaneHolds
{
public:

  /// Store a requester's full list for a key. Latest message wins, on one
  /// timeline per requester and fleet (fleet-wide and robot requests alike):
  /// a later time replaces the key's list; the same time, key and lanes as the
  /// requester's latest message is a retry; anything else is ignored. The time
  /// is kept after an empty list. A request for an unknown robot is stored but
  /// changes nothing.
  Update update(
    const std::string& requester_id,
    const RequestKey& key,
    const std::vector<uint64_t>& closed_lanes,
    int64_t request_time_ns);

  /// Add a robot from /fleet_states. Sends nothing.
  ///
  /// \return false if already known
  bool add_robot(
    const std::string& fleet_name,
    const std::string& robot_name);

  /// Confirm pending reopens the fleet reports open
  void confirm_reopens(
    const RobotId& key,
    const std::set<uint64_t>& reported_closed);

  /// What to send again, given the fleet's last report
  Resend reconcile(
    const RobotId& key,
    const std::set<uint64_t>& reported_closed) const;

  /// Held lanes and holders per robot, for the heartbeat: each requester's
  /// applied list. Covers known robots and robots named in robot requests.
  Sessions sessions() const;

  /// Held lanes for a robot
  std::set<uint64_t> held_lanes(const RobotId& key) const;

  /// Known robots of a fleet
  std::set<std::string> robots(const std::string& fleet_name) const;

private:
  struct Latest
  {
    int64_t request_time_ns;
    std::set<uint64_t> lanes;
  };

  bool _known(const RobotId& key) const;

  /// A requester's latest message for a key, or nullptr
  const Latest* _find(
    const RequestKey& key,
    const std::string& requester_id) const;

  /// Requesters with a list for a robot, fleet-wide or its own
  std::set<std::string> _requesters(const RobotId& robot) const;

  /// A requester's list that applies to a robot, or nullptr
  const Latest* _applied(
    const std::string& requester_id,
    const RobotId& robot) const;

  /// Known robots a request key covers
  std::vector<RobotId> _covered(const RequestKey& key) const;

  /// Request key -> requester -> latest message (never erased)
  std::map<RequestKey, std::map<std::string, Latest>> _requests;

  /// A requester's latest accepted message for a fleet, any key
  struct LastMessage
  {
    int64_t request_time_ns;
    std::string robot_name;
    std::set<uint64_t> lanes;
  };

  /// (requester, fleet) -> latest accepted message (never erased)
  std::map<std::pair<std::string, std::string>, LastMessage> _timelines;

  /// Fleet -> known robots
  std::map<std::string, std::set<std::string>> _robots;

  /// Released lanes not yet reported open
  std::map<RobotId, std::set<uint64_t>> _pending_open;
};

} // namespace lane_closure_supervisor
} // namespace rmf_fleet_adapter

#endif // SRC__LANE_CLOSURE_SUPERVISOR__LANEHOLDS_HPP
