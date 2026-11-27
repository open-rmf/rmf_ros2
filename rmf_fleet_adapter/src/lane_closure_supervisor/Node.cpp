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

#include "Node.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <algorithm>
#include <sstream>

namespace rmf_fleet_adapter {
namespace lane_closure_supervisor {

namespace {
//==============================================================================
constexpr int64_t NsPerSec = 1000000000LL;

//==============================================================================
const std::set<uint64_t> NoLanes;

//==============================================================================
template<typename Lanes>
std::string to_string(const Lanes& lanes)
{
  std::stringstream ss;
  ss << "{";
  for (const auto lane : lanes)
    ss << " " << lane;
  ss << " }";
  return ss.str();
}

//==============================================================================
int64_t to_ns(const builtin_interfaces::msg::Time& time)
{
  return static_cast<int64_t>(time.sec) * NsPerSec
    + static_cast<int64_t>(time.nanosec);
}

//==============================================================================
/// Never throws, unlike rclcpp::Time for negative times
builtin_interfaces::msg::Time to_msg(const int64_t ns)
{
  int64_t sec = ns / NsPerSec;
  int64_t nanosec = ns % NsPerSec;
  if (nanosec < 0)
  {
    --sec;
    nanosec += NsPerSec;
  }

  builtin_interfaces::msg::Time time;
  time.sec = static_cast<int32_t>(sec);
  time.nanosec = static_cast<uint32_t>(nanosec);
  return time;
}

//==============================================================================
/// Lanes to send to one robot
struct Outgoing
{
  std::set<uint64_t> close;
  std::set<uint64_t> open;
};

} // anonymous namespace

//==============================================================================
Node::Node(const rclcpp::NodeOptions& options)
: rclcpp::Node("lane_closure_supervisor", options)
{
  const auto request_qos =
    rclcpp::SystemDefaultsQoS().reliable().keep_last(100).durability_volatile();

  // Best-effort matches any publisher
  const auto fleet_state_qos =
    rclcpp::SystemDefaultsQoS().best_effort().keep_last(100)
    .durability_volatile();

  // Get each fleet's latest report on startup
  const auto states_qos =
    rclcpp::SystemDefaultsQoS().reliable().keep_last(100).transient_local();

  // Deep enough for one message per robot
  const auto command_qos =
    rclcpp::SystemDefaultsQoS().reliable().keep_last(100).durability_volatile();

  // Late joiners get the latest
  const auto heartbeat_qos =
    rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local();

  _robot_lane_request_pub = create_publisher<RobotLaneRequest>(
    RobotLaneClosureRequestTopicName, command_qos);

  _heartbeat_pub = create_publisher<Heartbeat>(
    LaneClosureSupervisorHeartbeatTopicName, heartbeat_qos);

  _closure_request_sub = create_subscription<LaneClosureRequest>(
    LaneClosureSupervisorRequestTopicName, request_qos,
    [this](LaneClosureRequest::UniquePtr msg)
    {
      _closure_request_update(*msg);
    });

  _fleet_state_sub = create_subscription<FleetState>(
    FleetStateTopicName, fleet_state_qos,
    [this](FleetState::UniquePtr msg)
    {
      _fleet_state_update(*msg);
    });

  _lane_states_sub = create_subscription<LaneStates>(
    LaneStatesTopicName, states_qos,
    [this](LaneStates::UniquePtr msg)
    {
      _lane_states_update(*msg);
    });

  _publish_heartbeat();
}

//==============================================================================
void Node::_closure_request_update(const LaneClosureRequest& msg)
{
  if (msg.requester_id.empty() || msg.fleet_name.empty())
  {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring lane closure request with an empty requester_id [%s] or "
      "fleet_name [%s]",
      msg.requester_id.c_str(), msg.fleet_name.c_str());
    return;
  }

  // Could overflow the heartbeat's seconds
  if (msg.request_time.nanosec >= NsPerSec)
  {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring lane closure request from [%s]: request_time.nanosec must be "
      "below 1000000000 (got %u)",
      msg.requester_id.c_str(), msg.request_time.nanosec);
    return;
  }

  const RequestKey key{msg.fleet_name, msg.robot_name};
  const auto update = _lane_holds.update(
    msg.requester_id, key, msg.closed_lanes, to_ns(msg.request_time));

  if (!update.accepted)
  {
    RCLCPP_INFO(
      get_logger(),
      "Ignoring lane closure request from [%s] for %s (lanes %s): its "
      "request_time is older than its latest request for this fleet (fleet-"
      "wide or for any robot), or the same with a different robot or list of "
      "lanes. Use a later request_time for every new message.",
      msg.requester_id.c_str(), to_string(key).c_str(),
      to_string(msg.closed_lanes).c_str());
    return;
  }

  std::map<RobotId, Outgoing> outgoing;
  for (const auto& [robot, change] : update.changes)
  {
    auto& out = outgoing[robot];
    out.close.insert(change.newly_closed.begin(), change.newly_closed.end());
    out.open.insert(change.newly_opened.begin(), change.newly_opened.end());
  }

  // Re-send what present robots haven't applied. Only once the fleet has
  // published LaneStates: it lists closed lanes and every other lane is open,
  // so with no report we can't tell what's applied.
  std::size_t resent = 0;
  if (_reported_closed.count(key.fleet_name) > 0)
  {
    std::vector<RobotId> covered;
    const auto present_it = _present_robots.find(key.fleet_name);
    if (present_it != _present_robots.end())
    {
      for (const auto& robot : present_it->second)
      {
        if (key.robot_name.empty() || key.robot_name == robot)
          covered.push_back(RobotId{key.fleet_name, robot});
      }
    }

    for (const auto& robot : covered)
    {
      auto resend = _lane_holds.reconcile(robot, _reported_closed_for(robot));

      // reconcile() also returns this request's own changes, since the fleet
      // hasn't applied them yet. Drop those so a robot counts as re-sent only
      // for lanes left over from earlier requests. out is a set, so this
      // changes the count, not the message.
      auto& out = outgoing[robot];
      const auto drop = [](std::vector<uint64_t>& lanes,
        const std::set<uint64_t>& sent)
      {
        lanes.erase(
          std::remove_if(lanes.begin(), lanes.end(),
          [&](uint64_t lane) { return sent.count(lane) > 0; }),
          lanes.end());
      };
      drop(resend.close, out.close);
      drop(resend.open, out.open);

      // Nothing left over: the robot has applied everything before this
      // request
      if (resend.empty())
        continue;

      // Send the leftovers with this request's changes. For a pure retry
      // (no changes), this is the only thing in the robot's message.
      ++resent;
      out.close.insert(resend.close.begin(), resend.close.end());
      out.open.insert(resend.open.begin(), resend.open.end());
    }
  }

  // One INFO line per request
  if (!update.changes.empty() || resent > 0)
  {
    RCLCPP_INFO(
      get_logger(),
      "[%s] needs lanes %s closed for %s: %zu robot(s) changed, %zu robot(s) "
      "sent again",
      msg.requester_id.c_str(), to_string(msg.closed_lanes).c_str(),
      to_string(key).c_str(), update.changes.size(), resent);
  }

  for (const auto& [robot, out] : outgoing)
  {
    if (out.close.empty() && out.open.empty())
      continue;

    _send(
      robot,
      std::vector<uint64_t>(out.open.begin(), out.open.end()),
      std::vector<uint64_t>(out.close.begin(), out.close.end()));
  }

  _publish_heartbeat();
}

//==============================================================================
void Node::_fleet_state_update(const FleetState& msg)
{
  if (msg.name.empty())
    return;

  auto& present = _present_robots[msg.name];
  present.clear();
  for (const auto& robot : msg.robots)
  {
    if (!robot.name.empty())
      present.insert(robot.name);
  }

  // Requesters wait for robots: send nothing
  for (const auto& robot : present)
  {
    if (_lane_holds.add_robot(msg.name, robot))
    {
      RCLCPP_INFO(
        get_logger(), "New %s",
        to_string(RobotId{msg.name, robot}).c_str());
    }
  }
}

//==============================================================================
void Node::_lane_states_update(const LaneStates& msg)
{
  if (msg.fleet_name.empty())
  {
    RCLCPP_WARN(get_logger(), "Ignoring LaneStates with an empty fleet_name");
    return;
  }

  auto& reported = _reported_closed[msg.fleet_name];
  reported.clear();
  for (const auto& robot : msg.robot_lane_states)
  {
    if (robot.robot_name.empty())
      continue;

    reported[robot.robot_name] =
      std::set<uint64_t>(robot.closed_lanes.begin(), robot.closed_lanes.end());
  }

  // Unlisted robot: no closures. Never sends.
  for (const auto& robot : _lane_holds.robots(msg.fleet_name))
  {
    const RobotId key{msg.fleet_name, robot};
    _lane_holds.confirm_reopens(key, _reported_closed_for(key));
  }
}

//==============================================================================
const std::set<uint64_t>& Node::_reported_closed_for(const RobotId& key) const
{
  const auto fleet_it = _reported_closed.find(key.fleet_name);
  if (fleet_it == _reported_closed.end())
    return NoLanes;

  const auto robot_it = fleet_it->second.find(key.robot_name);
  return robot_it != fleet_it->second.end() ? robot_it->second : NoLanes;
}

//==============================================================================
void Node::_send(
  const RobotId& key,
  const std::vector<uint64_t>& open_lanes,
  const std::vector<uint64_t>& close_lanes)
{
  // Fleet adapters reject a lane in both lists
  for (const auto lane : open_lanes)
  {
    if (std::find(close_lanes.begin(), close_lanes.end(), lane)
      != close_lanes.end())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Not sending to %s: lane %lu would be both opened and closed "
        "(open %s, close %s). This is a bug in the supervisor.",
        to_string(key).c_str(), static_cast<unsigned long>(lane),
        to_string(open_lanes).c_str(), to_string(close_lanes).c_str());
      return;
    }
  }

  RobotLaneRequest request;
  request.fleet_name = key.fleet_name;
  request.robot_name = key.robot_name;
  request.open_lanes = open_lanes;
  request.close_lanes = close_lanes;
  _robot_lane_request_pub->publish(request);
}

//==============================================================================
void Node::_publish_heartbeat()
{
  Heartbeat msg;
  for (const auto& [key, lanes] : _lane_holds.sessions())
  {
    for (const auto& [lane, holders] : lanes)
    {
      rmf_fleet_msgs::msg::LaneClosureSessions sessions;
      sessions.fleet_name = key.fleet_name;
      sessions.robot_name = key.robot_name;
      sessions.lane_index = lane;
      for (const auto& holder : holders)
      {
        rmf_fleet_msgs::msg::LaneClosureSession s;
        s.requester_id = holder.requester_id;
        s.request_time = to_msg(holder.request_time_ns);
        sessions.sessions.push_back(std::move(s));
      }

      msg.all_sessions.push_back(std::move(sessions));
    }
  }

  _heartbeat_pub->publish(msg);
}

} // namespace lane_closure_supervisor
} // namespace rmf_fleet_adapter
