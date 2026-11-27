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

#include "LaneHolds.hpp"

#include <algorithm>
#include <iterator>

namespace rmf_fleet_adapter {
namespace lane_closure_supervisor {

namespace {
//==============================================================================
std::vector<uint64_t> difference(
  const std::set<uint64_t>& a,
  const std::set<uint64_t>& b)
{
  std::vector<uint64_t> result;
  std::set_difference(
    a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(result));
  return result;
}

} // anonymous namespace

//==============================================================================
std::string to_string(const RequestKey& key)
{
  if (key.robot_name.empty())
    return "fleet [" + key.fleet_name + "]";

  return "robot [" + key.fleet_name + "/" + key.robot_name + "]";
}

//==============================================================================
std::string to_string(const RobotId& key)
{
  return "robot [" + key.fleet_name + "/" + key.robot_name + "]";
}

//==============================================================================
Update LaneHolds::update(
  const std::string& requester_id,
  const RequestKey& key,
  const std::vector<uint64_t>& closed_lanes,
  const int64_t request_time_ns)
{
  Update result;

  // Compare as sets
  std::set<uint64_t> lanes(closed_lanes.begin(), closed_lanes.end());

  // One timeline per requester and fleet: only a later time, or a retry of
  // the latest message, is accepted
  const auto timeline_key = std::make_pair(requester_id, key.fleet_name);
  const auto last_it = _timelines.find(timeline_key);
  if (last_it != _timelines.end())
  {
    const auto& last = last_it->second;
    if (request_time_ns < last.request_time_ns
      || (request_time_ns == last.request_time_ns
      && (key.robot_name != last.robot_name || lanes != last.lanes)))
    {
      return result;
    }
  }

  const auto covered = _covered(key);
  std::map<RobotId, std::set<uint64_t>> before;
  for (const auto& robot : covered)
    before[robot] = held_lanes(robot);

  _timelines[timeline_key] =
    LastMessage{request_time_ns, key.robot_name, lanes};
  _requests[key][requester_id] = Latest{request_time_ns, std::move(lanes)};
  result.accepted = true;

  for (const auto& robot : covered)
  {
    const auto after = held_lanes(robot);
    Change change;
    change.newly_closed = difference(after, before[robot]);
    change.newly_opened = difference(before[robot], after);
    if (change.empty())
      continue;

    auto& pending = _pending_open[robot];
    for (const auto lane : change.newly_closed)
      pending.erase(lane);
    for (const auto lane : change.newly_opened)
      pending.insert(lane);
    if (pending.empty())
      _pending_open.erase(robot);

    result.changes[robot] = std::move(change);
  }

  return result;
}

//==============================================================================
bool LaneHolds::add_robot(
  const std::string& fleet_name,
  const std::string& robot_name)
{
  return _robots[fleet_name].insert(robot_name).second;
}

//==============================================================================
void LaneHolds::confirm_reopens(
  const RobotId& key,
  const std::set<uint64_t>& reported_closed)
{
  const auto pending_it = _pending_open.find(key);
  if (pending_it == _pending_open.end())
    return;

  // Reported open: confirmed
  auto& pending = pending_it->second;
  for (auto it = pending.begin(); it != pending.end(); )
  {
    if (reported_closed.count(*it) == 0)
      it = pending.erase(it);
    else
      ++it;
  }

  if (pending.empty())
    _pending_open.erase(pending_it);
}

//==============================================================================
Resend LaneHolds::reconcile(
  const RobotId& key,
  const std::set<uint64_t>& reported_closed) const
{
  Resend resend;

  // Held but reported open
  for (const auto lane : held_lanes(key))
  {
    if (reported_closed.count(lane) == 0)
      resend.close.push_back(lane);
  }

  // Released but reported closed
  const auto pending_it = _pending_open.find(key);
  if (pending_it != _pending_open.end())
  {
    for (const auto lane : pending_it->second)
    {
      if (reported_closed.count(lane) > 0)
        resend.open.push_back(lane);
    }
  }

  return resend;
}

//==============================================================================
Sessions LaneHolds::sessions() const
{
  // Known robots, and robots named in robot requests
  std::set<RobotId> robots;
  for (const auto& [fleet_name, names] : _robots)
  {
    for (const auto& name : names)
      robots.insert(RobotId{fleet_name, name});
  }

  for (const auto& [key, _] : _requests)
  {
    if (!key.robot_name.empty())
      robots.insert(RobotId{key.fleet_name, key.robot_name});
  }

  Sessions result;
  for (const auto& robot : robots)
  {
    for (const auto& requester_id : _requesters(robot))
    {
      const auto* applied = _applied(requester_id, robot);
      for (const auto lane : applied->lanes)
        result[robot][lane].push_back(
          Holder{requester_id, applied->request_time_ns});
    }
  }

  return result;
}

//==============================================================================
std::set<uint64_t> LaneHolds::held_lanes(const RobotId& key) const
{
  std::set<uint64_t> result;
  if (!_known(key))
    return result;

  for (const auto& requester_id : _requesters(key))
  {
    const auto* applied = _applied(requester_id, key);
    result.insert(applied->lanes.begin(), applied->lanes.end());
  }

  return result;
}

//==============================================================================
std::set<std::string> LaneHolds::robots(const std::string& fleet_name) const
{
  const auto it = _robots.find(fleet_name);
  return it != _robots.end() ? it->second : std::set<std::string>();
}

//==============================================================================
bool LaneHolds::_known(const RobotId& key) const
{
  const auto it = _robots.find(key.fleet_name);
  return it != _robots.end() && it->second.count(key.robot_name) > 0;
}

//==============================================================================
auto LaneHolds::_find(
  const RequestKey& key,
  const std::string& requester_id) const -> const Latest*
{
  const auto it = _requests.find(key);
  if (it == _requests.end())
    return nullptr;

  const auto latest = it->second.find(requester_id);
  return latest != it->second.end() ? &latest->second : nullptr;
}

//==============================================================================
std::set<std::string> LaneHolds::_requesters(const RobotId& robot) const
{
  std::set<std::string> result;
  for (const auto& key :
    {RequestKey{robot.fleet_name, ""},
      RequestKey{robot.fleet_name, robot.robot_name}})
  {
    const auto it = _requests.find(key);
    if (it == _requests.end())
      continue;

    for (const auto& [requester_id, _] : it->second)
      result.insert(requester_id);
  }

  return result;
}

//==============================================================================
auto LaneHolds::_applied(
  const std::string& requester_id,
  const RobotId& robot) const -> const Latest*
{
  const auto* fleet_wide = _find(RequestKey{robot.fleet_name, ""},
      requester_id);
  const auto* robot_only =
    _find(RequestKey{robot.fleet_name, robot.robot_name}, requester_id);

  if (!fleet_wide)
    return robot_only;

  if (!robot_only)
    return fleet_wide;

  // The later one. One timeline per requester and fleet means no tie; the
  // robot list would win one.
  return fleet_wide->request_time_ns > robot_only->request_time_ns ?
    fleet_wide : robot_only;
}

//==============================================================================
std::vector<RobotId> LaneHolds::_covered(const RequestKey& key) const
{
  std::vector<RobotId> result;
  if (key.robot_name.empty())
  {
    for (const auto& robot : robots(key.fleet_name))
      result.push_back(RobotId{key.fleet_name, robot});

    return result;
  }

  const RobotId robot{key.fleet_name, key.robot_name};
  if (_known(robot))
    result.push_back(robot);

  return result;
}

} // namespace lane_closure_supervisor
} // namespace rmf_fleet_adapter
