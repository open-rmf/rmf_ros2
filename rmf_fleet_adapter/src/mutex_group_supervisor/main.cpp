/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rmf_fleet_msgs/msg/mutex_group_request.hpp>
#include <rmf_fleet_msgs/msg/mutex_group_assignment.hpp>
#include <rmf_fleet_msgs/msg/mutex_group_states.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include <unordered_map>
#include <queue>
#include <chrono>

struct TimeStamps
{
  rclcpp::Time claim_time;
  std::chrono::steady_clock::time_point heartbeat_time;
};

using ClaimMap = std::unordered_map<uint64_t, TimeStamps>;
using MutexGroupRequest = rmf_fleet_msgs::msg::MutexGroupRequest;
using MutexGroupStates = rmf_fleet_msgs::msg::MutexGroupStates;
using MutextGroupAssignment = rmf_fleet_msgs::msg::MutexGroupAssignment;

const uint64_t Unclaimed = rmf_fleet_adapter::Unclaimed;

class Node : public rclcpp::Node
{
public:
  Node()
  : rclcpp::Node("mutex_group_supervisor")
  {
    const auto qos = rclcpp::SystemDefaultsQoS()
      .reliable()
      .transient_local()
      .keep_last(100);

    latest_states = rmf_fleet_msgs::build<MutexGroupStates>()
      .assignments({});

    request_sub = create_subscription<MutexGroupRequest>(
      rmf_fleet_adapter::MutexGroupRequestTopicName,
      qos,
      [&](const MutexGroupRequest& request)
      {
        handle_request(request);
      });

    state_pub = create_publisher<MutexGroupStates>(
      rmf_fleet_adapter::MutexGroupStatesTopicName,
      qos);

    heartbeat_timer = create_wall_timer(
      std::chrono::seconds(2),
      [&]() { do_heartbeat(); });
  }

  void handle_request(const MutexGroupRequest& request)
  {
    auto& claims = mutex_groups[request.group];
    if (request.mode == request.MODE_RELEASE)
    {
      const auto g_it = mutex_groups.find(request.group);
      if (g_it != mutex_groups.end())
      {
        const auto c_it = g_it->second.find(request.claimant);
        if (c_it != g_it->second.end())
        {
          if (c_it->second.claim_time <= request.claim_time)
          {
            g_it->second.erase(c_it);
            pick_next(request.group);
            state_pub->publish(latest_states);
          }
        }
      }
      return;
    }

    auto now = std::chrono::steady_clock::now();
    auto claim_time = rclcpp::Time(request.claim_time);
    auto timestamps = TimeStamps { request.claim_time, now };
    claims.insert_or_assign(request.claimant, timestamps);
    for (const auto& s : latest_states.assignments)
    {
      if (s.group == request.group && s.claimant != Unclaimed)
      {
        check_for_conflicts();
        return;
      }
    }
    pick_next(request.group);
  }

  void do_heartbeat()
  {
    const auto now = std::chrono::steady_clock::now();
    // TODO(MXG): Make this timeout configurable
    const auto timeout = std::chrono::seconds(10);
    for (auto& [group, claims] : mutex_groups)
    {
      std::vector<uint64_t> remove_claims;
      for (const auto& [claimant, timestamp] : claims)
      {
        if (timestamp.heartbeat_time + timeout < now)
        {
          remove_claims.push_back(claimant);
        }
      }

      uint64_t current_claimer = Unclaimed;
      for (const auto& assignment : latest_states.assignments)
      {
        if (assignment.group == group)
        {
          current_claimer = assignment.claimant;
          break;
        }
      }

      bool need_next_pick = false;
      for (const auto& remove_claim : remove_claims)
      {
        if (current_claimer == remove_claim)
        {
          need_next_pick = true;
        }

        claims.erase(remove_claim);
      }

      if (need_next_pick)
      {
        pick_next(group);
      }
    }

    state_pub->publish(latest_states);
  }

  struct ClaimList
  {
    std::unordered_set<std::string> groups;
    std::optional<rclcpp::Time> earliest_time;

    void insert(std::string group, rclcpp::Time time)
    {
      groups.insert(std::move(group));
      if (!earliest_time.has_value() || time < *earliest_time)
      {
        earliest_time = time;
      }
    }

    void normalize(
      uint64_t claimant,
      std::unordered_map<std::string, ClaimMap>& mutex_groups) const
    {
      if (!earliest_time.has_value())
        return;

      for (const auto& group : groups)
      {
        mutex_groups[group][claimant].claim_time = *earliest_time;
      }
    }
  };

  void check_for_conflicts()
  {
    std::unordered_map<uint64_t, ClaimList> claims;
    for (const auto& [group, group_claims] : mutex_groups)
    {
      for (const auto& [claimant, timestamps] : group_claims)
      {
        claims[claimant].insert(group, timestamps.claim_time);
      }
    }

    std::unordered_set<std::string> normalized_groups;
    for (auto i = claims.begin(); i != claims.end(); ++i)
    {
      auto j = i;
      ++j;
      for (; j != claims.end(); ++j)
      {
        const auto& claim_i = i->second;
        const auto& claim_j = j->second;
        if (claim_i.groups == claim_j.groups && claim_i.groups.size() > 1)
        {
          claim_i.normalize(i->first, mutex_groups);
          claim_j.normalize(j->first, mutex_groups);

          std::stringstream ss;
          for (const auto& group : claim_i.groups)
          {
            normalized_groups.insert(group);
            ss << "[" << group << "]";
          }

          RCLCPP_INFO(
            get_logger(),
            "Resolving mutex conflict between claimants [%lu] and [%lu] which both "
            "want the mutex combination %s",
            i->first,
            j->first,
            ss.str().c_str());
        }
      }
    }

    for (const auto& group : normalized_groups)
      pick_next(group);
  }

  void pick_next(const std::string& group)
  {
    const auto& claimants = mutex_groups[group];
    std::optional<std::pair<builtin_interfaces::msg::Time, uint64_t>> earliest;
    for (const auto& [claimant, timestamp] : claimants)
    {
      const auto& t = timestamp.claim_time;
      if (!earliest.has_value() || t < earliest->first)
      {
        earliest = std::make_pair(t, claimant);
      }
    }

    uint64_t claimant = Unclaimed;
    builtin_interfaces::msg::Time claim_time;
    if (earliest.has_value())
    {
      claim_time = earliest->first;
      claimant = earliest->second;
    }
    bool group_found = false;
    for (auto& a : latest_states.assignments)
    {
      if (a.group == group)
      {
        a.claimant = claimant;
        a.claim_time = claim_time;
        group_found = true;
        break;
      }
    }
    if (!group_found)
    {
      latest_states.assignments.push_back(
        rmf_fleet_msgs::build<MutextGroupAssignment>()
        .group(group)
        .claimant(claimant)
        .claim_time(claim_time));
    }
  }

  std::unordered_map<std::string, ClaimMap> mutex_groups;
  rclcpp::Subscription<MutexGroupRequest>::SharedPtr request_sub;
  rclcpp::Publisher<MutexGroupStates>::SharedPtr state_pub;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
  MutexGroupStates latest_states;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node>());
  rclcpp::shutdown();
}
