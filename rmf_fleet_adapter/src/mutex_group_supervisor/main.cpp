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

using ClaimMap = std::unordered_map<std::string, TimeStamps>;
using MutexGroupRequest = rmf_fleet_msgs::msg::MutexGroupRequest;
using MutexGroupStates = rmf_fleet_msgs::msg::MutexGroupStates;
using MutextGroupAssignment = rmf_fleet_msgs::msg::MutexGroupAssignment;

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
        const auto c_it = g_it->second.find(request.claimer);
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
    claims.insert_or_assign(request.claimer, timestamps);
    for (const auto& s : latest_states.assignments)
    {
      if (s.group == request.group && !s.claimed.empty())
      {
        // The group is already claimed, so nothing to be done here
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
      std::vector<std::string> remove_claims;
      for (const auto& [claimer, timestamp] : claims)
      {
        if (timestamp.heartbeat_time + timeout < now)
        {
          remove_claims.push_back(claimer);
        }
      }

      std::string current_claimer;
      for (const auto& assignment : latest_states.assignments)
      {
        if (assignment.group == group)
        {
          current_claimer = assignment.claimed;
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

  void pick_next(const std::string& group)
  {
    const auto& claimers = mutex_groups[group];
    std::optional<std::pair<builtin_interfaces::msg::Time, std::string>> earliest;
    for (const auto& [claimer, timestamp] : claimers)
    {
      const auto& t = timestamp.claim_time;
      if (!earliest.has_value() || t < earliest->first)
      {
        earliest = std::make_pair(t, claimer);
      }
    }

    std::string claimer;
    if (earliest.has_value())
    {
      claimer = earliest->second;
    }
    bool group_found = false;
    for (auto& a : latest_states.assignments)
    {
      if (a.group == group)
      {
        a.claimed = claimer;
        group_found = true;
        break;
      }
    }
    if (!group_found)
    {
      latest_states.assignments.push_back(
        rmf_fleet_msgs::build<MutextGroupAssignment>()
        .group(group)
        .claimed(claimer));
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
