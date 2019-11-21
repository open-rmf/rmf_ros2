/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef SRC__READ_ONLY__FLEETADAPTERNODE_HPP
#define SRC__READ_ONLY__FLEETADAPTERNODE_HPP

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rmf_traffic_msgs/srv/submit_trajectories.hpp>
#include <rmf_traffic_msgs/srv/delay_trajectories.hpp>
#include <rmf_traffic_msgs/srv/replace_trajectories.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>

#include <rclcpp/node.hpp>

#include <unordered_map>
#include <vector>

namespace rmf_fleet_adapter {
namespace read_only {

//==============================================================================
class FleetAdapterNode : public rclcpp::Node
{
public:

  static std::shared_ptr<FleetAdapterNode> make(
      const std::string& fleet_name,
      rmf_traffic::agv::VehicleTraits traits,
      rmf_traffic::Duration delay_threshold,
      rmf_traffic::Duration wait_time = std::chrono::seconds(10));

private:

  FleetAdapterNode(
      const std::string& fleet_name,
      rmf_traffic::agv::VehicleTraits traits,
      rmf_traffic::Duration delay_threshold);

  std::string _fleet_name;

  rmf_traffic::agv::VehicleTraits _traits;

  rmf_traffic::Duration _delay_threshold;

  using SubmitTrajectories = rmf_traffic_msgs::srv::SubmitTrajectories;
  rclcpp::Client<SubmitTrajectories>::SharedPtr _client_submit_trajectories;

  using DelayTrajectories = rmf_traffic_msgs::srv::DelayTrajectories;
  rclcpp::Client<DelayTrajectories>::SharedPtr _client_delay_trajectories;

  using ReplaceTrajectories = rmf_traffic_msgs::srv::ReplaceTrajectories;
  rclcpp::Client<ReplaceTrajectories>::SharedPtr _client_replace_trajectories;

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_subscription;


  void fleet_state_update(FleetState::UniquePtr new_state);

  using Location = rmf_fleet_msgs::msg::Location;
  struct ScheduleEntry
  {
    uint64_t schedule_id;
    bool dirty_id;
    std::vector<Location> path;
    rmf_traffic::Trajectory trajectory;

    ScheduleEntry()
    : dirty_id(true),
      trajectory("")
    {
      // Do nothing
    }
  };

  // TODO(MXG): We could add threads to make this adapter more efficient, but
  // then we'll need to protect this map with a mutex.
  using ScheduleEntries = std::unordered_map<std::string, ScheduleEntry>;
  ScheduleEntries _schedule_entries;

  using RobotState = rmf_fleet_msgs::msg::RobotState;

  void submit_robot(const RobotState& state);

  void update_robot(
      const RobotState& state,
      const ScheduleEntries::iterator& it);

  bool handle_delay(
      const RobotState& state,
      const ScheduleEntries::iterator& it);

  rmf_traffic::Trajectory make_trajectory(const RobotState& state) const;

  void update_id(const std::string& name, uint64_t id);
};

} // namespace read_only
} // namesapce rmf_fleet_adapter

#endif // SRC__READ_ONLY__FLEETADAPTERNODE_HPP
