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

#ifndef RMF_FLEET_ADAPTER__SRC__FLEETADAPTERNODE_HPP
#define RMF_FLEET_ADAPTER__SRC__FLEETADAPTERNODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>

#include <rmf_traffic_msgs/srv/submit_trajectory.hpp>


namespace rmf_fleet {
namespace adapter {

enum class FleetControlLevel : uint8_t
{
  FullControl,
  StopControl,
  ReadOnly
};

struct FleetComponents
{
  rmf_traffic::agv::Graph graph;
  std::unordered_map<std::string, std::size_t> waypoint_keys;
  rmf_traffic::agv::VehicleTraits traits;

  /// Mirror of the schedule database, which syncs with the main database
  /// asynchronously under the hood.
  rmf_traffic_ros2::schedule::MirrorManager mirror;

  /// Service handle to submit trajectories to the schedule database, to be
  /// forcefully registered for uncontrollable fleets, or to be accept 
  /// feedback if conflict arises.
  // Note (AC): this will be changed once the API for updating the main
  // database has been confirmed
  using SubmitTrajectory = rmf_traffic_msgs::srv::SubmitTrajectory;
  using SubmitTrajectoryClient = rclcpp::Client<SubmitTrajectory>;
  using SubmitTrajectoryHandle = SubmitTrajectoryClient::SharedPtr;
  SubmitTrajectoryHandle submit_trajectory;
};

} // adapter
} // rmf_fleet

#endif // RMF_FLEET_ADAPTER__SRC__FLEETADAPTERNODE_HPP
