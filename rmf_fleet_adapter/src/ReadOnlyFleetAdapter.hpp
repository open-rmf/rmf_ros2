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

#ifndef RMF_FLEET_ADAPTER__SRC__READONLYFLEETADAPTER_HPP
#define RMF_FLEET_ADAPTER__SRC__READONLYFLEETADAPTER_HPP

#include <atomic>
#include <chrono>
#include <unordered_map>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

#include <rmf_traffic_msgs/srv/submit_trajectory.hpp>

#include "FleetComponents.hpp"


namespace rmf_fleet {
namespace adapter {

class ReadOnlyFleetAdapter : public rclcpp::Node
{
public:

  using SharedPtr = std::shared_ptr<ReadOnlyFleetAdapter>;

  static SharedPtr make(
      const std::string& fleet_id, 
      const std::string& graph_file_path,
      rmf_traffic::agv::VehicleTraits vehicle_traits,
      rmf_traffic::Duration wait_time = std::chrono::seconds(10));

  ~ReadOnlyFleetAdapter();

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using RobotState = rmf_fleet_msgs::msg::RobotState;
  using Location = rmf_fleet_msgs::msg::Location;

  using SubmitTrajectory = rmf_traffic_msgs::srv::SubmitTrajectory;
  using SubmitTrajectoryClient = rclcpp::Client<SubmitTrajectory>;
  using SubmitTrajectoryHandle = SubmitTrajectoryClient::SharedPtr;

private:

  std::string fleet_id;

  FleetControlLevel fleet_control_level;

  rclcpp::Subscription<FleetState>::SharedPtr fleet_state_sub;

  ReadOnlyFleetAdapter(const std::string& fleet_id);

  /// Holds the compulsory components of each fleet, regardless of how much
  /// control the fleet adapters have over the fleet.
  std::unique_ptr<FleetComponents> components;

  /// This function should only be called when everything has finished
  /// initilializing, especially the schedule database mirror
  void start(FleetComponents components);

  /// Allow the callback to update the schedule only when the previous 
  /// udpate is done.
  std::atomic<bool> ready_to_update_schedule;

  /// Parses, interpolates and updates the schedule serially with every
  /// callback, might not be the most efficient, but with custom QoS we can
  /// ensure that only the lastest updates used for the scheduler.
  void fleet_state_cb(FleetState::UniquePtr _msg);

  /// Callback function for when the trajectory updating service completes
  /// and receives a response.
  void scheduler_updated_response_fn(
      const SubmitTrajectoryClient::SharedFuture& response);

};

} // namespace adapter
} // namespace rmf_fleet

#endif // RMF_FLEET_ADAPTER__SRC__READONLYFLEETADAPTER_HPP
