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

#include <rmf_traffic/agv/Interpolate.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include "ReadOnlyFleetAdapter.hpp"
#include "utils/ParseGraph.hpp"

namespace rmf_fleet {
namespace adapter {

ReadOnlyFleetAdapter::SharedPtr ReadOnlyFleetAdapter::make(
    const std::string& _fleet_id,
    const std::string& _graph_file_path,
    rmf_traffic::agv::VehicleTraits _vehicle_traits,
    rmf_traffic::Duration _wait_time)
{
  const auto start_time = std::chrono::steady_clock::now();
  SharedPtr fleet_adapter(new ReadOnlyFleetAdapter(_fleet_id));

  auto mirror_mgr_future = rmf_traffic_ros2::schedule::make_mirror(
        *fleet_adapter, rmf_traffic::schedule::Query::Spacetime());

  // Note (AC): this will be changed once the API for updating the main
  // database has been confirmeds
  auto submit_trajectory = fleet_adapter->create_client<SubmitTrajectory>(
        rmf_traffic_ros2::SubmitTrajectoryServiceName);

  rmf_traffic::agv::Graph graph;
  std::unordered_map<std::string, std::size_t> waypoint_keys;
  if(!parse_graph(
      _graph_file_path, _vehicle_traits, *fleet_adapter, graph, waypoint_keys))
    return nullptr;

  const auto stop_time = start_time + _wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    // NOTE(MXG): We need to spin the node in order to get the mirror manager
    // fully initialized.
    rclcpp::spin_some(fleet_adapter);

    using namespace std::chrono_literals;
    bool ready = (mirror_mgr_future.wait_for(0s) == std::future_status::ready);
    ready &= submit_trajectory->service_is_ready();

    if(ready)
    {
      fleet_adapter->start(
          FleetComponents{
              std::move(graph),
              std::move(waypoint_keys),
              std::move(_vehicle_traits),
              mirror_mgr_future.get(),
              std::move(submit_trajectory)
          });
      return fleet_adapter;
    }
  }

  RCLCPP_ERROR(
      fleet_adapter->get_logger(),
      "Mirror was not initialized in enough time ["
      + std::to_string(rmf_traffic::time::to_seconds(_wait_time)) + "s]!");
  return nullptr;
}

ReadOnlyFleetAdapter::ReadOnlyFleetAdapter(const std::string& _fleet_id)
: Node(_fleet_id),
  fleet_id(_fleet_id),
  fleet_control_level(FleetControlLevel::ReadOnly)
{}

ReadOnlyFleetAdapter::~ReadOnlyFleetAdapter()
{}

void ReadOnlyFleetAdapter::start(FleetComponents _components)
{
  ready_to_update_schedule = true;

  components = std::make_unique<FleetComponents>(std::move(_components));
  components->mirror.update();

  fleet_state_sub = create_subscription<FleetState>(
      fleet_id + "/fleet_state", rclcpp::SystemDefaultsQoS(),
      [&](FleetState::UniquePtr msg)
  {
    fleet_state_cb(std::move(msg));
  });
}

void ReadOnlyFleetAdapter::fleet_state_cb(FleetState::UniquePtr _msg)
{
  RCLCPP_INFO(get_logger(), "got something from " + _msg->name);

  /// If the previous schedule database update has not been completed yet,
  /// skip this callback.
  if (!ready_to_update_schedule)
    return;

  // parses through each robot state message, going through their waypoints
  // updates their trajectories with the schedule database

  RCLCPP_INFO(get_logger(), "handling it now");
  if (_msg->robots.empty())
  {
    RCLCPP_INFO(get_logger(), "no robots found.");
    return;
  }

  // populate the request message
  SubmitTrajectory::Request request_msg;
  request_msg.fleet_id = fleet_id;
  request_msg.fleet_control_level = SubmitTrajectory::Request::READ_ONLY_FLEET;
  request_msg.trajectories.clear();

  RCLCPP_INFO(get_logger(), "parsing each robot");
  // get the Trajectory for each robot
  for (const RobotState rs : _msg->robots)
  {
    RCLCPP_INFO(get_logger(), "working on robot: " + rs.name);

    // assuming that RobotState.path states the path, starting from the next 
    // waypoint location, instead of the start of the whole path, which the
    // robot started from

    // get the map name, using a random name by default for now,
    // Note (AC): the nomal way to do things would be to get the map name
    // from the waypoints in the graph, we currently only have the message
    // waypoints, nothing to do with the parsed graph
    //
    // Q: should the map names be unified between RMF and vendors?
    std::string map_name = "default_map";

    // translate the starting time
    std::chrono::steady_clock::time_point start_time = 
        std::chrono::steady_clock::time_point(
            std::chrono::seconds(rs.location.t.sec));

    // construct the string of positions and orientations
    std::vector<Eigen::Vector3d> input_positions;
    for (const Location loc : rs.path)
    {
      input_positions.emplace_back(loc.x, loc.y, loc.yaw);
    }

    // construct the interpolation options, use the defaults for now
    rmf_traffic::agv::Interpolate::Options interpolate_options();

    // get the interpolation results for this robot
    rmf_traffic::Trajectory robot_trajectory =
        rmf_traffic::agv::Interpolate::positions(
            map_name, 
            components->traits, 
            start_time,
            input_positions,
            interpolate_options);

    // insert the interpolated trajectory into the service request message
    request_msg.trajectories.emplace_back(
        rmf_traffic_ros2::convert(robot_trajectory));
  }

  // submit the service request
  RCLCPP_INFO(get_logger(), "sending request");
  ready_to_update_schedule = false;
  components->submit_trajectory->async_send_request(
      std::make_shared<SubmitTrajectory::Request>(std::move(request_msg)),
      [&](const SubmitTrajectoryClient::SharedFuture _response)
  {
    scheduler_updated_response_fn(_response);
  });
}

void ReadOnlyFleetAdapter::scheduler_updated_response_fn(
    const SubmitTrajectoryClient::SharedFuture& _response)
{
  const auto response_msg = _response.get();
  
  /// Read-only fleets' schedules are forcibly updated on the scheduler
  /// database, and should only be rejected if there is an error in the request
  if (!response_msg->accepted)
    RCLCPP_INFO(get_logger(), "Response: " + response_msg->error);

  ready_to_update_schedule = true;
}

} // namespace adapter
} // namespace rmf_fleet
