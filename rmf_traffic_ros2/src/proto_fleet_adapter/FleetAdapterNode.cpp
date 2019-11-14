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

#include "FleetAdapterNode.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include <rclcpp/rclcpp.hpp>

#include "ParseGraph.hpp"

namespace proto_fleet_adapter {

//==============================================================================
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make(
    std::string fleet_name,
    const std::string& graph_file,
    rmf_traffic::agv::VehicleTraits vehicle_traits,
    rmf_traffic::Duration wait_time)
{
  const auto start_time = std::chrono::steady_clock::now();
  std::shared_ptr<FleetAdapterNode> fleet_adapter(
        new FleetAdapterNode(std::move(fleet_name)));

  auto mirror_mgr_future = rmf_traffic_ros2::schedule::make_mirror(
        *fleet_adapter, rmf_traffic::schedule::Query::Spacetime());

  auto submit_trajectory = fleet_adapter->create_client<SubmitTrajectory>(
        rmf_traffic_ros2::SubmitTrajectoryServiceName);

  rmf_traffic::agv::Graph graph;
  std::unordered_map<std::string, std::size_t> waypoint_keys;
  if(!parse_graph(graph_file, vehicle_traits, *fleet_adapter, graph, waypoint_keys))
    return nullptr;

  const auto stop_time = start_time + wait_time;
  while(rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
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
            Data{
              std::move(waypoint_keys),
              mirror_mgr_future.get(),
              std::move(submit_trajectory),
              rmf_traffic::agv::Planner::Configuration{
                std::move(graph),
                std::move(vehicle_traits)
              }
            });

      return fleet_adapter;
    }
  }

  RCLCPP_ERROR(
        fleet_adapter->get_logger(),
        "Mirror was not initialized in enough time ["
        + std::to_string(rmf_traffic::time::to_seconds(wait_time)) + "s]!");
  return nullptr;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode(std::string _fleet_name)
  : Node(_fleet_name + "_fleet_adapter"),
    fleet_name(std::move(_fleet_name))
{
  // Do nothing

  // NOTE(MXG): We need to initialize an empty node so we can spin up the
  // MirrorManagerFuture into a full MirrorManager. But also we don't want this
  // node to do anything until all its data fields are finalized, so this
  // constructor is more like a formality than a real constructor.
}

//==============================================================================
void FleetAdapterNode::start(Data _data)
{
  data = std::make_unique<Data>(std::move(_data));
  data->mirror.update();

  test_task_request_sub = create_subscription<TestTaskRequest>(
        "test_task_request", rclcpp::SystemDefaultsQoS(),
        [&](TestTaskRequest::UniquePtr msg)
  {
    this->test_task_request(std::move(msg));
  });

  test_task_request_retry = create_publisher<TestTaskRequest>(
        "test_task_request");

  path_request_publisher = create_publisher<PathRequest>("gazebo_path_requests");
}

//==============================================================================
void FleetAdapterNode::test_task_request(TestTaskRequest::UniquePtr msg)
{
  std::cout << "test_task_request triggered (" << msg->fleet_name
            << ", " << get_name() << ")" << std::endl;

  if(msg->fleet_name != fleet_name)
    return;

  RCLCPP_INFO(
        get_logger(),
        "Performing task request from [" + msg->start_waypoint_name + "] to ["
        + msg->goal_waypoint_name + "]");

  const auto start_it = data->waypoint_keys.find(msg->start_waypoint_name);
  if(start_it == data->waypoint_keys.end())
  {
    RCLCPP_ERROR(
          get_logger(),
          "Unrecognized start waypoint requested: " + msg->start_waypoint_name);
    return;
  }

  const auto goal_it = data->waypoint_keys.find(msg->goal_waypoint_name);
  if(goal_it == data->waypoint_keys.end())
  {
    RCLCPP_ERROR(
          get_logger(),
          "Unrecognized goal waypoint requested: " + msg->goal_waypoint_name);
    return;
  }

  using namespace std::chrono_literals;
  const auto target_time = std::chrono::steady_clock::now() + 2s;

  rmf_traffic::agv::Plan::Start start{
        target_time,
        start_it->second,
        msg->initial_orientation
  };

  rmf_traffic::agv::Plan::Goal goal{goal_it->second};

  const auto plan = data->planner.plan(start, goal);
  if(!plan)
  {
    RCLCPP_WARN(get_logger(), "Failed to find a solution! We will retry!");
    test_task_request_retry->publish(*msg);
    return;
  }

  const std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints =
      plan.get_waypoints();

  const std::vector<rmf_traffic::Trajectory> solution = plan.get_trajectories();

  if(solution.empty() || solution.back().duration() == rmf_traffic::Duration(0))
  {
    RCLCPP_INFO(get_logger(), "No trajectory needed!");
    return;
  }

  const auto new_target_time = std::chrono::steady_clock::now() + 2s;
  const auto time_adjustment = new_target_time - target_time;
  SubmitTrajectory::Request request_msg;
  for(auto t : solution)
  {
    t.begin()->adjust_finish_times(time_adjustment);
    request_msg.trajectories.emplace_back(rmf_traffic_ros2::convert(t));
  }

  std::string notice =
      "Generated trajectories [" + std::to_string(solution.size()) + "]";
  for(const auto& t : solution)
    notice += " | " + std::to_string(rmf_traffic::time::to_seconds(t.duration()));

  RCLCPP_INFO(get_logger(), notice);

  data->submit_trajectory->async_send_request(
        std::make_shared<SubmitTrajectory::Request>(std::move(request_msg)),
        [&](const SubmitTrajectoryClient::SharedFuture response)
  {
    this->test_task_receive_response(response);
  });

  PathRequest path_msg;
  path_msg.fleet_name = fleet_name;
  path_msg.robot_name = fleet_name;
  for(const auto& wp : waypoints)
  {
    rmf_fleet_msgs::msg::Location location;
    const auto p = wp.position();
    location.x = p[0];
    location.y = p[1];
    location.yaw = p[2];
    location.t = rmf_traffic_ros2::convert(wp.time());

    path_msg.path.emplace_back(std::move(location));
  }

  // TODO FIXME(MXG): This should only be published after we've received
  // confirmation from the schedule node.
  path_request_publisher->publish(path_msg);
}

//==============================================================================
void FleetAdapterNode::test_task_receive_response(
    const SubmitTrajectoryClient::SharedFuture& response)
{
  const auto response_msg = response.get();
  if(response_msg->accepted)
  {
    RCLCPP_INFO(get_logger(), "Response: accepted");
  }
  else
  {
    std::string error_msg = "Response: " + response_msg->error + ". Conflicts:";
    for(const auto& conflict : response_msg->conflicts)
    {
      error_msg += "\n -- " + std::to_string(conflict.index) + " @ "
          + std::to_string(conflict.time);
    }
    RCLCPP_INFO(get_logger(), error_msg);
  }
}

} // namespace proto_fleet_adapter
