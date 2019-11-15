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


#ifndef SRC__RMF_TRAFFIC_ROS2__PROTO_FLEET_ADAPTER__FLEETADAPTERNODE_HPP
#define SRC__RMF_TRAFFIC_ROS2__PROTO_FLEET_ADAPTER__FLEETADAPTERNODE_HPP

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>

#include <rmf_fleet_msgs/msg/path_request.hpp>

#include <rmf_traffic_msgs/msg/test_task_request.hpp>
#include <rmf_traffic_msgs/srv/submit_trajectory.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rclcpp/node.hpp>

namespace proto_fleet_adapter {

//==============================================================================
class FleetAdapterNode : public rclcpp::Node
{
public:

  static std::shared_ptr<FleetAdapterNode> make(
      std::string fleet_name,
      const std::string& graph_file,
      rmf_traffic::agv::VehicleTraits vehicle_traits,
      rmf_traffic::Duration wait_time = std::chrono::seconds(10));

private:

  using SubmitTrajectory = rmf_traffic_msgs::srv::SubmitTrajectory;
  using SubmitTrajectoryClient = rclcpp::Client<SubmitTrajectory>;
  using SubmitTrajectoryHandle = SubmitTrajectoryClient::SharedPtr;

  struct Data
  {
    std::unordered_map<std::string, std::size_t> waypoint_keys;
    rmf_traffic_ros2::schedule::MirrorManager mirror;
    SubmitTrajectoryHandle submit_trajectory;
    rmf_traffic::agv::Planner planner;

    Data(
        std::unordered_map<std::string, std::size_t> waypoint_keys_,
        rmf_traffic_ros2::schedule::MirrorManager mirror_,
        SubmitTrajectoryHandle submit_trajectory_,
        rmf_traffic::agv::Planner::Configuration config_)
    : waypoint_keys(std::move(waypoint_keys_)),
      mirror(std::move(mirror_)),
      submit_trajectory(std::move(submit_trajectory_)),
      planner(std::move(config_),
              rmf_traffic::agv::Planner::Options{mirror.viewer()})
    {
      // Do nothing
    }
  };

  FleetAdapterNode(std::string fleet_name);

  void start(Data data);


  std::string fleet_name;

  // TODO(MXG): Replace this with a std::optional as soon as we can use C++17
  std::unique_ptr<Data> data;

  using TestTaskRequest = rmf_traffic_msgs::msg::TestTaskRequest;
  using TestTaskRequestSub = rclcpp::Subscription<TestTaskRequest>;
  using TestTaskRequestPub = rclcpp::Publisher<TestTaskRequest>;

  TestTaskRequestSub::SharedPtr test_task_request_sub;
  TestTaskRequestPub::SharedPtr test_task_request_retry;
  void test_task_request(TestTaskRequest::UniquePtr msg);


  using PathRequest = rmf_fleet_msgs::msg::PathRequest;
  using PathRequestPub = rclcpp::Publisher<PathRequest>;

  PathRequestPub::SharedPtr path_request_publisher;

  void test_task_receive_response(
      const SubmitTrajectoryClient::SharedFuture& response);

};

} // namespace proto_fleet_adapter

#endif // SRC__RMF_TRAFFIC_ROS2__PROTO_FLEET_ADAPTER__FLEETADAPTERNODE_HPP
