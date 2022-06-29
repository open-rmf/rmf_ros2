/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef SRC__LANE_BLOCKER__LANEBLOCKER_HPP
#define SRC__LANE_BLOCKER__LANEBLOCKER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic/agv/Graph.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <rmf_building_map_msgs/msg/graph.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/speed_limit_request.hpp>
#include <rmf_fleet_msgs/msg/lane_states.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <unordered_map>

//==============================================================================
/// Modify states of lanes for fleet adapters based on density of obstacles
class LaneBlocker : public rclcpp::Node
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;
  using NavGraph = rmf_building_map_msgs::msg::Graph;
  using TrafficGraph = rmf_traffic::agv::Graph;
  using LaneRequest = rmf_fleet_msgs::msg::LaneRequest;
  using SpeedLimitRequest = rmf_fleet_msgs::msg::SpeedLimitRequest;
  using LaneStates = rmf_fleet_msgs::msg::LaneStates;

  /// Constructor
  LaneBlocker(
    const rclcpp::NodeOptions& options  = rclcpp::NodeOptions());

private:
    void obstacle_cb(const Obstacles& msg);

    rclcpp::Subscription<Obstacles>::SharedPtr _obstacle_sub;
    rclcpp::Subscription<NavGraph>::SharedPtr _graph_sub;
    rclcpp::Subscription<LaneStates>::SharedPtr _lane_states_sub;
    rclcpp::Publisher<LaneRequest>::SharedPtr _lane_closure_pub;
    rclcpp::Publisher<SpeedLimitRequest>::SharedPtr _speed_limit_pub;
    double _tf2_lookup_duration;

    std::string _rmf_frame;
    std::unique_ptr<tf2_ros::Buffer> _tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _transform_listener;

    std::unordered_map<std::string, TrafficGraph> _traffic_graphs;
    std::unordered_map<std::string, LaneStates::ConstSharedPtr> _lane_states;
    double _obstacle_lane_threshold;
};


#endif // SRC__LANE_BLOCKER__LANEBLOCKER_HPP
