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

#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <rmf_building_map_msgs/msg/graph.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/lane_states.hpp>

//==============================================================================
/// Modify states of lanes for fleet adapters based on density of obstacles
class LaneBlocker : public rclcpp::Node
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;
  using NavGraph = rmf_building_map_msgs::msg::Graph;
  using LaneRequest = rmf_fleet_msgs::msg::LaneRequest;
  using LaneStates = rmf_fleet_msgs::msg::LaneStates;

  /// Constructor
  LaneBlocker(
    const rclcpp::NodeOptions& options  = rclcpp::NodeOptions());

private:
  struct Data
  {
    void obstacle_cb(const Obstacles& msg);
    void graph_cb(std::shared_ptr<const NavGraph> msg);
    void lane_states_cb(std::shared_ptr<const LaneStates> msg);

    rclcpp::Subscription<Obstacles>::SharedPtr obstacle_sub;
    rclcpp::Subscription<NavGraph>::SharedPtr graph_sub;
    rclcpp::Subscription<LaneStates>::SharedPtr lane_states_sub;
    rclcpp::Publisher<LaneRequest>::SharedPtr lane_closure_pub;
  };

  std::shared_ptr<Data> _data;

};


#endif // SRC__LANE_BLOCKER__LANEBLOCKER_HPP