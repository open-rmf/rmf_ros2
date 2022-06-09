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


#include "LaneBlocker.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
LaneBlocker::LaneBlocker(const rclcpp::NodeOptions& options)
: Node("lane_blocker_node", options)
{
  _data = std::make_shared<Data>();

  _data->lane_closure_pub = this->create_publisher<LaneRequest>(
    rmf_fleet_adapter::LaneClosureRequestTopicName,
    rclcpp::SystemDefaultsQoS());

  _data->obstacle_sub = this->create_subscription<Obstacles>(
    "rmf_obstacles",
    rclcpp::SystemDefaultsQoS(),
    [data = _data](const Obstacles& msg)
    {
      data->obstacle_cb(msg);
    });

  auto transient_qos = rclcpp::SystemDefaultsQoS().reliable().transient_local();

  _data->graph_sub = this->create_subscription<NavGraph>(
    "nav_graphs",
    transient_qos,
    [data = _data](std::shared_ptr<const NavGraph> msg)
    {
      data->graph_cb(std::move(msg));
    });

  _data->lane_states_sub = this->create_subscription<LaneStates>(
    "lane_states",
    rclcpp::SystemDefaultsQoS(),
    [data = _data](std::shared_ptr<const LaneStates> msg)
    {
      data->lane_states_cb(std::move(msg));
    });

}

//==============================================================================
void LaneBlocker::Data::obstacle_cb(const Obstacles& msg)
{

}

//==============================================================================
void LaneBlocker::Data::graph_cb(std::shared_ptr<const NavGraph> msg)
{

}

//==============================================================================
void LaneBlocker::Data::lane_states_cb(std::shared_ptr<const LaneStates> msg)
{

}

RCLCPP_COMPONENTS_REGISTER_NODE(LaneBlocker)
