/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rmf_traffic/agv/Graph.hpp>

#include <rmf_site_map_msgs/msg/site_map.hpp>

#include <rmf_building_map_msgs/msg/graph.hpp>

#include <rmf_traffic/agv/Graph.hpp>

#include <optional>
#include <memory>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::agv::Graph convert(const rmf_site_map_msgs::msg::SiteMap& from,
  int graph_idx = 0, double wp_tolerance = 1e-3);

//==============================================================================
/// Convert a valid rmf_building_map_msgs::msg::Graph message to an
/// rmf_traffic::agv::Graph object.
/// Returns nullopt if required fields are missing.
std::optional<rmf_traffic::agv::Graph> convert(
  const rmf_building_map_msgs::msg::Graph& from);

//==============================================================================
/// Convert a valid rmf_traffic::agv::Graph object to an
/// rmf_building_map_msgs::msg::Graph message.
/// Returns nullptr if required fields are missing or fleet_name is empty.
std::unique_ptr<rmf_building_map_msgs::msg::Graph> convert(
  const rmf_traffic::agv::Graph& from, const std::string& fleet_name);


} // namespace rmf_traffic_ros2
