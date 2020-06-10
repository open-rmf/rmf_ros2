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

#ifndef SRC__RMF_FLEET_ADAPTER__PARSEGRAPH_HPP
#define SRC__RMF_FLEET_ADAPTER__PARSEGRAPH_HPP

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <unordered_map>

#include <rclcpp/node.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_fleet_adapter {

struct GraphInfo
{
  rmf_traffic::agv::Graph graph;

  // TODO(MXG): Consider making waypoint names an intrinsic property of a Graph
  using NameMap = std::unordered_map<std::size_t, std::string>;

  /// A map from a waypoint index to the workcell that is stationed at that
  /// waypoint
  NameMap workcell_names;

  /// The parking spot
  std::vector<std::size_t> parking_spots;
};

//==============================================================================
rmf_utils::optional<GraphInfo> parse_graph(
  const std::string& filename,
  const rmf_traffic::agv::VehicleTraits& vehicle_traits,
  const rclcpp::Node& node);

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PARSEGRAPH_HPP
