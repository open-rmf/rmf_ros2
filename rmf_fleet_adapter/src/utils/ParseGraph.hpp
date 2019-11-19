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

#ifndef RMF_FLEET_ADAPTER__SRC__UTILS__PARSEGRAPH_HPP
#define RMF_FLEET_ADAPTER__SRC__UTILS__PARSEGRAPH_HPP

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <unordered_map>

#include <rclcpp/node.hpp>

namespace rmf_fleet {
namespace adapter {

//==============================================================================
bool parse_graph(
    const std::string& filename,
    const rmf_traffic::agv::VehicleTraits& vehicle_traits,
    const rclcpp::Node& node,
    rmf_traffic::agv::Graph& graph,
    std::unordered_map<std::string, std::size_t>& waypoint_keys);

} // namespace adapter
} // namespace rmf_fleet

#endif // RMF_FLEET_ADAPTER__SRC__UTILS__PARSEGRAPH_HPP
