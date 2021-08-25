/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef RMF_FLEET_ADAPTER__AGV__PARSE_GRAPH_HPP
#define RMF_FLEET_ADAPTER__AGV__PARSE_GRAPH_HPP

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_fleet_adapter {
namespace agv {

/// Parse the graph described by a yaml file.
///
/// \warning This will throw a std::runtime_error if the file has a syntax
/// error.
rmf_traffic::agv::Graph parse_graph(
  const std::string& filename,
  const rmf_traffic::agv::VehicleTraits& vehicle_traits);

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__PARSE_GRAPH_HPP
