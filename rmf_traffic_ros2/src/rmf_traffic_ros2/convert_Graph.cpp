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

#include <rmf_traffic_ros2/agv/Graph.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::agv::Graph convert(const rmf_building_map_msgs::msg::Graph& from,
  int waypoint_offset)
{
  rmf_traffic::agv::Graph graph;
  // Iterate over vertices / waypoints
  // Graph params are not used for now
  for (const auto& vertex : from.vertices)
  {
    const Eigen::Vector2d location{
      vertex.x, vertex.y};
    auto& wp = graph.add_waypoint(from.name, location);
    // Add waypoint name if in the message
    if (vertex.name.size() > 0 && !graph.add_key(vertex.name, wp.index()))
    {
      throw std::runtime_error(
              "Duplicated waypoint name [" + vertex.name + "]");
    }
    for (const auto& param : vertex.params)
    {
      if (param.name == "is_parking_spot")
        wp.set_parking_spot(param.value_bool);
      else if (param.name == "is_holding_point")
        wp.set_holding_point(param.value_bool);
      else if (param.name == "is_passthrough_point")
        wp.set_passthrough_point(param.value_bool);
      else if (param.name == "is_charger")
        wp.set_charger(param.value_bool);
    }
  }
  // Iterate over edges / lanes
  for (const auto& edge : from.edges)
  {
    using Lane = rmf_traffic::agv::Graph::Lane;
    using Event = Lane::Event;
    // TODO(luca) Add remaining functionality, lifts, doors, docking points
    // events, orientation constraints
    rmf_utils::clone_ptr<Event> entry_event;
    rmf_utils::clone_ptr<Event> exit_event;
    // Waypoint offset is applied to ensure unique IDs when multiple levels
    // are present
    const std::size_t start_wp = edge.v1_idx + waypoint_offset;
    const std::size_t end_wp = edge.v2_idx + waypoint_offset;
    graph.add_lane({start_wp, entry_event}, {end_wp, exit_event});
    if (edge.edge_type == edge.EDGE_TYPE_BIDIRECTIONAL)
      graph.add_lane({end_wp, entry_event}, {start_wp, exit_event});
  }
  return graph;
}

} // namespace rmf_traffic_ros2
