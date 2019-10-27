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

#include "ParseGraph.hpp"

#include <yaml-cpp/yaml.h>

namespace proto_fleet_adapter {
//==============================================================================
bool parse_graph(
    const std::string& graph_file,
    const rmf_traffic::agv::VehicleTraits& vehicle_traits,
    const rclcpp::Node& node,
    rmf_traffic::agv::Graph& graph,
    std::unordered_map<std::string, std::size_t>& waypoint_keys)
{
  const YAML::Node graph_config = YAML::LoadFile(graph_file);
  if(!graph_config)
  {
    RCLCPP_ERROR(
          node.get_logger(),
          "Failed to load graph file [" + graph_file + "]");
    return false;
  }

  const YAML::Node levels = graph_config["levels"];
  if(!levels)
  {
    RCLCPP_ERROR(
          node.get_logger(),
          "Graph file [" + graph_file + "] is missing the [levels] key");
    return false;
  }

  if(!levels.IsMap())
  {
    RCLCPP_ERROR(
          node.get_logger(),
          "The [levels] key does not point to a map in graph file ["
          + graph_file + "]");
    return false;
  }

  for(const auto& level : levels)
  {
    const std::string& map_name = level.first.as<std::string>();

    const YAML::Node& vertices = level.second["vertices"];
    for(const auto& vertex : vertices)
    {
      const Eigen::Vector2d location{
        vertex[0].as<double>(), vertex[1].as<double>()};

      const auto& wp = graph.add_waypoint(map_name, location, true);

      const std::string& name = vertex[2].as<std::string>();
      if(!name.empty())
      {
        const auto ins = waypoint_keys.insert(std::make_pair(name, wp.index()));
        if(!ins.second)
        {
          RCLCPP_ERROR(
                node.get_logger(),
                "Duplicated waypoint name [" + name + "] in graph ["
                + graph_file + "]");
          return false;
        }
      }
    }

    const YAML::Node& lanes = level.second["lanes"];
    for(const auto& lane : lanes)
    {
      using Constraint = rmf_traffic::agv::Graph::OrientationConstraint;
      using ConstraintPtr = std::unique_ptr<Constraint>;

      const std::string& constraint_label = lane[2].as<std::string>();
      ConstraintPtr constraint = nullptr;
      if(!constraint_label.empty())
      {
        if(constraint_label == "forward")
        {
          constraint = Constraint::make(
                Constraint::Direction::Forward,
                vehicle_traits.get_differential()->get_forward());
        }
        else if(constraint_label == "backward")
        {
          constraint = Constraint::make(
                Constraint::Direction::Backward,
                vehicle_traits.get_differential()->get_forward());
        }
        else
        {
          RCLCPP_ERROR(
                node.get_logger(),
                "Unrecognized constraint label given to lane ["
                + std::to_string(lane[0].as<std::size_t>()) + ", "
                + std::to_string(lane[1].as<std::size_t>()) + "]: ["
                + lane[2].as<std::string>() + "] in graph ["
                + graph_file + "]");
          return false;
        }
      }

      graph.add_lane(
          lane[0].as<std::size_t>(),
          {lane[1].as<std::size_t>(), std::move(constraint)});
    }
  }

  return true;
}
} // namespace proto_fleet_adapter
