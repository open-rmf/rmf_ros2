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

#include <unordered_set>

namespace rmf_fleet_adapter {

//==============================================================================
rmf_utils::optional<GraphInfo> parse_graph(
  const std::string& graph_file,
  const rmf_traffic::agv::VehicleTraits& vehicle_traits,
  const rclcpp::Node& node)
{
  const YAML::Node graph_config = YAML::LoadFile(graph_file);
  if (!graph_config)
  {
    RCLCPP_ERROR(
      node.get_logger(),
      "Failed to load graph file [" + graph_file + "]");
    return rmf_utils::nullopt;
  }

  const YAML::Node levels = graph_config["levels"];
  if (!levels)
  {
    RCLCPP_ERROR(
      node.get_logger(),
      "Graph file [" + graph_file + "] is missing the [levels] key");
    return rmf_utils::nullopt;
  }

  if (!levels.IsMap())
  {
    RCLCPP_ERROR(
      node.get_logger(),
      "The [levels] key does not point to a map in graph file ["
      + graph_file + "]");
    return rmf_utils::nullopt;
  }

  GraphInfo info;

  for (const auto& level : levels)
  {
    const std::string& map_name = level.first.as<std::string>();

    const YAML::Node& vertices = level.second["vertices"];
    for (const auto& vertex : vertices)
    {
      const Eigen::Vector2d location{
        vertex[0].as<double>(), vertex[1].as<double>()};

      auto& wp = info.graph.add_waypoint(map_name, location);

      const YAML::Node& options = vertex[2];
      const YAML::Node& name_option = options["name"];
      if (name_option)
      {
        const std::string& name = name_option.as<std::string>();
        if (!name.empty())
        {
          if (!info.graph.add_key(name, wp.index()))
          {
            RCLCPP_ERROR(
              node.get_logger(),
              "Duplicated waypoint name [" + name + "] in graph ["
              + graph_file + "]");
            return rmf_utils::nullopt;
          }
        }
      }

      const YAML::Node& workcell_name_option = options["workcell_name"];
      if (workcell_name_option)
      {
        const std::string& workcell = workcell_name_option.as<std::string>();
        info.workcell_names.insert({wp.index(), workcell});
      }

      const YAML::Node& parking_spot_option = options["is_parking_spot"];
      if (parking_spot_option)
      {
        const bool is_parking_spot = parking_spot_option.as<bool>();
        if (is_parking_spot)
        {
          std::cout << "Adding waypoint [" << wp.index() <<
            "] as a parking spot" << std::endl;
          info.parking_spots.push_back(wp.index());
          wp.set_parking_spot(true);
        }
      }
    }

    const YAML::Node& lanes = level.second["lanes"];
    for (const auto& lane : lanes)
    {
      using Constraint = rmf_traffic::agv::Graph::OrientationConstraint;
      using ConstraintPtr = rmf_utils::clone_ptr<Constraint>;

      ConstraintPtr constraint = nullptr;

      const YAML::Node& options = lane[2];
      const YAML::Node& orientation_constraint_option =
        options["orientation_constraint"];
      if (orientation_constraint_option)
      {
        const std::string& constraint_label =
          orientation_constraint_option.as<std::string>();
        if (constraint_label == "forward")
        {
          constraint = Constraint::make(
            Constraint::Direction::Forward,
            vehicle_traits.get_differential()->get_forward());
        }
        else if (constraint_label == "backward")
        {
          constraint = Constraint::make(
            Constraint::Direction::Backward,
            vehicle_traits.get_differential()->get_forward());
        }
        else
        {
          RCLCPP_ERROR(
            node.get_logger(),
            "Unrecognized orientation constraint label given to lane ["
            + std::to_string(lane[0].as<std::size_t>()) + ", "
            + std::to_string(lane[1].as<std::size_t>()) + "]: ["
            + constraint_label + "] in graph ["
            + graph_file + "]");
          return rmf_utils::nullopt;
        }
      }

      using Lane = rmf_traffic::agv::Graph::Lane;
      using Event = Lane::Event;
      rmf_utils::clone_ptr<Event> entry_event;
      rmf_utils::clone_ptr<Event> exit_event;
      if (const YAML::Node mock_lift_option = options["demo_mock_floor_name"])
      {
        // TODO(MXG): Replace this with a key like lift_name when we have proper
        // support for lifts.
        const std::string floor_name = mock_lift_option.as<std::string>();
        const YAML::Node lift_name_option = options["demo_mock_lift_name"];

        if (!lift_name_option)
        {
          RCLCPP_ERROR(
            node.get_logger(),
            "Missing [demo_mock_lift_name] parameter which is required for "
            "mock lifts");
          return rmf_utils::nullopt;
        }

        const std::string lift_name = lift_name_option.as<std::string>();
        const rmf_traffic::Duration duration = std::chrono::seconds(4);
        entry_event = Event::make(
          Lane::LiftDoorOpen(lift_name, floor_name, duration));
        // NOTE(MXG): We do not need an exit event for lifts
      }
      else if (const YAML::Node door_name_option = options["door_name"])
      {
        const std::string name = door_name_option.as<std::string>();
        const rmf_traffic::Duration duration = std::chrono::seconds(4);
        entry_event = Event::make(Lane::DoorOpen(name, duration));
        exit_event = Event::make(Lane::DoorClose(name, duration));
      }

      if (const YAML::Node docking_option = options["dock_name"])
      {
        // TODO(MXG): Add support for this
        if (entry_event || exit_event)
        {
          // *INDENT-OFF*
          throw std::runtime_error(
            "We do not currently support a dock_name option when any other "
            "lane options are also specified");
          // *INDENT-ON*
        }

        const std::string dock_name = docking_option.as<std::string>();
        const rmf_traffic::Duration duration = std::chrono::seconds(5);
        entry_event = Event::make(Lane::Dock(dock_name, duration));
      }

      info.graph.add_lane(
        {lane[0].as<std::size_t>(), entry_event},
        {lane[1].as<std::size_t>(), exit_event, std::move(constraint)});
    }
  }

  std::unordered_set<std::size_t> generic_waypoint;
  for (std::size_t i = 0; i < info.graph.num_waypoints(); ++i)
    generic_waypoint.insert(i);

  for (std::size_t i = 0; i < info.graph.num_lanes(); ++i)
  {
    const auto& lane = info.graph.get_lane(i);
    if (lane.entry().event())
      generic_waypoint.erase(lane.entry().waypoint_index());

    if (lane.exit().event())
      generic_waypoint.erase(lane.exit().waypoint_index());
  }

  for (const auto& workcell_wp : info.workcell_names)
    generic_waypoint.erase(workcell_wp.first);

  std::cout << "Named waypoints:";
  for (const auto& key : info.graph.keys())
    std::cout << "\n -- [" << key.first << "]";
  std::cout << std::endl;

  return std::move(info);
}

} // namespace rmf_fleet_adapter
