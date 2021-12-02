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

#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
rmf_traffic::agv::Graph parse_graph(
  const std::string& graph_file,
  const rmf_traffic::agv::VehicleTraits& vehicle_traits)
{
  const YAML::Node graph_config = YAML::LoadFile(graph_file);
  if (!graph_config)
  {
    throw std::runtime_error("Failed to load graph file [" + graph_file + "]");
  }

  const YAML::Node levels = graph_config["levels"];
  if (!levels)
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "Graph file [" + graph_file + "] is missing the [levels] key");
    // *INDENT-ON*
  }

  if (!levels.IsMap())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "The [levels] key does not point to a map in graph file ["
      + graph_file + "]");
    // *INDENT-ON*
  }

  using Constraint = rmf_traffic::agv::Graph::OrientationConstraint;
  using ConstraintPtr = rmf_utils::clone_ptr<Constraint>;
  using Lane = rmf_traffic::agv::Graph::Lane;
  using Event = Lane::Event;

  rmf_traffic::agv::Graph graph;
  std::unordered_map<std::string, std::vector<std::size_t>> wps_of_lift;
  std::unordered_map<std::size_t, std::string> lift_of_wp;
  std::size_t vnum = 0;  // To increment lane endpoint ids

  for (const auto& level : levels)
  {
    const std::string& map_name = level.first.as<std::string>();
    std::size_t vnum_temp = 0;

    const YAML::Node& vertices = level.second["vertices"];
    for (const auto& vertex : vertices)
    {
      const Eigen::Vector2d location{
        vertex[0].as<double>(), vertex[1].as<double>()};

      auto& wp = graph.add_waypoint(map_name, location);

      const YAML::Node& options = vertex[2];
      const YAML::Node& name_option = options["name"];
      if (name_option)
      {
        const std::string& name = name_option.as<std::string>();
        if (!name.empty())
        {
          if (!graph.add_key(name, wp.index()))
          {
            // *INDENT-OFF*
            throw std::runtime_error(
              "Duplicated waypoint name [" + name + "] in graph ["
              + graph_file + "]");
            // *INDENT-ON*
          }
        }
      }
      vnum_temp ++;

      const YAML::Node& parking_spot_option = options["is_parking_spot"];
      if (parking_spot_option)
      {
        const bool is_parking_spot = parking_spot_option.as<bool>();
        if (is_parking_spot)
          wp.set_parking_spot(true);
      }

      const YAML::Node& holding_point_option = options["is_holding_point"];
      if (holding_point_option)
      {
        const bool is_holding_point = holding_point_option.as<bool>();
        if (is_holding_point)
          wp.set_holding_point(true);
      }

      const YAML::Node& passthrough_option = options["is_passthrough_point"];
      if (passthrough_option)
      {
        const bool is_passthrough_point = passthrough_option.as<bool>();
        if (is_passthrough_point)
          wp.set_passthrough_point(true);
      }

      const YAML::Node& charger_option = options["is_charger"];
      if (charger_option)
      {
        const bool is_charger = charger_option.as<bool>();
        if (is_charger)
          wp.set_charger(true);
      }

      const YAML::Node& lift_option = options["lift"];
      if (lift_option)
      {
        const std::string lift_name = lift_option.as<std::string>();
        if (lift_name != "")
        {
          wps_of_lift[lift_name].push_back(wp.index());
          lift_of_wp[wp.index()] = lift_name;
        }
      }
    }

    const YAML::Node& lanes = level.second["lanes"];
    for (const auto& lane : lanes)
    {

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
          // *INDENT-OFF*
          throw std::runtime_error(
            "Unrecognized orientation constraint label given to lane ["
            + std::to_string(lane[0].as<std::size_t>() + vnum) + ", "
            + std::to_string(lane[1].as<std::size_t>() + vnum) + "]: ["
            + constraint_label + "] in graph ["
            + graph_file + "]");
          // *INDENT-ON*
        }
      }

      rmf_utils::clone_ptr<Event> entry_event;
      rmf_utils::clone_ptr<Event> exit_event;
      std::size_t begin = lane[0].as<std::size_t>() + vnum;
      std::size_t end = lane[1].as<std::size_t>() + vnum;

      const auto lift_of_begin = lift_of_wp.find(begin);
      const auto lift_of_end = lift_of_wp.find(end);

      const bool begin_in_lift = lift_of_begin != lift_of_wp.end();
      const bool end_in_lift = lift_of_end != lift_of_wp.end();

      const bool is_lift = begin_in_lift || end_in_lift;
      if (is_lift)
      {
        const rmf_traffic::Duration duration = std::chrono::seconds(4);
        if (!begin_in_lift && end_in_lift)
        {
          // Entering lift
          const std::string& lift_name = lift_of_end->second;
          entry_event = Event::make(
            Lane::LiftSessionBegin(lift_name, map_name, duration));
        }
        else if (begin_in_lift && end_in_lift)
        {
          if (lift_of_begin->second != lift_of_end->second)
          {
            // If these are two lift waypoints on the same floor, then they
            // should be inside the same lift
            // *INDENT-OFF*
            throw std::runtime_error(
              "Inconsistency in building map. Map [" + map_name + "] has two "
              "connected waypoints [" + std::to_string(
                begin) + " -> "
              + std::to_string(end) + "] that are in different lifts ["
              + lift_of_begin->second + " -> " + lift_of_end->second
              + "]. This is not supported!");
            // *INDENT-ON*
          }

          // If we make it here, then both waypoints are inside the same lift,
          // so we don't need any event for the robots to move between these
          // waypoints.
        }
        else if (begin_in_lift && !end_in_lift)
        {
          // Exiting lift
          const std::string& lift_name = lift_of_begin->second;
          entry_event = Event::make(
            Lane::LiftDoorOpen(lift_name, map_name, duration));
          exit_event = Event::make(
            Lane::LiftSessionEnd(lift_name, map_name,
            rmf_traffic::Duration(0)));
        }
      }
      else
      {
        if (const YAML::Node mock_lift_option = options["demo_mock_floor_name"])
        {
          // NOTE: This is specifically for cases where users want to have a
          // mock lift in the map. It should not be used for real lifts.
          const std::string floor_name = mock_lift_option.as<std::string>();
          const YAML::Node lift_name_option = options["demo_mock_lift_name"];

          if (!lift_name_option)
          {
            // *INDENT-OFF*
            throw std::runtime_error(
              "Missing [demo_mock_lift_name] parameter which is required for "
              "mock lifts");
            // *INDENT-ON*
          }

          // TODO(MXG): This implementation is not air tight. After a robot has
          // entered the lift, a second robot could start a new lift session,
          // which would cause problems for the robot that's in the lift.
          //
          // We will need to rework this implementation if we ever need to do
          // a demo where multiple robots negotiate the use of a mock lift.
          const std::string lift_name = lift_name_option.as<std::string>();
          const rmf_traffic::Duration duration = std::chrono::seconds(4);
          entry_event = Event::make(
            Lane::LiftSessionBegin(lift_name, floor_name, duration));
          exit_event = Event::make(
            Lane::LiftSessionEnd(lift_name, floor_name,
            rmf_traffic::Duration(0)));
        }
        else if (const YAML::Node door_name_option = options["door_name"])
        {
          const std::string name = door_name_option.as<std::string>();
          const rmf_traffic::Duration duration = std::chrono::seconds(4);
          entry_event = Event::make(Lane::DoorOpen(name, duration));
          exit_event = Event::make(Lane::DoorClose(name, duration));
        }
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

      auto& graph_lane = graph.add_lane(
        {begin, entry_event},
        {end, exit_event, std::move(constraint)});

      if (const YAML::Node speed_limit_option = options["speed_limit"])
      {
        const double speed_limit = speed_limit_option.as<double>();
        if (speed_limit > 0.0)
          graph_lane.properties().speed_limit(speed_limit);
      }
    }
    vnum += vnum_temp;
  }

  for (const auto& lift : wps_of_lift)
  {
    const auto& wps = lift.second;
    for (std::size_t i = 0; i < wps.size()-1; i++)
    {
      rmf_utils::clone_ptr<Event> entry_event;
      rmf_utils::clone_ptr<Event> exit_event;
      const rmf_traffic::Duration duration = std::chrono::seconds(1);

      entry_event = Event::make(Lane::LiftMove(
            lift.first, graph.get_waypoint(wps[i+1]).get_map_name(), duration));
      graph.add_lane(
        {wps[i], entry_event},
        {wps[i+1], exit_event});

      entry_event = Event::make(Lane::LiftMove(
            lift.first, graph.get_waypoint(wps[i]).get_map_name(), duration));
      graph.add_lane(
        {wps[i+1], entry_event},
        {wps[i], exit_event});
    }
  }

  return graph;
}

} // namespace agv
} // namespace rmf_fleet_adapter
