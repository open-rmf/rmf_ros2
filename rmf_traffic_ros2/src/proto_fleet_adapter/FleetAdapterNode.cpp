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

#include "FleetAdapterNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace proto_fleet_adapter {

//==============================================================================
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make(
    std::string fleet_name,
    const std::string& graph_file,
    rmf_traffic::agv::VehicleTraits vehicle_traits,
    rmf_traffic::Duration wait_time)
{
  const auto start_time = std::chrono::steady_clock::now();
  std::shared_ptr<FleetAdapterNode> fleet_adapter(
        new FleetAdapterNode(std::move(fleet_name)));

  auto mirror_mgr_future = rmf_traffic_ros2::schedule::make_mirror(
        *fleet_adapter, rmf_traffic::schedule::Query::Spacetime());

  const YAML::Node graph_config = YAML::LoadFile(graph_file);
  if(!graph_config)
  {
    RCLCPP_ERROR(
          fleet_adapter->get_logger(),
          "Failed to load graph file [" + graph_file + "]");
    return nullptr;
  }

  const YAML::Node levels = graph_config["levels"];
  if(!levels)
  {
    RCLCPP_ERROR(
          fleet_adapter->get_logger(),
          "Graph file [" + graph_file + "] is missing the [levels] key");
    return nullptr;
  }

  if(!levels.IsMap())
  {
    RCLCPP_ERROR(
          fleet_adapter->get_logger(),
          "The [levels] key does not point to a map in graph file ["
          + graph_file + "]");
    return nullptr;
  }

  rmf_traffic::agv::Graph graph;
  std::unordered_map<std::string, std::size_t> waypoint_keys;
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
                fleet_adapter->get_logger(),
                "Duplicated waypoint name [" + name + "] in graph ["
                + graph_file + "]");
          return nullptr;
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
                fleet_adapter->get_logger(),
                "Unrecognized constraint label given to lane ["
                + std::to_string(lane[0].as<std::size_t>()) + ", "
                + std::to_string(lane[1].as<std::size_t>()) + "]: ["
                + lane[2].as<std::string>() + "] in graph ["
                + graph_file + "]");
          return nullptr;
        }
      }

      graph.add_lane(
          lane[0].as<std::size_t>(),
          {lane[1].as<std::size_t>(), std::move(constraint)});
    }
  }

  const auto stop_time = start_time + wait_time;
  while(rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    // NOTE(MXG): We need to spin the node in order to get the mirror manager
    // fully initialized.
    rclcpp::spin_some(fleet_adapter);

    using namespace std::chrono_literals;
    const auto status = mirror_mgr_future.wait_for(0s);
    if(std::future_status::ready == status)
    {
      fleet_adapter->start(
            Data{
              std::move(graph),
              std::move(waypoint_keys),
              std::move(vehicle_traits),
              mirror_mgr_future.get()
            });

      return fleet_adapter;
    }
  }

  RCLCPP_ERROR(
        fleet_adapter->get_logger(),
        "Mirror was not initialized in enough time ["
        + std::to_string(rmf_traffic::time::to_seconds(wait_time)) + "s]!");
  return nullptr;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode(std::string _fleet_name)
  : Node(_fleet_name + "_fleet_adapter"),
    fleet_name(std::move(_fleet_name))
{
  // Do nothing

  // NOTE(MXG): We need to initialize an empty node so we can spin up the
  // MirrorManagerFuture into a full MirrorManager. But also we don't want this
  // node to do anything until all its data fields are finalized, so this
  // constructor is more like a formality than a real constructor.
}

//==============================================================================
void FleetAdapterNode::start(Data _data)
{
  data = std::make_unique<Data>(std::move(_data));


}

} // namespace proto_fleet_adapter
