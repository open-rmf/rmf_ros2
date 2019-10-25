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

#include <rmf_traffic/geometry/Circle.hpp>

int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  const auto graph_arg = std::find(args.begin(), args.end(), "-g");
  if(graph_arg == args.end())
  {
    // TODO(MXG): See if there's a way to use RCLCPP_ERROR here without first
    // constructing a node. If not, we could consider constructing the
    // FleetAdapterNode in two parts.
    std::cerr << "You must specify a graph file using the -g argument!"
              << std::endl;
    return 1;
  }
  else if(graph_arg+1 == args.end())
  {
    std::cerr << "The -g argument must be followed by a graph file name!"
              << std::endl;
    return 1;
  }

  const std::string& graph_file = *(graph_arg+1);

  // TODO(MXG): Parse arguments for specifying vehicle traits and profile
  // properties, or allow the user to pass a yaml file describing the properties
  auto profile = rmf_traffic::Trajectory::Profile::make_guided(
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(0.5));

  rmf_traffic::agv::VehicleTraits traits{{0.7, 0.3}, {1.0, 0.45}, profile};

  const auto fleet_adapter_node =
    proto_fleet_adapter::FleetAdapterNode::make(graph_file, std::move(traits));
  if(!fleet_adapter_node)
  {
    std::cerr << "Failed to initialize the fleet adapter node" << std::endl;
    return 1;
  }

  rclcpp::spin(fleet_adapter_node);
  rclcpp::shutdown();
}
