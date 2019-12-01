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

#include <rmf_traffic/geometry/Circle.hpp>

#include <rclcpp/executors.hpp>

#include "../rmf_fleet_adapter/ParseArgs.hpp"

using namespace rmf_fleet_adapter;

int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string fleet_name;
  if (!get_arg(args, "-f", fleet_name, "fleet name"))
    return 1;

  const double v_nom = get_double_arg(args, "-v", "linear velocity", 0.7);
  const double w_nom = get_double_arg(args, "-w", "angular velocity", 0.3);
  const double a_nom = get_double_arg(args, "-a", "linear acceleration", 0.5);
  const double b_nom = get_double_arg(args, "-b", "angular acceleration", 1.5);
  const double r = get_double_arg(args, "-r", "profile radius", 0.6);

  const auto delay =
      get_time_arg(args, "-d", "delay threshold", 5.0);

  const auto quit_time =
      get_time_arg(args, "-q", "discovery timeout", 10.0);

  auto profile = rmf_traffic::Trajectory::Profile::make_guided(
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(r));

  rmf_traffic::agv::VehicleTraits traits{
    {v_nom, a_nom},
    {w_nom, b_nom},
    profile
  };

  const auto fleet_adapter_node =
      read_only::FleetAdapterNode::make(
        fleet_name, std::move(traits),
        delay, quit_time);

  if (!fleet_adapter_node)
    return 1;

  RCLCPP_INFO(fleet_adapter_node->get_logger(), "Starting Fleet Adapter");
  rclcpp::spin(fleet_adapter_node);
  RCLCPP_INFO(fleet_adapter_node->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();
}
