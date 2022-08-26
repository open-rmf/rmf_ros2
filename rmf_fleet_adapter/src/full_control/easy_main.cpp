/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

// Public rmf_fleet_adapter API headers
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

using EasyFullControl = rmf_fleet_adapter::agv::EasyFullControl;
using Configuration = rmf_fleet_adapter::agv::EasyFullControl::Configuration;


//==============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Temp: manually provide file path to config and nav graph
  std::string fleet_name = "test_adapter";
  std::string config_file = "/home/xiyu/efc_ws/install/rmf_demos/share/rmf_demos/config/office/tinyRobot_config.yaml";
  std::string nav_graph_path = "/home/xiyu/efc_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml";

  // Set up Configuration to easily parse parameters to Adapter
  auto adapter_config = Configuration("easy_fleet_adapter",
                                      fleet_name,
                                      config_file,
                                      nav_graph_path);

  const auto easy_adapter = rmf_fleet_adapter::agv::EasyFullControl::make(adapter_config);
  if (!easy_adapter)
    return 1;


  // TODO: add use_sim_time somewhere
}