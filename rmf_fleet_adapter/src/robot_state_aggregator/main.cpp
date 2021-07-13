// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#define LINKTIME_COMPOSITION_LOGGER_NAME "linktime_composition"

int main(int argc, char* argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::Logger logger = rclcpp::get_logger(LINKTIME_COMPOSITION_LOGGER_NAME);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  std::vector<class_loader::ClassLoader*> loaders;
  std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

  std::vector<std::string> libraries =
  {
    // all classes from libraries linked by the linker (rather then dlopen)
    // are registered under the library_path ""
    "",
  };

  //Easy way to catch the failover parameter
  auto dummy_node = std::make_shared<rclcpp::Node>("dumy_node");
  bool failover_mode = dummy_node->declare_parameter("failover_mode", false);
  dummy_node.reset();

#ifndef FAILOVER_MODE
  if (failover_mode)
  {
    RCLCPP_ERROR(logger, "robot_state_aggregator was compiled without failover"
      " support. Make sure you have the required libraries and the"
      " environment variable $RMF_ENABLE_FAILOVER is set during"
      " compilation.");
    return 1;
  }
#endif

  for (auto library : libraries)
  {
    auto loader = new class_loader::ClassLoader(library);
    auto classes =
      loader->getAvailableClasses<rclcpp_components::NodeFactory>();

    for (auto clazz : classes)
    {
      if (failover_mode ||
        ((clazz.compare("rclcpp_components::NodeFactoryTemplate"
        "<lifecycle_heartbeat::LifecycleHeartbeat>") != 0) &&
        ((clazz.compare("rclcpp_components::NodeFactoryTemplate"
        "<lifecycle_watchdog::LifecycleWatchdog>") != 0))))
      {
        auto node_factory =
          loader->createInstance<rclcpp_components::NodeFactory>(clazz);
        auto wrapper = node_factory->create_node_instance(options);
        auto node = wrapper.get_node_base_interface();

        node_wrappers.push_back(wrapper);
        exec.add_node(node);
      }

    }
    loaders.push_back(loader);
  }
  exec.spin();

  for (auto wrapper : node_wrappers)
  {
    exec.remove_node(wrapper.get_node_base_interface());
  }
  node_wrappers.clear();

  rclcpp::shutdown();

  return 0;
}
