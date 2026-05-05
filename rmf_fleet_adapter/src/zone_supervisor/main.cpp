#include "Node.hpp"

#include <rclcpp/executors.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rmf_fleet_adapter::zone_supervisor::Node>();

  // MultiThreadedExecutor is required for future synchronous sensor service
  // calls. Using it from here ensures no executor change is
  // needed when sensor support is added. The MutuallyExclusive callback group
  // on zone request processing still ensures zone_log serialization.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
