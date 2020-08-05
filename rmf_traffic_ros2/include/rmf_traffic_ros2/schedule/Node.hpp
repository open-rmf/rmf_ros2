#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__NODE_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__NODE_HPP

#include <rclcpp/node.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

/// Make a ScheduleNode instance
std::shared_ptr<rclcpp::Node> make_node(
  const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__NODE_HPP
