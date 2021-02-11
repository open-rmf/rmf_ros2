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

#ifndef SRC__RMF_TASK_ROS2__ACTION__SERVER_HPP
#define SRC__RMF_TASK_ROS2__ACTION__SERVER_HPP

#include <rclcpp/node.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/TaskStatus.hpp>
#include <rmf_task_msgs/msg/dispatch_request.hpp>
#include <rmf_task_msgs/msg/dispatch_ack.hpp>
#include <rmf_traffic/Time.hpp>

using TaskProfile = rmf_task_msgs::msg::TaskProfile;

namespace rmf_task_ros2 {
namespace action {

//==============================================================================
/// Task Action Server - This is used within the fleet adapter with the role of
/// receiving incoming dispatch requests (from a action_client/Dispatcher),
/// then execute the task accordingly.
class Server
{
public:
  /// initialize action server
  ///
  /// \param[in] node
  ///   ros2 node instance
  ///
  /// \param[in] fleet_name
  ///   action server id (e.g. fleet adapter name)
  static std::shared_ptr<Server> make(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name);

  using AddTaskCallback = std::function<bool(const TaskProfile& task)>;
  using CancelTaskCallback = std::function<bool(const TaskProfile& task)>;

  /// Add event callback functions. These functions will be triggered when a
  /// relevant task requests msg is received.
  ///
  /// \param[in] add_task_cb
  ///   When a new task request is called.
  ///
  /// \param[in] cancel_task_cb
  ///   when a cancel task request is called
  void register_callbacks(
    AddTaskCallback add_task_cb,
    CancelTaskCallback cancel_task_cb);

  /// Use this to send a status update to action client
  /// A On Change update is recommended to inform the task progress
  ///
  /// \param[in] task_status
  ///   latest status of the task
  void update_status(const TaskStatus& task_status);

private:
  Server(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name);

  using RequestMsg = rmf_task_msgs::msg::DispatchRequest;
  using AckMsg = rmf_task_msgs::msg::DispatchAck;

  std::shared_ptr<rclcpp::Node> _node;
  std::string _fleet_name;
  AddTaskCallback _add_task_cb_fn;
  CancelTaskCallback _cancel_task_cb_fn;
  rclcpp::Subscription<RequestMsg>::SharedPtr _request_msg_sub;
  rclcpp::Publisher<StatusMsg>::SharedPtr _status_msg_pub;
  rclcpp::Publisher<AckMsg>::SharedPtr _ack_msg_pub;
};

} // namespace action
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__ACTION__SERVER_HPP
