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

#include "Server.hpp"

namespace rmf_task_ros2 {
namespace action {

std::shared_ptr<Server> Server::make(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& fleet_name)
{
  return std::shared_ptr<Server>(new Server(node, fleet_name));
}

//==============================================================================
void Server::register_callbacks(
  AddTaskCallback add_task_cb_fn,
  CancelTaskCallback cancel_task_cb_fn)
{
  _add_task_cb_fn = std::move(add_task_cb_fn);
  _cancel_task_cb_fn = std::move(cancel_task_cb_fn);
}

//==============================================================================
void Server::update_status(
  const TaskStatus& task_status)
{
  auto msg = convert_status(task_status);
  msg.fleet_name = _fleet_name;
  _status_msg_pub->publish(msg);
}

//==============================================================================
Server::Server(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& fleet_name)
: _node(node), _fleet_name(fleet_name)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_sub = _node->create_subscription<RequestMsg>(
    TaskRequestTopicName, dispatch_qos,
    [&](const std::unique_ptr<RequestMsg> msg)
    {
      if (msg->fleet_name != _fleet_name)
        return;// not me

      RCLCPP_INFO(_node->get_logger(),
      "[Action server] Received task request!");
      AckMsg ack_msg;
      ack_msg.dispatch_request = *msg;
      ack_msg.success = false;

      switch (msg->method)
      {
        // Add Request
        case RequestMsg::ADD:
          {
            if (_add_task_cb_fn && _add_task_cb_fn(msg->task_profile))
              ack_msg.success = true;
            break;
          }
        // Cancel Request
        case RequestMsg::CANCEL:
          {
            if (_cancel_task_cb_fn && _cancel_task_cb_fn(msg->task_profile))
              ack_msg.success = true;
            break;
          }
        default:
          RCLCPP_ERROR(_node->get_logger(), "Request Method is not supported!");
      }
      _ack_msg_pub->publish(ack_msg);
    });

  _ack_msg_pub = _node->create_publisher<AckMsg>(
    TaskAckTopicName, dispatch_qos);

  _status_msg_pub = _node->create_publisher<StatusMsg>(
    TaskStatusTopicName, dispatch_qos);
}

} // namespace action
} // namespace rmf_task_ros2
