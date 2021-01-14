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

#include "Client.hpp"

namespace rmf_task_ros2 {
namespace action {

std::shared_ptr<Client>
Client::make(std::shared_ptr<rclcpp::Node> node)
{
  return std::shared_ptr<Client>(new Client(node));
}

//==============================================================================
Client::Client(std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_pub = _node->create_publisher<RequestMsg>(
    TaskRequestTopicName, dispatch_qos);

  _status_msg_sub = _node->create_subscription<StatusMsg>(
    TaskStatusTopicName, dispatch_qos,
    [&](const std::unique_ptr<StatusMsg> msg)
    {
      auto task_id = msg->task_profile.task_id;
      // status update mode
      if (_active_task_status.count(task_id))
      {
        auto weak_status = _active_task_status[task_id].lock();

        if (!weak_status)
        {
          RCLCPP_INFO(_node->get_logger(), "Task was previously terminated");
          _active_task_status.erase(task_id);
          return;
        }

        // TODO: hack to retain task profile and fleet name (to remove)
        auto cache_profile = weak_status->task_profile;
        // update status to ptr
        *weak_status = convert_status(*msg);
        weak_status->task_profile = cache_profile;

        if (_on_change_callback)
          _on_change_callback(weak_status);

        // if active task terminated
        if (weak_status->is_terminated())
        {
          RCLCPP_INFO(_node->get_logger(),
          "Receive status from fleet [%s], task [%s] is now terminated",
          msg->fleet_name.c_str(), task_id.c_str());
          _active_task_status.erase(task_id);

          if (_on_terminate_callback)
            _on_terminate_callback(weak_status);
        }
      }
      else
      {
        // will still provide onchange even if the task_id is unknown.
        RCLCPP_DEBUG(_node->get_logger(),
        "[action] Unknown task: [%s]", task_id.c_str());
        auto task_status = std::make_shared<TaskStatus>(convert_status(*msg));

        if (_on_change_callback)
          _on_change_callback(task_status);

        if (!task_status->is_terminated())
          _active_task_status[task_id] = task_status;
      }
    });
}

//==============================================================================
void Client::add_task(
  const std::string& fleet_name,
  const TaskProfile& task_profile,
  TaskStatusPtr status_ptr)
{
  // send request and wait for acknowledgement
  RequestMsg request_msg;
  request_msg.fleet_name = fleet_name;
  request_msg.task_profile = task_profile;
  request_msg.method = RequestMsg::ADD;
  _request_msg_pub->publish(request_msg);

  // save status ptr
  status_ptr->fleet_name = fleet_name;
  status_ptr->task_profile = task_profile;
  _active_task_status[task_profile.task_id] = status_ptr;
  RCLCPP_DEBUG(_node->get_logger(), "Assign task: [%s] to fleet [%s]",
    task_profile.task_id.c_str(), fleet_name.c_str());
  return;
}

//==============================================================================
bool Client::cancel_task(
  const TaskProfile& task_profile)
{
  const auto task_id = task_profile.task_id;
  RCLCPP_DEBUG(_node->get_logger(),
    "[action] Cancel Task: [%s]", task_id.c_str());

  // check if task is previously added
  if (!_active_task_status.count(task_id))
  {
    RCLCPP_WARN(_node->get_logger(),
      "Canceling an unknown task [%s]", task_id.c_str());
    return false;
  }

  auto weak_status = _active_task_status[task_id].lock();

  if (!weak_status)
  {
    RCLCPP_WARN(_node->get_logger(), "Task was previously terminated");
    _active_task_status.erase(task_id);
    return false;
  }

  // send cancel
  RequestMsg request_msg;
  request_msg.fleet_name = weak_status->fleet_name;
  request_msg.task_profile = task_profile;
  request_msg.method = RequestMsg::CANCEL;
  _request_msg_pub->publish(request_msg);
  return true;
}

//==============================================================================
int Client::size()
{
  for (auto it = _active_task_status.begin(); it != _active_task_status.end(); )
  {
    if (auto weak_status = it->second.lock() )
    {
      if (weak_status->is_terminated())
        it = _active_task_status.erase(it);
      else
        ++it;
    }
    else
      it = _active_task_status.erase(it);
  }
  return _active_task_status.size();
}

//==============================================================================
void Client::on_change(
  StatusCallback status_cb_fn)
{
  _on_change_callback = std::move(status_cb_fn);
}

//==============================================================================
void Client::on_terminate(
  StatusCallback status_cb_fn)
{
  _on_terminate_callback = std::move(status_cb_fn);
}

} // namespace action
} // namespace rmf_task_ros2
