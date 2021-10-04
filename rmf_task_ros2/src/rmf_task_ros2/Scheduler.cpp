/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rmf_task_scheduler_msgs/msg/task_schedule_cancel.hpp>
#include <rmf_task_scheduler_msgs/msg/task_schedule_request.hpp>
#include <rmf_task_scheduler_msgs/msg/task_schedule_response.hpp>
#include <rmf_task_scheduler_msgs/msg/task_schedule_rules.hpp>

#include <rmf_task_ros2/Scheduler.hpp>
#include <rmf_task_ros2/StandardNames.hpp>

using namespace rmf_task_ros2;

class Scheduler::Implementation
{
using TaskCancelMsg = rmf_task_scheduler_msgs::msg::TaskScheduleCancel;
using TaskRequestMsg = rmf_task_scheduler_msgs::msg::TaskScheduleRequest;
using TaskResponseMsg = rmf_task_scheduler_msgs::msg::TaskScheduleResponse;
using TaskRuleMsg = rmf_task_scheduler_msgs::msg::TaskScheduleRule;
using TaskRulesMsg = rmf_task_scheduler_msgs::msg::TaskScheduleRules;

public:
std::shared_ptr<rclcpp::Publisher<TaskRulesMsg>> _status_pub;

std::shared_ptr<rclcpp::Publisher<TaskResponseMsg>> _response_pub;

std::shared_ptr<rclcpp::Subscription<TaskRequestMsg>> _request_sub;

std::shared_ptr<rclcpp::Subscription<TaskCancelMsg>> _cancel_sub;

void on_cancel_request(const TaskCancelMsg::SharedPtr msg)
{

}

void on_receive_request(const TaskRequestMsg::SharedPtr msg)
{

} 

Implementation(const std::shared_ptr<rclcpp::Node>& node)
{
  _cancel_sub =
    node->create_subscription<TaskCancelMsg>(TaskScheduleCancel, 10,
      std::bind(&Scheduler::Implementation::on_cancel_request, this, std::placeholders::_1));
  _request_sub =
    node->create_subscription<TaskRequestMsg>(TaskScheduleRequests, 10,
      std::bind(&Scheduler::Implementation::on_receive_request, this, std::placeholders::_1));

  _status_pub =
    node->create_publisher<TaskRulesMsg>(TaskScheduleState, 10);
  _response_pub =
    node->create_publisher<TaskResponseMsg>(TaskScheduleResponses, 10);
}
};

std::unique_ptr<Scheduler> Scheduler::make(
  const std::shared_ptr<rclcpp::Node>& node)
{
  auto pimpl = rmf_utils::make_impl<Implementation>(node);
  auto sched = std::make_unique<Scheduler>();
  sched->_pimpl = std::move(pimpl);

  return sched;
}
