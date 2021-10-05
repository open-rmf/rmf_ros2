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

#ifndef RMF_TASK_ROS2_SCHEDULER_INTERNAL_TASK_REQUEST_H
#define RMF_TASK_ROS2_SCHEDULER_INTERNAL_TASK_REQUEST_H

#include <string>
#include <vector>

#include <rmf_task_scheduler_msgs/msg/task_schedule_request.hpp>
#include <rmf_task_msgs/msg/task_description.hpp>
#include <rclcpp/time.hpp> 
#include <rclcpp/duration.hpp> 

namespace rmf_task_ros2 {

class TaskRequest
{
public:
  std::string _name;
  rclcpp::Time _start_time;
  std::optional<rclcpp::Time> _end_time;
  std::optional<rclcpp::Duration> _repeat_interval;
  std::string _task_type;
  std::string _args;

  static TaskRequest Make(
    const rmf_task_scheduler_msgs::msg::TaskScheduleRequest& req)
  {
    TaskRequest request;
    request._name = req.name;
    request._start_time = req.start_time;

    if (req.has_end_time)request.
    {
      request._end_time = {req.end_time};
    }
    else
    {
      request._end_time = std::nullopt;
    }

    if (req.has_repeat_interval)
    {
      request._repeat_interval = {req.repeat_interval};
    }
    else
    {
      request._repeat_interval = std::nullopt;
    }

    request._task_type = req.task_type;
    request._args = req.args; 

    return request;
  }
};
}
#endif