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

#ifndef RMF_TASK_ROS2__TASK_STATUS_HPP
#define RMF_TASK_ROS2__TASK_STATUS_HPP

#include <unordered_map>
#include <rmf_traffic/Time.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>
#include <rmf_task_msgs/msg/task_summary.hpp>

namespace rmf_task_ros2 {

//==============================================================================
using TaskProfile = rmf_task_msgs::msg::TaskProfile;
using StatusMsg = rmf_task_msgs::msg::TaskSummary;
using TaskID = std::string;

//==============================================================================
/// \note TaskStatus struct is based on TaskSummary.msg
struct TaskStatus
{
  enum class State : uint8_t
  {
    Queued    = StatusMsg::STATE_QUEUED,
    Executing = StatusMsg::STATE_ACTIVE,
    Completed = StatusMsg::STATE_COMPLETED,
    Failed    = StatusMsg::STATE_FAILED,
    Canceled  = StatusMsg::STATE_CANCELED,
    Pending   = StatusMsg::STATE_PENDING
  };

  std::string fleet_name;
  TaskProfile task_profile;
  rmf_traffic::Time start_time;
  rmf_traffic::Time end_time;
  std::string robot_name;
  std::string status; // verbose msg
  State state = State::Pending; // default

  bool is_terminated() const;
};

using TaskStatusPtr = std::shared_ptr<TaskStatus>;

// ==============================================================================
TaskStatus convert_status(const StatusMsg& from);

// ==============================================================================
StatusMsg convert_status(const TaskStatus& from);

} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__TASK_STATUS_HPP
