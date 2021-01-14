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

#include <rmf_task_ros2/TaskStatus.hpp>
#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_task_ros2 {

// ==============================================================================
bool TaskStatus::is_terminated() const
{
  return (state == State::Failed) ||
    (state == State::Completed) ||
    (state == State::Canceled);
}

// ==============================================================================
TaskStatus convert_status(const StatusMsg& from)
{
  TaskStatus status;
  status.fleet_name = from.fleet_name;
  status.task_profile = from.task_profile;
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = static_cast<TaskStatus::State>(from.state);
  return status;
}

// ==============================================================================
StatusMsg convert_status(const TaskStatus& from)
{
  StatusMsg status;
  status.fleet_name = from.fleet_name;
  status.task_id = from.task_profile.task_id;  // duplication
  status.task_profile = from.task_profile;
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = static_cast<uint32_t>(from.state);
  return status;
}

} // namespace rmf_task_ros2
