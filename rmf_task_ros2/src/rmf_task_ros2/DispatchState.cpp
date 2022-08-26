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

#include <rmf_task_ros2/DispatchState.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_task_msgs/msg/assignment.hpp>

namespace rmf_task_ros2 {

//==============================================================================
DispatchState::DispatchState(
  std::string task_id_,
  rmf_traffic::Time submission_time_)
: task_id(std::move(task_id_)),
  submission_time(submission_time_)
{
  // Do nothing
}

//==============================================================================
std::string status_to_string(DispatchState::Status status)
{
  using Status = DispatchState::Status;
  switch (status)
  {
    case Status::Queued:
      return "queued";
    case Status::Selected:
      return "selected";
    case Status::Dispatched:
      return "dispatched";
    case Status::FailedToAssign:
      return "failed_to_assign";
    case Status::CanceledInFlight:
      return "canceled_in_flight";
    default:
      return "failed_to_assign";
  }
}

//=============================================================================
rmf_task_msgs::msg::Assignment convert(
  const std::optional<DispatchState::Assignment>& assignment)
{
  if (assignment.has_value())
  {
    return rmf_task_msgs::build<rmf_task_msgs::msg::Assignment>()
      .is_assigned(true)
      .fleet_name(assignment->fleet_name)
      .expected_robot_name(assignment->expected_robot_name);
  }

  return rmf_task_msgs::build<rmf_task_msgs::msg::Assignment>()
    .is_assigned(false)
    .fleet_name("")
    .expected_robot_name("");
}

//=============================================================================
std::vector<std::string> convert(const std::vector<nlohmann::json>& values)
{
  std::vector<std::string> output;
  output.reserve(values.size());
  for (const auto& v : values)
    output.push_back(v.dump());

  return output;
}

//=============================================================================
rmf_task_msgs::msg::DispatchState convert(const DispatchState& state)
{
  return rmf_task_msgs::build<rmf_task_msgs::msg::DispatchState>()
    .task_id(state.task_id)
    .status(static_cast<uint8_t>(state.status))
    .assignment(convert(state.assignment))
    .errors(convert(state.errors));
}

} // namespace rmf_task_ros2
