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

#ifndef RMF_TASK_ROS2__DISPATCH_STATE_HPP
#define RMF_TASK_ROS2__DISPATCH_STATE_HPP

#include <unordered_map>
#include <memory>
#include <optional>

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_task_msgs/msg/dispatch_state.hpp>

#include <nlohmann/json.hpp>

namespace rmf_task_ros2 {

//==============================================================================
using TaskID = std::string;

//==============================================================================
/// \note TaskStatus struct is based on TaskSummary.msg
struct DispatchState
{
  using Msg = rmf_task_msgs::msg::DispatchState;

  enum class Status : uint8_t
  {
    /// This task has not been assigned yet
    Queued = Msg::STATUS_QUEUED,

    /// An assignment has been selected, but we have not received acknowledgment
    /// from the assignee
    Selected = Msg::STATUS_SELECTED,

    /// The task is dispatched and no longer being managed by the dispatcher
    Dispatched = Msg::STATUS_DISPATCHED,

    /// There was a failure to assign the task to anyone
    FailedToAssign = Msg::STATUS_FAILED_TO_ASSIGN,

    /// The task was canceled before it managed to get dispatched
    CanceledInFlight = Msg::STATUS_CANCELED_IN_FLIGHT
  };

  struct Assignment
  {
    std::string fleet_name;
    std::string expected_robot_name;
  };

  /// The Task ID for that this dispatching refers to
  std::string task_id;

  /// Submission arrival time
  rmf_traffic::Time submission_time;

  /// The status of this dispatching
  Status status = Status::Queued;

  /// The assignment that was made for this dispatching
  std::optional<Assignment> assignment;

  /// Any errors that have occurred for this dispatching
  std::vector<nlohmann::json> errors;

  /// task request form
  nlohmann::json request;

  DispatchState(std::string task_id, rmf_traffic::Time submission_time);
};

using DispatchStatePtr = std::shared_ptr<DispatchState>;

//==============================================================================
std::string status_to_string(DispatchState::Status status);

//==============================================================================
rmf_task_msgs::msg::Assignment convert(
  const std::optional<DispatchState::Assignment>& assignment);

//==============================================================================
rmf_task_msgs::msg::DispatchState convert(const DispatchState& state);

} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__DISPATCH_STATE_HPP
