/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "ScheduleNode.hpp"

namespace rmf_traffic_schedule {

//==============================================================================
ScheduleNode::ScheduleNode()
  : Node("rmf_traffic_schedule_node")
{
  // TODO(MXG): As soon as possible, all of these services should be made
  // multi-threaded so they can be parallel processed.

  submit_trajectory_service =
      create_service<rmf_traffic_msgs::srv::SubmitTrajectory>(
        "rmf_traffic/submit_trajectory",
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const SubmitTrajectory::Request::SharedPtr request,
            const SubmitTrajectory::Response::SharedPtr response)
        { this->submit_trajectory(request_header, request, response); });

  erase_schedule_service =
      create_service<rmf_traffic_msgs::srv::EraseSchedule>(
        "rmf_traffic/erase_schedule",
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const EraseSchedule::Request::SharedPtr request,
            const EraseSchedule::Response::SharedPtr response)
        { this->erase_schedule(request_header, request, response); });
}

//==============================================================================
void ScheduleNode::submit_trajectory(
    const std::shared_ptr<rmw_request_id_t>& request_header,
    const SubmitTrajectory::Request::SharedPtr& request,
    const SubmitTrajectory::Response::SharedPtr& response)
{

}

//==============================================================================
void ScheduleNode::erase_schedule(
    const std::shared_ptr<rmw_request_id_t>& request_header,
    const EraseSchedule::Request::SharedPtr& request,
    const EraseSchedule::Response::SharedPtr& response)
{

}

} // namespace rmf_traffic_schedule
