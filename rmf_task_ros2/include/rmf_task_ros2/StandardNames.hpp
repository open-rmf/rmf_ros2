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

#ifndef RMF_TASK_ROS2__STANDARDNAMES_HPP
#define RMF_TASK_ROS2__STANDARDNAMES_HPP

#include <string>

namespace rmf_task_ros2 {

const std::string Prefix = "rmf_task/";
const std::string BidNoticeTopicName = Prefix + "bid_notice";
const std::string BidResponseTopicName = Prefix + "bid_response";

const std::string SubmitTaskSrvName = "submit_task";
const std::string CancelTaskSrvName = "cancel_task";
const std::string GetDispatchStatesSrvName = "get_dispatches";

const std::string DispatchCommandTopicName = Prefix + "dispatch_request";
const std::string DispatchAckTopicName = Prefix + "dispatch_ack";
const std::string DispatchStatesTopicName = "dispatch_states";
const std::string TaskStatusTopicName = "task_summaries";

} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__STANDARDNAMES_HPP
