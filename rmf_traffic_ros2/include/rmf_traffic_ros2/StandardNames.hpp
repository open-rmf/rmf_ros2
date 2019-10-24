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

#ifndef RMF_TRAFFIC_ROS2__STANDARDNAMES_HPP
#define RMF_TRAFFIC_ROS2__STANDARDNAMES_HPP

#include <string>

namespace rmf_traffic_ros2 {

const std::string Prefix = "rmf_traffic/";
const std::string SubmitTrajectoryServiceName = Prefix + "submit_trajectory";
const std::string EraseScheduleServiceName = Prefix + "erase_schedule";
const std::string RegisterQueryServiceName = Prefix + "register_query";
const std::string UnregisterQueryServiceName = Prefix + "unregister_query";
const std::string MirrorUpdateServiceName = Prefix + "mirror_update";
const std::string MirrorWakeupTopicName = Prefix + "mirror_wakeup";

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__STANDARDNAMES_HPP
