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

#ifndef RMF_TASK_ROS2__BIDDING__SUBMISSION_HPP
#define RMF_TASK_ROS2__BIDDING__SUBMISSION_HPP

#include <rmf_traffic/Time.hpp>
#include <rmf_task_msgs/msg/bid_notice.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
struct Submission
{
  std::string fleet_name;
  std::string robot_name;
  double prev_cost = 0.0;
  double new_cost = std::numeric_limits<double>::max();
  rmf_traffic::Time finish_time;
};

//==============================================================================
using Submissions = std::vector<Submission>;
using BidNotice = rmf_task_msgs::msg::BidNotice;

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__BIDDING__SUBMISSION_HPP
