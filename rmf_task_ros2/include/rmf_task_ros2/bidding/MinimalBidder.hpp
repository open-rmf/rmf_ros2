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

#ifndef RMF_TASK_ROS2__BIDDING__MINIMALBIDDER_HPP
#define RMF_TASK_ROS2__BIDDING__MINIMALBIDDER_HPP

#include <unordered_set>

#include <rclcpp/node.hpp>
#include <rmf_utils/impl_ptr.hpp>

#include <rmf_task_ros2/bidding/Submission.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
class MinimalBidder
{
public:
  /// Type of Task in RMF
  using TaskTypeMsg = rmf_task_msgs::msg::TaskType;
  enum class TaskType
  {
    Station       = TaskTypeMsg::TYPE_STATION,
    Loop          = TaskTypeMsg::TYPE_LOOP,
    Delivery      = TaskTypeMsg::TYPE_DELIVERY,
    ChargeBattery = TaskTypeMsg::TYPE_CHARGE_BATTERY,
    Clean         = TaskTypeMsg::TYPE_CLEAN,
    Patrol        = TaskTypeMsg::TYPE_PATROL
  };

  /// Callback function when a bid notice is received from the autioneer
  ///
  /// \param[in] notice
  ///   bid notice msg
  ///
  /// \return submission
  ///   Estimates of a task. This submission is used by dispatcher for eval
  using ParseSubmissionCallback =
    std::function<Submission(const BidNotice& notice)>;


  /// Create a bidder to bid for incoming task requests from Task Dispatcher
  ///
  /// \param[in] node
  ///   ROS 2 node instance
  ///
  /// \param[in] fleet_name
  ///   Name of the bidder
  ///
  /// \param[in] valid_task_types
  ///   A list of valid tasks types which are supported by the bidder
  ///
  /// \param[in] submission_cb
  ///   fn which is used to provide a bid submission during a call for bid
  static std::shared_ptr<MinimalBidder> make(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& fleet_name,
    const std::unordered_set<TaskType>& valid_task_types,
    ParseSubmissionCallback submission_cb);

  class Implementation;

private:
  MinimalBidder();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__BIDDING__MINIMALBIDDER_HPP
