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

#ifndef RMF_TASK_ROS2__BIDDING__RESPONSE_HPP
#define RMF_TASK_ROS2__BIDDING__RESPONSE_HPP

#include <optional>

#include <rmf_traffic/Time.hpp>
#include <rmf_task_msgs/msg/bid_notice.hpp>
#include <rmf_task_msgs/msg/bid_proposal.hpp>
#include <rmf_task_msgs/msg/bid_response.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
struct Response
{
  struct Proposal
  {
    std::string fleet_name;
    std::string expected_robot_name;
    double prev_cost;
    double new_cost;
    rmf_traffic::Time finish_time;
  };

  std::optional<Proposal> proposal;
  std::vector<std::string> errors;
};

//==============================================================================
using Responses = std::vector<Response>;

//==============================================================================
using BidResponseMsg = rmf_task_msgs::msg::BidResponse;
using BidProposalMsg = rmf_task_msgs::msg::BidProposal;
using BidNoticeMsg = rmf_task_msgs::msg::BidNotice;

//==============================================================================
BidProposalMsg convert(const Response::Proposal& proposal);

//==============================================================================
BidResponseMsg convert(
  const Response& response,
  const std::string& task_id);

//==============================================================================
Response convert(const BidResponseMsg& msg);

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__BIDDING__RESPONSE_HPP
