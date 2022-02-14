/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <rmf_task_ros2/bidding/Response.hpp>

#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
BidProposalMsg convert(const Response::Proposal& proposal)
{
  return rmf_task_msgs::build<BidProposalMsg>()
    .fleet_name(proposal.fleet_name)
    .expected_robot_name(proposal.expected_robot_name)
    .prev_cost(proposal.prev_cost)
    .new_cost(proposal.new_cost)
    .finish_time(rmf_traffic_ros2::convert(proposal.finish_time));
}

//==============================================================================
BidResponseMsg convert(
  const Response& response,
  const std::string& task_id)
{
  if (response.proposal.has_value())
  {
    return rmf_task_msgs::build<BidResponseMsg>()
      .task_id(task_id)
      .has_proposal(true)
      .proposal(convert(*response.proposal))
      .errors(response.errors);
  }

  return rmf_task_msgs::build<BidResponseMsg>()
    .task_id(task_id)
    .has_proposal(false)
    .proposal(rmf_task_msgs::msg::BidProposal())
    .errors(response.errors);
}

//==============================================================================
Response convert(const BidResponseMsg& msg)
{
  if (msg.has_proposal)
  {
    return Response{
      Response::Proposal{
        msg.proposal.fleet_name,
        msg.proposal.expected_robot_name,
        msg.proposal.prev_cost,
        msg.proposal.new_cost,
        rmf_traffic_ros2::convert(msg.proposal.finish_time)
      },
      msg.errors
    };
  }

  return Response{
    std::nullopt,
    msg.errors
  };
}

} // namespace bidding
} // namespace rmf_task_ros2
