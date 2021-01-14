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

#include <rmf_task_ros2/bidding/Submission.hpp>
#include <rmf_task_ros2/bidding/MinimalBidder.hpp>

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_task_msgs/msg/bid_proposal.hpp>
#include <rmf_task_ros2/StandardNames.hpp>

namespace rmf_task_ros2 {
namespace bidding {

using BidProposal = rmf_task_msgs::msg::BidProposal;

//==============================================================================
BidProposal convert(const Submission& from)
{
  BidProposal proposal_msg;
  proposal_msg.fleet_name = from.fleet_name;
  proposal_msg.robot_name = from.robot_name;
  proposal_msg.prev_cost = from.prev_cost;
  proposal_msg.new_cost = from.new_cost;
  proposal_msg.finish_time = rmf_traffic_ros2::convert(from.finish_time);
  return proposal_msg;
}

//==============================================================================
class MinimalBidder::Implementation
{
public:

  std::shared_ptr<rclcpp::Node> node;
  std::string fleet_name;
  std::unordered_set<TaskType> valid_task_types;
  ParseSubmissionCallback get_submission_fn;

  using BidNoticeSub = rclcpp::Subscription<BidNotice>;
  BidNoticeSub::SharedPtr dispatch_notice_sub;

  using BidProposalPub = rclcpp::Publisher<BidProposal>;
  BidProposalPub::SharedPtr dispatch_proposal_pub;

  Implementation(
    std::shared_ptr<rclcpp::Node> node_,
    const std::string& fleet_name_,
    const std::unordered_set<TaskType>& valid_task_types_,
    ParseSubmissionCallback submission_cb)
  : node{std::move(node_)},
    fleet_name{std::move(fleet_name_)},
    valid_task_types{std::move(valid_task_types_)},
    get_submission_fn{std::move(submission_cb)}
  {
    const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

    dispatch_notice_sub = node->create_subscription<BidNotice>(
      rmf_task_ros2::BidNoticeTopicName, dispatch_qos,
      [&](const BidNotice::UniquePtr msg)
      {
        this->receive_notice(*msg);
      });

    dispatch_proposal_pub = node->create_publisher<BidProposal>(
      rmf_task_ros2::BidProposalTopicName, dispatch_qos);
  }

  // Callback fn when a dispatch notice is received
  void receive_notice(const BidNotice& msg)
  {
    RCLCPP_INFO(node->get_logger(),
      "[Bidder] Received Bidding notice for task_id [%s]",
      msg.task_profile.task_id.c_str());

    const auto task_type = (msg.task_profile.description.task_type.type);

    // check if task type is valid
    if (!valid_task_types.count(static_cast<TaskType>(task_type)))
    {
      RCLCPP_WARN(node->get_logger(), "[%s]: task type %d is not supported",
        fleet_name.c_str(), task_type);
      return;
    }

    // check if get submission function is declared
    if (!get_submission_fn)
      return;

    // Submit proposal
    const auto bid_submission = get_submission_fn(msg);
    auto best_proposal = convert(bid_submission);
    best_proposal.fleet_name = fleet_name;
    best_proposal.task_profile = msg.task_profile;
    dispatch_proposal_pub->publish(best_proposal);
  }
};

//==============================================================================
std::shared_ptr<MinimalBidder> MinimalBidder::make(
  const std::shared_ptr<rclcpp::Node>& node,
  const std::string& fleet_name,
  const std::unordered_set<TaskType>& valid_task_types,
  ParseSubmissionCallback submission_cb)
{
  auto pimpl = rmf_utils::make_unique_impl<Implementation>(
    node, fleet_name, valid_task_types, submission_cb);
  auto bidder = std::shared_ptr<MinimalBidder>(new MinimalBidder());
  bidder->_pimpl = std::move(pimpl);
  return bidder;
}

//==============================================================================
MinimalBidder::MinimalBidder()
{
  // do nothing
}

} // namespace bidding
} // namespace rmf_task_ros2
