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


#ifndef SRC__RMF_TASK_ROS2__BIDDING__INTERNAL_AUCTIONEER_HPP
#define SRC__RMF_TASK_ROS2__BIDDING__INTERNAL_AUCTIONEER_HPP

#include <rmf_task_ros2/bidding/Submission.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rmf_task_msgs/msg/bid_proposal.hpp>

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_task_ros2/StandardNames.hpp>

namespace rmf_task_ros2 {
namespace bidding {

using BidProposal = rmf_task_msgs::msg::BidProposal;

//==============================================================================
class Auctioneer::Implementation
{
public:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::TimerBase::SharedPtr timer;
  BiddingResultCallback bidding_result_callback;
  std::shared_ptr<Evaluator> evaluator;

  struct BiddingTask
  {
    BidNotice bid_notice;
    builtin_interfaces::msg::Time start_time;
    std::vector<bidding::Submission> submissions;
  };

  bool bidding_in_proccess = false;
  std::queue<BiddingTask> queue_bidding_tasks;

  using BidNoticePub = rclcpp::Publisher<BidNotice>;
  BidNoticePub::SharedPtr bid_notice_pub;

  using BidProposalSub = rclcpp::Subscription<BidProposal>;
  BidProposalSub::SharedPtr bid_proposal_sub;

  Implementation(
    const std::shared_ptr<rclcpp::Node>& node_,
    BiddingResultCallback result_callback);

  /// Start a bidding process
  void start_bidding(const BidNotice& bid_notice);

  // Receive proposal and evaluate
  void receive_proposal(const BidProposal& msg);

  // determine the winner within a bidding task instance
  void check_bidding_process();

  bool determine_winner(const BiddingTask& bidding_task);

  std::optional<Submission> evaluate(const Submissions& submissions);

  static const Implementation& get(const Auctioneer& auctioneer)
  {
    return *auctioneer._pimpl;
  }
};

//==============================================================================
std::optional<Submission> evaluate(
  const Auctioneer& auctioneer,
  const Submissions& submissions)
{
  auto fimpl = Auctioneer::Implementation::get(auctioneer);
  return fimpl.evaluate(submissions);
}

} // namespace bidding
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__BIDDING__INTERNAL_AUCTIONEER_HPP
