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

#include <rmf_task_ros2/bidding/Response.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rmf_task_msgs/msg/bid_proposal.hpp>

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_task_ros2/StandardNames.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
class Auctioneer::Implementation
{
public:

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
    node_logging_interface;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_interface;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
  node_parameters_interface;

  rclcpp::TimerBase::SharedPtr timer;
  BiddingResultCallback bidding_result_callback;
  ConstEvaluatorPtr evaluator;

  struct OpenBid
  {
    BidNoticeMsg bid_notice;
    builtin_interfaces::msg::Time start_time;
    std::vector<bidding::Response> responses;
  };

  bool bidding_in_process = false;
  std::queue<OpenBid> open_bid_queue;

  using BidNoticePub = rclcpp::Publisher<BidNoticeMsg>;
  BidNoticePub::SharedPtr bid_notice_pub;

  using BidResponseSub = rclcpp::Subscription<BidResponseMsg>;
  BidResponseSub::SharedPtr bid_proposal_sub;

  Implementation(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    node_base_interface,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr
    node_clock_interface,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
    node_logging_interface,
    const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
    node_timers_interface,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
    node_topics_interface,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
    node_parameters_interface_,
    BiddingResultCallback result_callback,
    ConstEvaluatorPtr evaluator);

  /// Start a bidding process
  void request_bid(const BidNoticeMsg& bid_notice);

  // Receive proposal and evaluate
  void receive_response(const BidResponseMsg& msg);

  // determine the winner within a bidding task instance
  void finish_bidding_process();

  bool determine_winner(const OpenBid& bidding_task);

  std::optional<Response::Proposal> evaluate(const Responses& responses);

  static const Implementation& get(const Auctioneer& auctioneer)
  {
    return *auctioneer._pimpl;
  }
};

//==============================================================================
std::optional<Response::Proposal> evaluate(
  const Auctioneer& auctioneer,
  const Responses& responses)
{
  auto fimpl = Auctioneer::Implementation::get(auctioneer);
  return fimpl.evaluate(responses);
}

} // namespace bidding
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__BIDDING__INTERNAL_AUCTIONEER_HPP
