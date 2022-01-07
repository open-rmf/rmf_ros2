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

#include <rmf_task_ros2/bidding/Response.hpp>
#include <rmf_task_ros2/bidding/AsyncBidder.hpp>

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_task_msgs/msg/bid_proposal.hpp>
#include <rmf_task_ros2/StandardNames.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
class AsyncBidder::Implementation
{
public:

  std::weak_ptr<rclcpp::Node> w_node;
  ReceiveNotice receive_notice;

  using BidNoticeSub = rclcpp::Subscription<BidNoticeMsg>;
  BidNoticeSub::SharedPtr bid_notice_sub;

  using BidResponsePub = rclcpp::Publisher<BidResponseMsg>;
  BidResponsePub::SharedPtr bid_response_pub;

  Implementation(
    std::shared_ptr<rclcpp::Node> node_,
    ReceiveNotice receive_notice)
  : w_node{std::move(node_)},
    receive_notice{std::move(receive_notice)}
  {
    const auto bid_qos = rclcpp::ServicesQoS().reliable();
    const auto node = w_node.lock();

    bid_notice_sub = node->create_subscription<BidNoticeMsg>(
      rmf_task_ros2::BidNoticeTopicName, bid_qos,
      [&](const BidNoticeMsg::UniquePtr msg)
      {
        this->handle_notice(*msg);
      });

    bid_response_pub = node->create_publisher<BidResponseMsg>(
      rmf_task_ros2::BidResponseTopicName, bid_qos);
  }

  // Callback fn when a dispatch notice is received
  void handle_notice(const BidNoticeMsg& msg)
  {
    const auto node = w_node.lock();
    if (!node)
      return;

    RCLCPP_INFO(node->get_logger(),
      "[Bidder] Received Bidding notice for task_id [%s]",
      msg.task_id.c_str());

    // check if the user did not supply a receive notice callback
    if (!receive_notice)
      return;

    // Send the notice
    receive_notice(
      msg,
      [task_id = msg.task_id, pub = bid_response_pub](
        const Response& response)
      {
        pub->publish(convert(response, task_id));
      });
  }
};

//==============================================================================
std::shared_ptr<AsyncBidder> AsyncBidder::make(
  const std::shared_ptr<rclcpp::Node>& node,
  ReceiveNotice receive_notice)
{
  auto bidder = std::shared_ptr<AsyncBidder>(new AsyncBidder());
  bidder->_pimpl =
    rmf_utils::make_unique_impl<Implementation>(node, receive_notice);

  return bidder;
}

//==============================================================================
AsyncBidder::AsyncBidder()
{
  // do nothing
}

} // namespace bidding
} // namespace rmf_task_ros2
