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

#include <rmf_task_ros2/bidding/MinimalBidder.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <chrono>
#include <thread>
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {
namespace bidding {

using TaskProfile = rmf_task_msgs::msg::TaskProfile;
using TaskType = bidding::MinimalBidder::TaskType;

//==============================================================================
BidNotice bidding_task1;
BidNotice bidding_task2;

// set time window to 2s
auto timeout = rmf_traffic_ros2::convert(rmf_traffic::time::from_seconds(2.0));

//==============================================================================
SCENARIO("Auction with 2 Bids", "[TwoBids]")
{
  // Initializing bidding task
  bidding_task1.task_profile.task_id = "bid1";
  bidding_task1.time_window = timeout;
  bidding_task1.task_profile.description.task_type.type =
    rmf_task_msgs::msg::TaskType::TYPE_STATION;

  bidding_task2.task_profile.task_id = "bid2";
  bidding_task2.time_window = timeout;
  bidding_task2.task_profile.description.task_type.type =
    rmf_task_msgs::msg::TaskType::TYPE_DELIVERY;

  //============================================================================
  // test received msg
  std::optional<TaskProfile> test_notice_bidder1;
  std::optional<TaskProfile> test_notice_bidder2;
  std::string r_result_id = "";
  std::string r_result_winner = "";

  // Creating 1 auctioneer and 1 bidder
  const auto rcl_context = std::make_shared<rclcpp::Context>();
  rcl_context->init(0, nullptr);
  const auto node = rclcpp::Node::make_shared(
    "test_selfbidding", rclcpp::NodeOptions().context(rcl_context));

  auto auctioneer = Auctioneer::make(
    node,
    /// Bidding Result Callback Function
    [&r_result_id, &r_result_winner](
      const std::string& task_id, const std::optional<Submission> winner)
    {
      if (!winner)
        return;
      r_result_id = task_id;
      r_result_winner = winner->fleet_name;
      return;
    }
  );

  rclcpp::ExecutorOptions exec_options;
  exec_options.context = rcl_context;
  rclcpp::executors::SingleThreadedExecutor executor(exec_options);
  executor.add_node(node);

  auto bidder1 = MinimalBidder::make(
    node, "bidder1", { TaskType::Station, TaskType::Delivery },
    [&test_notice_bidder1](const BidNotice& notice)
    {
      Submission best_robot_estimate;
      test_notice_bidder1 = notice.task_profile;
      return best_robot_estimate;
    }
  );

  auto bidder2 = MinimalBidder::make(
    node, "bidder2", { TaskType::Delivery, TaskType::Clean },
    [&test_notice_bidder2](const BidNotice& notice)
    {
      // TaskType should not be supported
      Submission best_robot_estimate;
      best_robot_estimate.new_cost = 2.3; // lower cost than bidder1
      test_notice_bidder2 = notice.task_profile;
      return best_robot_estimate;
    }
  );

  // ROS Spin: forever incompleted future
  std::promise<void> ready_promise;
  std::shared_future<void> ready_future(ready_promise.get_future());

  WHEN("First 'Station' Task Bid")
  {
    // start bidding
    bidding_task1.task_profile.submission_time = node->now();
    auctioneer->start_bidding(bidding_task1);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(1.0));

    // Check if bidder 1 & 2 receive BidNotice1
    REQUIRE(test_notice_bidder1);
    REQUIRE(test_notice_bidder1->task_id == bidding_task1.task_profile.task_id);
    REQUIRE(!test_notice_bidder2); // bidder2 doesnt support tasktype

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(2.5));

    // Check if Auctioneer received Bid from bidder1
    REQUIRE(r_result_winner == "bidder1");
    REQUIRE(r_result_id == "bid1");
  }

  WHEN("Second 'Delivery' Task bid")
  {
    // start bidding
    bidding_task2.task_profile.submission_time = node->now();
    auctioneer->start_bidding(bidding_task2);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(1.0));

    // Check if bidder 1 & 2 receive BidNotice2
    auto task2_profile = bidding_task2.task_profile;
    REQUIRE(test_notice_bidder1->task_id == task2_profile.task_id);
    REQUIRE(test_notice_bidder2->task_id == task2_profile.task_id);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(2.5));

    // Check if Auctioneer received Bid from bidder1
    REQUIRE(r_result_winner == "bidder2");
    REQUIRE(r_result_id == "bid2");
  }

  rclcpp::shutdown(rcl_context);
}

} // namespace bidding
} // namespace rmf_task_ros2
