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

#include <rmf_task_ros2/bidding/AsyncBidder.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <nlohmann/json.hpp>

#include <chrono>
#include <thread>
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
BidNoticeMsg bidding_task1;
BidNoticeMsg bidding_task2;

// set time window to 2s
auto timeout = rmf_traffic_ros2::convert(rmf_traffic::time::from_seconds(2.0));

//==============================================================================
SCENARIO("Auction with 2 Bids", "[TwoBids]")
{
  // Initializing bidding task
  nlohmann::json req_1;
  req_1["category"] = "patrol";
  req_1["description"] = "mocking a patrol";

  nlohmann::json req_2;
  req_2["category"] = "delivery";
  req_2["description"] = "mocking a delivery";

  bidding_task1.task_id = "bid1";
  bidding_task1.time_window = timeout;
  bidding_task1.request = req_1.dump();

  bidding_task2.task_id = "bid2";
  bidding_task2.time_window = timeout;
  bidding_task2.request = req_2.dump();

  //============================================================================
  // test received msg
  std::optional<std::string> test_notice_bidder1;
  std::optional<std::string> test_notice_bidder2;
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
      const auto& task_id,
      const auto winner,
      const auto&)
    {
      if (!winner)
        return;
      r_result_id = task_id;
      r_result_winner = winner->fleet_name;
      return;
    },
    nullptr
  );

  rclcpp::ExecutorOptions exec_options;
  exec_options.context = rcl_context;
  rclcpp::executors::SingleThreadedExecutor executor(exec_options);
  executor.add_node(node);

  auto bidder1 = AsyncBidder::make(
    node,
    [&test_notice_bidder1](const auto& notice, auto respond)
    {
      Response::Proposal best_robot_estimate;
      test_notice_bidder1 = notice.request;
      best_robot_estimate.fleet_name = "bidder1";
      best_robot_estimate.finish_time =
      std::chrono::steady_clock::time_point::max();

      respond(Response{best_robot_estimate, {}});
    }
  );

  auto bidder2 = AsyncBidder::make(
    node,
    [&test_notice_bidder2](const auto& notice, auto respond)
    {
      auto request = nlohmann::json::parse(notice.request);
      if (request["category"] == "patrol")
        return respond(Response{std::nullopt, {"Cannot patrol"}});

      // TaskType should not be supported
      Response::Proposal best_robot_estimate;
      best_robot_estimate.new_cost = 2.3; // lower cost than bidder1
      best_robot_estimate.fleet_name = "bidder2";
      best_robot_estimate.finish_time =
      std::chrono::steady_clock::time_point::min();
      test_notice_bidder2 = notice.request;

      respond(Response{best_robot_estimate, {}});
    }
  );

  // ROS Spin: forever incompleted future
  std::promise<void> ready_promise;
  std::shared_future<void> ready_future(ready_promise.get_future());

  WHEN("First 'patrol' Task Bid")
  {
    auctioneer->request_bid(bidding_task1);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(1.0));

    // Check if bidder 1 & 2 receive BidNotice1
    REQUIRE(test_notice_bidder1.has_value());
    CHECK(*test_notice_bidder1 == bidding_task1.request);
    REQUIRE(!test_notice_bidder2.has_value()); // bidder2 doesn't support patrol

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(2.5));

    // Check if Auctioneer received Bid from bidder1
    REQUIRE(r_result_winner == "bidder1");
    REQUIRE(r_result_id == "bid1");
  }

  WHEN("Second 'delivery' Task bid")
  {
    // start bidding
    auctioneer->request_bid(bidding_task2);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(1.0));

    // Check if bidder 1 & 2 receive BidNotice2
    REQUIRE(test_notice_bidder1.has_value());
    CHECK(*test_notice_bidder1 == bidding_task2.request);
    REQUIRE(test_notice_bidder2.has_value());
    REQUIRE(*test_notice_bidder2 == bidding_task2.request);

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
