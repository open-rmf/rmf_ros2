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

/// Note: This is a testing bidder node script

#include <rmf_task_ros2/bidding/AsyncBidder.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <nlohmann/json.hpp>

using namespace rmf_task_ros2;

int main(int argc, char* argv[])
{
  std::string fleet_name = "dummy_fleet";

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(fleet_name);

  RCLCPP_INFO(
    node->get_logger(),
    "Beginning example task bidder node");

  //============================================================================
  // Create Bidder instance
  std::shared_ptr<bidding::AsyncBidder> bidder = bidding::AsyncBidder::make(
    node,
    [node](
      const bidding::BidNoticeMsg& notice,
      bidding::AsyncBidder::Respond resp)
    {
      // Here user will provide the best robot as a bid submission
      std::cout << "[MockBidder] Providing best estimates" << std::endl;
      const auto req = nlohmann::json::parse(notice.request);
      const auto& start_time_json = req["unix_millis_earliest_start_time"];
      const auto req_start_time = start_time_json ?
      rmf_traffic::Time(
        std::chrono::milliseconds(start_time_json.get<int64_t>())) :
      rmf_traffic_ros2::convert(node->now());

      resp(bidding::Response{
        bidding::Response::Proposal{
          "mockfleet",
          "mockbot",
          10.2,
          13.5,
          rmf_traffic::time::apply_offset(req_start_time, 7)
        },
        {}
      });
    }
  );

  rclcpp::spin(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Closing down task bidder node");

  rclcpp::shutdown();
}
