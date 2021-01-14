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

#include <rmf_task_ros2/bidding/MinimalBidder.hpp>
#include "../rmf_task_ros2/action/Server.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rmf_traffic_ros2/Time.hpp>

using namespace rmf_task_ros2;
using TaskType = bidding::MinimalBidder::TaskType;

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
  std::shared_ptr<bidding::MinimalBidder> bidder = bidding::MinimalBidder::make(
    node,
    fleet_name,
    { TaskType::Clean, TaskType::Delivery },
    [](const bidding::BidNotice& notice)
    {
      // Here user will provice the best robot as a bid submission
      std::cout << "[MockBidder] Providing best estimates" << std::endl;
      auto req_start_time =
      rmf_traffic_ros2::convert(notice.task_profile.description.start_time);

      bidding::Submission best_robot_estimate;
      best_robot_estimate.robot_name = "dumbot";
      best_robot_estimate.prev_cost = 10.2;
      best_robot_estimate.new_cost = 13.5;
      best_robot_estimate.finish_time =
      rmf_traffic::time::apply_offset(req_start_time, 7);
      return best_robot_estimate;
    }
  );

  //============================================================================
  // Create sample RMF task action server
  auto action_server = action::Server::make(node, fleet_name);

  action_server->register_callbacks(
    [&action_server, &node](const TaskProfile& task_profile)
    {
      std::cout << "[MockBidder] ~Start Queue Task: "
                << task_profile.task_id<<std::endl;

      // async on executing task
      // auto _ = std::async(std::launch::async,
      auto t = std::thread(
        [&action_server, &node](auto profile)
        {
          TaskStatus status;
          status.task_profile = profile;
          status.robot_name = "dumbot";
          status.start_time = rmf_traffic_ros2::convert(node->now());
          status.end_time =
          rmf_traffic::time::apply_offset(status.start_time, 7);

          const auto id = profile.task_id;
          std::cout << " [MockBidder] Queued, TaskID: "  << id << std::endl;
          action_server->update_status(status);

          std::this_thread::sleep_for(std::chrono::seconds(2));
          std::cout << " [MockBidder] Executing, TaskID: " << id << std::endl;
          status.state = TaskStatus::State::Executing;
          action_server->update_status(status);

          std::this_thread::sleep_for(std::chrono::seconds(5));
          std::cout << " [MockBidder] Completed, TaskID: " << id << std::endl;
          status.state = TaskStatus::State::Completed;
          action_server->update_status(status);
        }, task_profile
      );
      t.detach();

      return true; //successs (send State::Queued)
    },
    [&action_server](const TaskProfile& task_profile)
    {
      std::cout << "[MockBidder] ~Cancel Executing Task: "
                << task_profile.task_id<<std::endl;
      return true; //success , send State::Canceled
    }
  );

  rclcpp::spin(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Closing down task bidder node");

  rclcpp::shutdown();
}
