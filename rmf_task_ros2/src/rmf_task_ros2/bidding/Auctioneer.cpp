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

#include "internal_Auctioneer.hpp"

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
Submission convert(const BidProposal& from)
{
  Submission submission;
  submission.fleet_name = from.fleet_name;
  submission.robot_name = from.robot_name;
  submission.prev_cost = from.prev_cost;
  submission.new_cost = from.new_cost;
  submission.finish_time = rmf_traffic_ros2::convert(from.finish_time);
  return submission;
}

//==============================================================================
Auctioneer::Implementation::Implementation(
  const std::shared_ptr<rclcpp::Node>& node_,
  BiddingResultCallback result_callback)
: node{std::move(node_)},
  bidding_result_callback{std::move(result_callback)}
{
  // default evaluator
  evaluator = std::make_shared<LeastFleetDiffCostEvaluator>();
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  bid_notice_pub = node->create_publisher<BidNotice>(
    rmf_task_ros2::BidNoticeTopicName, dispatch_qos);

  bid_proposal_sub = node->create_subscription<BidProposal>(
    rmf_task_ros2::BidProposalTopicName, dispatch_qos,
    [&](const BidProposal::UniquePtr msg)
    {
      this->receive_proposal(*msg);
    });

  timer = node->create_wall_timer(std::chrono::milliseconds(200), [&]()
      {
        this->check_bidding_process();
      });
}

//==============================================================================
void Auctioneer::Implementation::start_bidding(
  const BidNotice& bid_notice)
{
  RCLCPP_INFO(node->get_logger(), "Add Task [%s] to a bidding queue",
    bid_notice.task_profile.task_id.c_str());

  BiddingTask bidding_task;
  bidding_task.bid_notice = bid_notice;
  bidding_task.start_time = node->now();
  queue_bidding_tasks.push(bidding_task);
}

//==============================================================================
void Auctioneer::Implementation::receive_proposal(
  const BidProposal& msg)
{
  const auto id = msg.task_profile.task_id;
  RCLCPP_DEBUG(node->get_logger(),
    "[Auctioneer] Receive proposal from task_id: %s | from: %s",
    id.c_str(), msg.fleet_name.c_str());

  // check if bidding task is initiated by the auctioneer previously
  // add submited proposal to the current bidding tasks list
  if (queue_bidding_tasks.front().bid_notice.task_profile.task_id == id)
    queue_bidding_tasks.front().submissions.push_back(convert(msg));
}

//==============================================================================
// determine the winner within a bidding task instance
void Auctioneer::Implementation::check_bidding_process()
{
  if (queue_bidding_tasks.size() == 0)
    return;

  // Executing the task at the front queue
  auto front_task = queue_bidding_tasks.front();

  if (bidding_in_proccess)
  {
    if (determine_winner(front_task))
    {
      queue_bidding_tasks.pop();
      bidding_in_proccess = false;
    }
  }
  else
  {
    RCLCPP_DEBUG(node->get_logger(), " - Start new bidding task: %s",
      front_task.bid_notice.task_profile.task_id.c_str());
    queue_bidding_tasks.front().start_time = node->now();
    bid_notice_pub->publish(front_task.bid_notice);
    bidding_in_proccess = true;
  }
}

//==============================================================================
bool Auctioneer::Implementation::determine_winner(
  const BiddingTask& bidding_task)
{
  const auto duration = node->now() - bidding_task.start_time;

  if (duration > bidding_task.bid_notice.time_window)
  {
    auto id = bidding_task.bid_notice.task_profile.task_id;
    RCLCPP_DEBUG(node->get_logger(), "Bidding Deadline reached: %s",
      id.c_str());
    std::optional<Submission> winner = std::nullopt;

    if (bidding_task.submissions.size() == 0)
    {
      RCLCPP_DEBUG(node->get_logger(),
        "Bidding task has not received any bids");
    }
    else
    {
      winner = evaluate(bidding_task.submissions);
      RCLCPP_INFO(
        node->get_logger(),
        "Determined winning Fleet Adapter: [%s], from %ld submissions",
        winner->fleet_name.c_str(),
        bidding_task.submissions.size());
    }

    // Call the user defined callback function
    if (bidding_result_callback)
      bidding_result_callback(id, winner);

    return true;
  }
  return false;
}

//==============================================================================
std::optional<Submission> Auctioneer::Implementation::evaluate(
  const Submissions& submissions)
{
  if (submissions.size() == 0)
    return std::nullopt;

  if (!evaluator)
  {
    RCLCPP_WARN(node->get_logger(), "Bidding Evaluator is not set");
    return std::nullopt;
  }

  const std::size_t choice = evaluator->choose(submissions);

  if (choice >= submissions.size())
    return std::nullopt;

  return submissions[choice];
}

//==============================================================================
std::shared_ptr<Auctioneer> Auctioneer::make(
  const std::shared_ptr<rclcpp::Node>& node,
  BiddingResultCallback result_callback)
{
  auto pimpl = rmf_utils::make_unique_impl<Implementation>(
    node, result_callback);
  auto auctioneer = std::shared_ptr<Auctioneer>(new Auctioneer());
  auctioneer->_pimpl = std::move(pimpl);
  return auctioneer;
}

//==============================================================================
void Auctioneer::start_bidding(const BidNotice& bid_notice)
{
  _pimpl->start_bidding(bid_notice);
}

//==============================================================================
void Auctioneer::select_evaluator(
  std::shared_ptr<Auctioneer::Evaluator> evaluator)
{
  _pimpl->evaluator = std::move(evaluator);
}

//==============================================================================
Auctioneer::Auctioneer()
{
  // do nothing
}

//==============================================================================
std::size_t LeastFleetDiffCostEvaluator::choose(
  const Submissions& submissions) const
{
  auto winner_it = submissions.begin();
  float winner_cost_diff = winner_it->new_cost - winner_it->prev_cost;
  for (auto nominee_it = ++submissions.begin();
    nominee_it != submissions.end(); ++nominee_it)
  {
    float nominee_cost_diff = nominee_it->new_cost - nominee_it->prev_cost;
    if (nominee_cost_diff < winner_cost_diff)
    {
      winner_it = nominee_it;
      winner_cost_diff = nominee_cost_diff;
    }
  }
  return std::distance(submissions.begin(), winner_it);
}

//==============================================================================
std::size_t LeastFleetCostEvaluator::choose(
  const Submissions& submissions) const
{
  auto winner_it = submissions.begin();
  for (auto nominee_it = ++submissions.begin();
    nominee_it != submissions.end(); ++nominee_it)
  {
    if (nominee_it->new_cost < winner_it->new_cost)
      winner_it = nominee_it;
  }
  return std::distance(submissions.begin(), winner_it);
}

//==============================================================================
std::size_t QuickestFinishEvaluator::choose(
  const Submissions& submissions) const
{
  auto winner_it = submissions.begin();
  for (auto nominee_it = ++submissions.begin();
    nominee_it != submissions.end(); ++nominee_it)
  {
    if (nominee_it->finish_time < winner_it->finish_time)
      winner_it = nominee_it;
  }
  return std::distance(submissions.begin(), winner_it);
}

} // namespace bidding
} // namespace rmf_task_ros2
