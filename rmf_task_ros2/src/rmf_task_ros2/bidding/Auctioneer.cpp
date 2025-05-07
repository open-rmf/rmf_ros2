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

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/create_timer.hpp>

#include "internal_Auctioneer.hpp"

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
Auctioneer::Implementation::Implementation(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  node_base_interface_,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr
  node_clock_interface_,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
  node_logging_interface_,
  const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
  node_timers_interface_,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
  node_topics_interface_,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
  node_parameters_interface_,
  BiddingResultCallback result_callback,
  ConstEvaluatorPtr evaluator_)
: node_base_interface{std::move(node_base_interface_)},
  node_clock_interface{std::move(node_clock_interface_)},
  node_logging_interface{std::move(node_logging_interface_)},
  node_timers_interface{std::move(node_timers_interface_)},
  node_topics_interface{std::move(node_topics_interface_)},
  node_parameters_interface{std::move(node_parameters_interface_)},
  bidding_result_callback{std::move(result_callback)},
  evaluator(std::move(evaluator_))
{
  // default evaluator
  if (!evaluator)
  {
    evaluator = std::make_shared<QuickestFinishEvaluator>();
    RCLCPP_INFO(
      node_logging_interface->get_logger(),
      "Dispatcher evaluator set to QuickestFinishEvaluator by default");
  }
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  bid_notice_pub = rclcpp::create_publisher<BidNoticeMsg>(
    node_parameters_interface,
    node_topics_interface,
    rmf_task_ros2::BidNoticeTopicName,
    dispatch_qos);

  bid_proposal_sub = rclcpp::create_subscription<BidResponseMsg>(
    node_parameters_interface,
    node_topics_interface,
    rmf_task_ros2::BidResponseTopicName,
    dispatch_qos,
    [&](const BidResponseMsg::UniquePtr msg)
    {
      this->receive_response(*msg);
    });

  timer = rclcpp::create_timer(
    node_base_interface,
    node_timers_interface,
    node_clock_interface->get_clock(),
    std::chrono::milliseconds(200),
    [&]()
      {
        this->finish_bidding_process();
      });
}

//==============================================================================
void Auctioneer::Implementation::request_bid(
  const BidNoticeMsg& bid_notice)
{
  RCLCPP_INFO(
    node_logging_interface->get_logger(),
    "Add Task [%s] to a bidding queue",
    bid_notice.task_id.c_str());

  open_bid_queue.push(
    OpenBid{bid_notice, node_clock_interface->get_clock()->now(), {}});
}

//==============================================================================
void Auctioneer::Implementation::receive_response(const BidResponseMsg& msg)
{
  const auto id = msg.task_id;

  const auto response = convert(msg);
  if (response.proposal.has_value())
  {
    RCLCPP_DEBUG(node_logging_interface->get_logger(),
      "[Auctioneer] Receive proposal from task_id: %s | from: %s",
      id.c_str(), response.proposal->fleet_name.c_str());
  }
  else if (!response.errors.empty())
  {
    RCLCPP_DEBUG(
      node_logging_interface->get_logger(),
      "[Auctioneer] Received %lu errors from a bidder",
      response.errors.size());
  }

  // check if bidding task is initiated by the auctioneer previously
  // add submited proposal to the current bidding tasks list
  if (open_bid_queue.front().bid_notice.task_id == id)
    open_bid_queue.front().responses.push_back(response);
}

//==============================================================================
// determine the winner within a bidding task instance
void Auctioneer::Implementation::finish_bidding_process()
{
  if (open_bid_queue.size() == 0)
    return;

  // Executing the task at the front queue
  auto front_task = open_bid_queue.front();

  if (bidding_in_process)
  {
    if (determine_winner(front_task))
    {
      open_bid_queue.pop();
    }
  }
  else
  {
    RCLCPP_INFO(
      node_logging_interface->get_logger(),
      " - Start new bidding task: %s",
      front_task.bid_notice.task_id.c_str());
    open_bid_queue.front().start_time =
      node_clock_interface->get_clock()->now();
    bid_notice_pub->publish(front_task.bid_notice);
    bidding_in_process = true;
  }
}

//==============================================================================
bool Auctioneer::Implementation::determine_winner(
  const OpenBid& bidding_task)
{
  const auto duration =
    node_clock_interface->get_clock()->now() - bidding_task.start_time;
  if (duration < bidding_task.bid_notice.time_window)
    return false;

  if (!bidding_result_callback)
    return true;

  auto task_id = bidding_task.bid_notice.task_id;
  RCLCPP_DEBUG(
    node_logging_interface->get_logger(),
    "Bidding Deadline reached for [%s]",
    task_id.c_str());

  std::vector<std::string> errors;
  for (const auto& r : bidding_task.responses)
  {
    errors.insert(errors.end(), r.errors.begin(), r.errors.end());
  }

  if (bidding_task.responses.empty())
  {
    RCLCPP_INFO(node_logging_interface->get_logger(),
      "Task auction for [%s] did not received any bids", task_id.c_str());

    bidding_result_callback(task_id, std::nullopt, errors);
    return true;
  }

  auto winner = evaluate(bidding_task.responses);
  if (winner.has_value())
  {
    RCLCPP_INFO(
      node_logging_interface->get_logger(),
      "Determined winning Fleet Adapter: [%s], from %ld responses",
      winner->fleet_name.c_str(),
      bidding_task.responses.size());
  }

  // Call the user defined callback function
  bidding_result_callback(task_id, winner, errors);

  return true;
}

//==============================================================================
std::optional<Response::Proposal> Auctioneer::Implementation::evaluate(
  const Responses& responses)
{
  if (responses.size() == 0)
    return std::nullopt;

  if (!evaluator)
  {
    RCLCPP_WARN(
      node_logging_interface->get_logger(), "Bidding Evaluator is not set");
    return std::nullopt;
  }

  const auto choice = evaluator->choose(responses);
  if (!choice.has_value())
    return std::nullopt;

  if (*choice >= responses.size())
    return std::nullopt;

  return responses[*choice].proposal;
}

//==============================================================================
std::shared_ptr<Auctioneer> Auctioneer::make(
  const std::shared_ptr<rclcpp::Node>& node,
  BiddingResultCallback result_callback,
  ConstEvaluatorPtr evaluator)
{
  auto auctioneer = make(
    ActioneerInterfaces(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_timers_interface(),
      node->get_node_topics_interface(),
      node->get_node_parameters_interface()),
    std::move(result_callback),
    std::move(evaluator));
  return auctioneer;
}

//==============================================================================
std::shared_ptr<Auctioneer> Auctioneer::make(
  ActioneerInterfaces node_interfaces,
  BiddingResultCallback result_callback,
  ConstEvaluatorPtr evaluator)
{
  auto auctioneer = std::shared_ptr<Auctioneer>(new Auctioneer());
  auctioneer->_pimpl = rmf_utils::make_unique_impl<Implementation>(
    node_interfaces.get<rclcpp::node_interfaces::NodeBaseInterface>(),
    node_interfaces.get<rclcpp::node_interfaces::NodeClockInterface>(),
    node_interfaces.get<rclcpp::node_interfaces::NodeLoggingInterface>(),
    node_interfaces.get<rclcpp::node_interfaces::NodeTimersInterface>(),
    node_interfaces.get<rclcpp::node_interfaces::NodeTopicsInterface>(),
    node_interfaces.get<rclcpp::node_interfaces::NodeParametersInterface>(),
    std::move(result_callback),
    std::move(evaluator));
  return auctioneer;
}

//==============================================================================
void Auctioneer::request_bid(const BidNoticeMsg& bid_notice)
{
  _pimpl->request_bid(bid_notice);
}

//==============================================================================
void Auctioneer::ready_for_next_bid()
{
  _pimpl->bidding_in_process = false;
}

//==============================================================================
void Auctioneer::set_evaluator(ConstEvaluatorPtr evaluator)
{
  _pimpl->evaluator = std::move(evaluator);
}

//==============================================================================
Auctioneer::Auctioneer()
{
  // do nothing
}

namespace {
//==============================================================================
std::optional<std::size_t> select_best(
  const Responses& responses,
  const std::function<double(const Response::Proposal&)> eval)
{
  std::optional<std::size_t> best_index;
  std::optional<double> best_cost;
  for (std::size_t i = 0; i < responses.size(); ++i)
  {
    if (responses[i].proposal.has_value())
    {
      const auto cost = eval(*responses[i].proposal);
      if (!best_cost.has_value() || cost < *best_cost)
      {
        best_index = i;
        best_cost = cost;
      }
    }
  }

  return best_index;
}
} // anonymous namespace

//==============================================================================
std::optional<std::size_t> LeastFleetDiffCostEvaluator::choose(
  const Responses& responses) const
{
  return select_best(
    responses,
    [](const auto& nominee) { return nominee.new_cost - nominee.prev_cost; });
}

//==============================================================================
std::optional<std::size_t> LeastFleetCostEvaluator::choose(
  const Responses& responses) const
{
  return select_best(
    responses,
    [](const auto& nominee) { return nominee.new_cost; });
}

//==============================================================================
std::optional<std::size_t> QuickestFinishEvaluator::choose(
  const Responses& responses) const
{
  return select_best(
    responses,
    [](const auto& nominee)
    {
      return rmf_traffic::time::to_seconds(
        nominee.finish_time.time_since_epoch());
    });
}

} // namespace bidding
} // namespace rmf_task_ros2
