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

#ifndef RMF_TASK_ROS2__BIDDING__AUCTIONEER_HPP
#define RMF_TASK_ROS2__BIDDING__AUCTIONEER_HPP

#include <queue>
#include <rclcpp/node.hpp>
#include <rmf_utils/impl_ptr.hpp>

#include <rmf_task_ros2/bidding/Response.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
/// The Auctioneer class is responsible for initiating and mediating the bidding
/// process during task dispatching. Hence, this class instance is solely used
/// within the Dispatcher class
class Auctioneer : public std::enable_shared_from_this<Auctioneer>
{
public:

  /// Callback which will provide the winner when a bid is concluded
  ///
  /// \param[in] task_id
  ///   bidding task id
  ///
  /// \param[in] winner
  ///   single winner from all submissions. nullopt if non
  using BiddingResultCallback =
    std::function<
    void(
      const std::string& task_id,
      const std::optional<Response::Proposal> winner,
      const std::vector<std::string>& errors)>;

  /// A pure abstract interface class for the auctioneer to choose the best
  /// choosing the best submissions.
  class Evaluator
  {
  public:

    /// Given a list of submissions, choose the one that is the "best". It is
    /// up to the implementation of the Evaluator to decide how to rank.
    virtual std::optional<std::size_t> choose(
      const Responses& responses) const = 0;

    virtual ~Evaluator() = default;
  };

  using ConstEvaluatorPtr = std::shared_ptr<const Evaluator>;

  /// Create an instance of the Auctioneer. This instance will handle all
  /// the task dispatching bidding mechanism. A default evaluator is used.
  ///
  /// \param[in] node
  ///   ros2 node which will manage the bidding
  ///
  /// \param[in] result_callback
  ///   This callback fn will be called when a bidding result is concluded
  ///
  /// \sa make()
  static std::shared_ptr<Auctioneer> make(
    const std::shared_ptr<rclcpp::Node>& node,
    BiddingResultCallback result_callback,
    ConstEvaluatorPtr evaluator);

  /// Start a bidding process by provide a bidding task. Note the each
  /// bidding process is conducted sequentially
  ///
  /// \param[in] bid_notice
  ///   bidding task, task which will call for bid
  void request_bid(const BidNoticeMsg& bid_notice);

  /// Call this to tell the auctioneer that it may begin to perform the next bid
  void ready_for_next_bid();

  /// Provide a custom evaluator which will be used to choose the best bid
  /// If no selection is given, Default is: LeastFleetDiffCostEvaluator
  ///
  /// \param[in] evaluator
  void set_evaluator(ConstEvaluatorPtr evaluator);

  class Implementation;

private:
  Auctioneer();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class LeastFleetDiffCostEvaluator : public Auctioneer::Evaluator
{
public:
  std::optional<std::size_t> choose(const Responses& responses) const final;
};

//==============================================================================
class LeastFleetCostEvaluator : public Auctioneer::Evaluator
{
public:
  std::optional<std::size_t> choose(const Responses& submissions) const final;
};

//==============================================================================
class QuickestFinishEvaluator : public Auctioneer::Evaluator
{
public:
  std::optional<std::size_t> choose(const Responses& submissions) const final;
};

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__BIDDING__AUCTIONEER_HPP
