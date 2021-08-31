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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP

#include <rmf_task_msgs/msg/loop.hpp>

#include <rmf_task_msgs/msg/bid_proposal.hpp>
#include <rmf_task_msgs/msg/bid_notice.hpp>
#include <rmf_task_msgs/msg/dispatch_request.hpp>
#include <rmf_task_msgs/msg/dispatch_ack.hpp>

#include <rmf_task/agv/TaskPlanner.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/requests/Clean.hpp>
#include <rmf_task/BinaryPriorityScheme.hpp>

#include <rmf_fleet_msgs/msg/dock_summary.hpp>

#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include "Node.hpp"
#include "RobotContext.hpp"
#include "../TaskManager.hpp"

#include <rmf_traffic/schedule/Snapshot.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/LaneClosure.hpp>

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <iostream>
#include <unordered_set>
#include <optional>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// This abstract interface class allows us to use the same implementation of
/// FleetUpdateHandle whether we are running it in a distributed system or in a
/// single-process testing environment.
class ParticipantFactory
{
public:

  using ReadyCallback = std::function<void(rmf_traffic::schedule::Participant)>;

  virtual void async_make_participant(
    rmf_traffic::schedule::ParticipantDescription description,
    ReadyCallback ready_callback) = 0;

  virtual ~ParticipantFactory() = default;
};

//==============================================================================
class SimpleParticipantFactory : public ParticipantFactory
{
public:

  SimpleParticipantFactory(
    std::shared_ptr<rmf_traffic::schedule::Writer> writer)
  : _writer{std::move(writer)}
  {
    // Do nothing
  }

  void async_make_participant(
    rmf_traffic::schedule::ParticipantDescription description,
    ReadyCallback ready_callback) final
  {
    ready_callback(
      rmf_traffic::schedule::make_participant(
        std::move(description),
        _writer,
        nullptr)
    );
  }

private:
  std::shared_ptr<rmf_traffic::schedule::Writer> _writer;
};

//==============================================================================
class ParticipantFactoryRos2 : public ParticipantFactory
{
public:

  ParticipantFactoryRos2(
    rmf_traffic_ros2::schedule::WriterPtr writer)
  : _writer{std::move(writer)}
  {
    // Do nothing
  }

  void async_make_participant(
    rmf_traffic::schedule::ParticipantDescription description,
    ReadyCallback ready_callback) final
  {
    _writer->async_make_participant(
      std::move(description),
      std::move(ready_callback));
  }

private:
  rmf_traffic_ros2::schedule::WriterPtr _writer;
};

//==============================================================================
class FleetUpdateHandle::Implementation
{
public:

  std::string name;
  std::shared_ptr<std::shared_ptr<const rmf_traffic::agv::Planner>> planner;
  std::shared_ptr<Node> node;
  rxcpp::schedulers::worker worker;
  std::shared_ptr<ParticipantFactory> writer;
  std::shared_ptr<rmf_traffic::schedule::Snappable> snappable;
  std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation;

  // Task planner params
  std::shared_ptr<rmf_task::CostCalculator> cost_calculator =
    rmf_task::BinaryPriorityScheme::make_cost_calculator();
  std::shared_ptr<rmf_task::agv::TaskPlanner> task_planner = nullptr;

  rmf_utils::optional<rmf_traffic::Duration> default_maximum_delay =
    std::chrono::nanoseconds(std::chrono::seconds(10));

  AcceptDeliveryRequest accept_delivery = nullptr;
  std::unordered_map<RobotContextPtr,
    std::shared_ptr<TaskManager>> task_managers = {};

  rclcpp::Publisher<rmf_fleet_msgs::msg::FleetState>::SharedPtr
    fleet_state_pub = nullptr;
  rclcpp::TimerBase::SharedPtr fleet_state_timer = nullptr;

  // Map task id to pair of <RequestPtr, Assignments>
  using Assignments = rmf_task::agv::TaskPlanner::Assignments;

  // Map of dock name to dock parameters
  std::unordered_map<std::string,
    rmf_fleet_msgs::msg::DockParameter> dock_param_map = {};

  // TODO Support for various charging configurations
  std::unordered_set<std::size_t> charging_waypoints = {};

  double current_assignment_cost = 0.0;
  // Map to store task id with assignments for BidNotice
  std::unordered_map<std::string, Assignments> bid_notice_assignments = {};

  std::unordered_map<
    std::string, rmf_task::ConstRequestPtr> generated_requests = {};
  std::unordered_map<
    std::string, rmf_task::ConstRequestPtr> assigned_requests = {};
  std::unordered_set<std::string> cancelled_task_ids = {};
  using TaskProfileMsg = rmf_task_msgs::msg::TaskProfile;
  std::unordered_map<std::string, TaskProfileMsg> task_profile_map = {};

  AcceptTaskRequest accept_task = nullptr;

  using BidNotice = rmf_task_msgs::msg::BidNotice;
  using BidNoticeSub = rclcpp::Subscription<BidNotice>::SharedPtr;
  BidNoticeSub bid_notice_sub = nullptr;

  using BidProposal = rmf_task_msgs::msg::BidProposal;
  using BidProposalPub = rclcpp::Publisher<BidProposal>::SharedPtr;
  BidProposalPub bid_proposal_pub = nullptr;

  using DispatchRequest = rmf_task_msgs::msg::DispatchRequest;
  using DispatchRequestSub = rclcpp::Subscription<DispatchRequest>::SharedPtr;
  DispatchRequestSub dispatch_request_sub = nullptr;

  using DispatchAck = rmf_task_msgs::msg::DispatchAck;
  using DispatchAckPub = rclcpp::Publisher<DispatchAck>::SharedPtr;
  DispatchAckPub dispatch_ack_pub = nullptr;

  using DockSummary = rmf_fleet_msgs::msg::DockSummary;
  using DockSummarySub = rclcpp::Subscription<DockSummary>::SharedPtr;
  DockSummarySub dock_summary_sub = nullptr;

  template<typename... Args>
  static std::shared_ptr<FleetUpdateHandle> make(Args&& ... args)
  {
    auto handle = std::shared_ptr<FleetUpdateHandle>(new FleetUpdateHandle);
    handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
      Implementation{std::forward<Args>(args)...});

    handle->_pimpl->fleet_state_pub = handle->_pimpl->node->fleet_state();
    handle->_pimpl->fleet_state_timer =
      handle->_pimpl->node->try_create_wall_timer(
      std::chrono::seconds(1), [me = handle->weak_from_this()]()
      {
        if (const auto self = me.lock())
          self->_pimpl->publish_fleet_state();
      });

    // Create subs and pubs for bidding
    auto default_qos = rclcpp::SystemDefaultsQoS();
    auto transient_qos = rclcpp::QoS(10);  transient_qos.transient_local();

    // Publish BidProposal
    handle->_pimpl->bid_proposal_pub =
      handle->_pimpl->node->create_publisher<BidProposal>(
      BidProposalTopicName, default_qos);

    // Publish DispatchAck
    handle->_pimpl->dispatch_ack_pub =
      handle->_pimpl->node->create_publisher<DispatchAck>(
      DispatchAckTopicName, default_qos);

    // Subscribe BidNotice
    handle->_pimpl->bid_notice_sub =
      handle->_pimpl->node->create_subscription<BidNotice>(
      BidNoticeTopicName,
      default_qos,
      [w = handle->weak_from_this()](const BidNotice::SharedPtr msg)
      {
        if (const auto self = w.lock())
          self->_pimpl->bid_notice_cb(msg);
      });

    // Subscribe DispatchRequest
    handle->_pimpl->dispatch_request_sub =
      handle->_pimpl->node->create_subscription<DispatchRequest>(
      DispatchRequestTopicName,
      default_qos,
      [w = handle->weak_from_this()](const DispatchRequest::SharedPtr msg)
      {
        if (const auto self = w.lock())
          self->_pimpl->dispatch_request_cb(msg);
      });

    // Subscribe DockSummary
    handle->_pimpl->dock_summary_sub =
      handle->_pimpl->node->create_subscription<DockSummary>(
      DockSummaryTopicName,
      transient_qos,
      [w = handle->weak_from_this()](const DockSummary::SharedPtr msg)
      {
        if (const auto self = w.lock())
          self->_pimpl->dock_summary_cb(msg);
      });

    // Populate charging waypoints
    const auto& graph = (*handle->_pimpl->planner)->get_configuration().graph();
    for (std::size_t i = 0; i < graph.num_waypoints(); ++i)
    {
      if (graph.get_waypoint(i).is_charger())
        handle->_pimpl->charging_waypoints.insert(i);
    }

    return handle;
  }

  void dock_summary_cb(const DockSummary::SharedPtr& msg);

  void bid_notice_cb(const BidNotice::SharedPtr msg);

  void dispatch_request_cb(const DispatchRequest::SharedPtr msg);

  std::optional<std::size_t> get_nearest_charger(
    const rmf_traffic::agv::Planner::Start& start);

  /// Generate task assignments for a collection of task requests comprising of
  /// task requests currently in TaskManager queues while optionally including a
  /// new request and while optionally ignoring a specific request.
  std::optional<Assignments> allocate_tasks(
    rmf_task::ConstRequestPtr new_request = nullptr,
    rmf_task::ConstRequestPtr ignore_request = nullptr) const;

  /// Helper function to check if assignments are valid. An assignment set is
  /// invalid if one of the assignments has already begun execution.
  bool is_valid_assignments(Assignments& assignments) const;

  static Implementation& get(FleetUpdateHandle& fleet)
  {
    return *fleet._pimpl;
  }

  static const Implementation& get(const FleetUpdateHandle& fleet)
  {
    return *fleet._pimpl;
  }

  void fleet_state_publish_period(
    std::optional<rmf_traffic::Duration> value);

  void publish_fleet_state() const;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
