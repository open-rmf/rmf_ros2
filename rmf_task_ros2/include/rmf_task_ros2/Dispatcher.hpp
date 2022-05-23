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

#ifndef RMF_TASK_ROS2__DISPATCHER_HPP
#define RMF_TASK_ROS2__DISPATCHER_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rmf_task_ros2/DispatchState.hpp>

// Deprecated message definition
#include <rmf_task_msgs/msg/task_description.hpp>

namespace rmf_task_ros2 {

//==============================================================================
/// This dispatcher class holds an instance which handles the dispatching of
/// tasks to all downstream RMF fleet adapters.
class Dispatcher : public std::enable_shared_from_this<Dispatcher>
{
public:
  using DispatchStates = std::unordered_map<TaskID, DispatchStatePtr>;

  /// Initialize an rclcpp context and make an dispatcher instance. This will
  /// instantiate an rclcpp::Node, a task dispatcher node. Dispatcher node will
  /// allow you to dispatch submitted task to the best fleet/robot within RMF.
  ///
  /// \param[in] dispatcher_node_name
  ///   The ROS 2 node to manage the Dispatching of Task
  ///
  /// \sa init_and_make_node()
  static std::shared_ptr<Dispatcher> init_and_make_node(
    const std::string dispatcher_node_name);

  /// Similarly this will init the dispatcher, but you will also need to init
  /// rclcpp via rclcpp::init(~).
  ///
  /// \param[in] dispatcher_node_name
  ///   The ROS 2 node to manage the Dispatching of Task
  ///
  /// \sa make_node()
  static std::shared_ptr<Dispatcher> make_node(
    const std::string dispatcher_node_name);

  /// Create a dispatcher by providing the ros2 node
  ///
  /// \param[in] node
  ///   ROS 2 node instance
  ///
  /// \sa make()
  static std::shared_ptr<Dispatcher> make(
    const std::shared_ptr<rclcpp::Node>& node);

  /// Submit task to dispatcher node. Calling this function will immediately
  /// trigger the bidding process, then the task "action". Once submmitted,
  /// Task State will be in 'Pending' State, till the task is awarded to a fleet
  /// then the state will turn to 'Queued'
  ///
  /// \param [in] task_description
  ///   Submit a task to dispatch
  ///
  /// \return task_id
  ///   self-generated task_id, nullopt is submit task failed
  [[deprecated]]
  std::optional<TaskID> submit_task(
    const rmf_task_msgs::msg::TaskDescription& task_description);

  /// Cancel an active task which was previously submitted to Dispatcher. This
  /// will terminate the task with a State of: `Canceled`. If a task is
  /// `Queued` or `Executing`, this function will send a cancel req to
  /// the respective fleet adapter. It is the responsibility of the fleet adapter
  /// to make sure it cancels the task internally.
  ///
  /// \param [in] task_id
  ///   Task to cancel
  ///
  /// \return true if success
  bool cancel_task(const TaskID& task_id);

  /// Check the state of a submited task. It can be either active or terminated
  ///
  /// \param [in] task_id
  ///   task_id obtained from `submit_task()`
  ///
  /// \return State of the task, nullopt if task is not available
  std::optional<DispatchState> get_dispatch_state(
    const TaskID& task_id) const;

  /// Get a mutable ref of active tasks map list handled by dispatcher
  const DispatchStates& active_dispatches() const;

  /// Get a mutable ref of terminated tasks map list
  const DispatchStates& finished_dispatches() const;

  using DispatchStateCallback =
    std::function<void(const DispatchState& status)>;

  /// Trigger this callback when a task status is changed. This will return the
  /// Changed task status.
  ///
  /// \param [in] callback function
  void on_change(DispatchStateCallback on_change_fn);

  /// Change the default evaluator to a custom evaluator, which is used by
  /// bidding auctioneer. Default evaluator is: `QuickestFinishEvaluator`
  ///
  /// \param [in] evaluator
  ///   evaluator used to select the best bid from fleets
  void evaluator(bidding::Auctioneer::ConstEvaluatorPtr evaluator);

  /// Get the rclcpp::Node that this dispatcher will be using for communication.
  std::shared_ptr<rclcpp::Node> node();

  /// spin dispatcher node
  void spin();

  class Implementation;

private:
  Dispatcher();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__DISPATCHER_HPP
