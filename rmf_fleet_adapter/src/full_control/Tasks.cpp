/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "Tasks.hpp"
#include "Actions.hpp"

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
class GenericTask : public Task
{
public:

  GenericTask(
      FleetAdapterNode* node,
      FleetAdapterNode::RobotContext* context,
      std::string task_id)
  : _node(node),
    _context(context),
    _task_id(std::move(task_id)),
    _submission_time(node->get_clock()->now())
  {
    // Do nothing
  }

  void fill_queue(std::queue<std::unique_ptr<Action>> action_queue)
  {
    _action_queue = std::move(action_queue);
  }

  void next()
  {
    if (!_start_time)
      _start_time = _node->get_clock()->now();

    if (_action_queue.empty())
    {
      std::cout << "no more actions - requesting next task" << std::endl;
      _action = nullptr;
      report_status();
      return _context->next_task();
    }

    _action = std::move(_action_queue.front());
    _action_queue.pop();
    _action->execute();
  }

  void interrupt()
  {
    if (!_action)
    {
      RCLCPP_WARN(
            _node->get_logger(),
            "No action for this task [" + id() + "] to interrupt. This might "
            "indicate a bug!");
      return;
    }

    _action->interrupt();
  }

  void resume()
  {
    if (!_action)
    {
      RCLCPP_WARN(
            _node->get_logger(),
            "No action for this task [" + id() + "] to resume. This might "
            "indicate a bug!");
      return;
    }

    _action->resume();
  }

  void resolve() final
  {
    if (!_action)
    {
      RCLCPP_WARN(
            _node->get_logger(),
            "No action for this task [" + id() + "] to resolve. This might "
            "indicate a bug!");
      return;
    }

    _action->resolve();
  }

  void report_status()
  {
    rmf_task_msgs::msg::TaskSummary summary;
    summary.task_id = id();
    summary.start_time = start_time();

    if (_action)
    {
      auto status = _action->get_status();
      summary.status = std::move(status.text);
      if (status.finish_estimate)
        summary.end_time = *status.finish_estimate;
      else
        summary.end_time = _node->now() + rclcpp::Duration(600);
      // TODO(MXG): Make better finish time estimates
    }
    else
    {
      summary.status = "Finished";
      summary.end_time = _node->now();
    }

    _node->task_summary_publisher->publish(summary);
  }

  void critical_failure(const std::string& error)
  {
    std::cout << " =============== CRITICAL FAILURE HAS OCCURRED" << std::endl;
    throw std::runtime_error("baaaaaaahhhhhhh");
    rmf_task_msgs::msg::TaskSummary summary;
    summary.task_id = id();
    summary.start_time = start_time();
    summary.submission_time = _submission_time;

    summary.status = "CRITICAL FAILURE: " + error;

    _node->task_summary_publisher->publish(summary);

    _context->discard_task(this);
  }

  const std::string& id() const
  {
    return _task_id;
  }

  const rclcpp::Time& start_time() const
  {
    return *_start_time;
  }

private:

  FleetAdapterNode* const _node;
  FleetAdapterNode::RobotContext* const _context;
  const std::string _task_id;
  std::unique_ptr<Action> _action;
  std::queue<std::unique_ptr<Action>> _action_queue;
  rmf_utils::optional<rclcpp::Time> _start_time;
  const rclcpp::Time _submission_time;

};

void report_impossible(
    FleetAdapterNode* const node,
    std::string task_id,
    std::string error)
{
  rmf_task_msgs::msg::TaskSummary summary;
  summary.status = std::move(error);
  summary.task_id = std::move(task_id);

  const auto now = node->get_clock()->now();
  summary.submission_time = now;
  summary.start_time = now;
  summary.end_time = now;

  node->task_summary_publisher->publish(std::move(summary));
}

//==============================================================================
std::unique_ptr<Task> make_delivery(
    FleetAdapterNode* const node,
    FleetAdapterNode::RobotContext* const context,
    rmf_task_msgs::msg::Delivery delivery)
{
  const auto& waypoint_keys = node->get_fields().graph_info.keys;
  const auto& dispenser_names = node->get_fields().graph_info.workcell_names;
  const auto& task_id = delivery.task_id;

  const auto pickup_wp = waypoint_keys.find(delivery.pickup_place_name);
  if (pickup_wp == waypoint_keys.end())
  {
    std::string error =
        "Unknown pickup location [" + delivery.pickup_place_name
        + "] in delivery request [" + delivery.task_id + "]";

    RCLCPP_ERROR(node->get_logger(), error);
    report_impossible(node, task_id, std::move(error));
    return nullptr;
  }

  const auto pickup_dispenser = dispenser_names.find(pickup_wp->second);
  if (pickup_dispenser == dispenser_names.end())
  {
    std::string error =
        "No known workcell available at pickup location ["
        + delivery.pickup_place_name + "] in delivery request ["
        + delivery.task_id + "]";

    RCLCPP_ERROR(node->get_logger(), error);
    report_impossible(node, task_id, std::move(error));
    return nullptr;
  }

  const auto dropoff_wp = waypoint_keys.find(delivery.dropoff_place_name);
  if (dropoff_wp == waypoint_keys.end())
  {
    std::string error =
        "Unknown dropoff location [" + delivery.dropoff_place_name + "] in "
        + "delivery request [" + delivery.task_id + "]";

    RCLCPP_ERROR(node->get_logger(), error);
    report_impossible(node, task_id, std::move(error));
    return nullptr;
  }

  const auto dropoff_dispenser = dispenser_names.find(dropoff_wp->second);
  if (dropoff_dispenser == dispenser_names.end())
  {
    std::string error =
        "No known workcell available at dropoff location ["
        + delivery.dropoff_place_name + "] in delivery request ["
        + delivery.task_id + "]";

    RCLCPP_ERROR(node->get_logger(), error);
    report_impossible(node, task_id, std::move(error));
    return nullptr;
  }

  auto task = std::make_unique<GenericTask>(node, context, task_id);

  std::queue<std::unique_ptr<Action>> action_queue;

  std::size_t move_id = 0;
  action_queue.push(
        make_move(node, context, task.get(), pickup_wp->second, move_id++));

  action_queue.push(
        make_dispense(node, context, task.get(), pickup_dispenser->second,
                      delivery.items, delivery.pickup_behavior));

  action_queue.push(
        make_move(node, context, task.get(), dropoff_wp->second, move_id++));

  action_queue.push(
        make_dispense(node, context, task.get(), dropoff_dispenser->second,
                      delivery.items, delivery.dropoff_behavior));

  task->fill_queue(std::move(action_queue));

  return std::move(task);
}


//==============================================================================
std::unique_ptr<Task> make_loop(
    FleetAdapterNode* node,
    FleetAdapterNode::RobotContext* context,
    rmf_task_msgs::msg::Loop loop)
{
  const auto& waypoint_keys = node->get_waypoint_keys();
  const auto& task_id = loop.task_id;

  const auto start_wp = waypoint_keys.find(loop.start_name);
  if (start_wp == waypoint_keys.end())
  {
    std::string error =
        "Unknown start location [" + loop.start_name + "] in loop request ["
        + task_id + "]";

    RCLCPP_ERROR(node->get_logger(), error);
    report_impossible(node, task_id, std::move(error));
    return nullptr;
  }

  const auto finish_wp = waypoint_keys.find(loop.finish_name);
  if (finish_wp == waypoint_keys.end())
  {
    std::string error =
        "Unknown finish location [" + loop.finish_name + "] in loop request ["
        + task_id + "]";

    RCLCPP_ERROR(node->get_logger(), error);
    report_impossible(node, task_id, std::move(error));
    return nullptr;
  }

  auto task = std::make_unique<GenericTask>(node, context, task_id);

  std::size_t move_id = 0;
  std::queue<std::unique_ptr<Action>> action_queue;
  for (std::size_t i=0; i < loop.num_loops; ++i)
  {
    action_queue.push(
          make_move(node, context, task.get(), start_wp->second, move_id++));

    action_queue.push(
          make_move(node, context, task.get(), finish_wp->second, move_id++));
  }

  task->fill_queue(std::move(action_queue));

  return std::move(task);
}

} // namespace full_control
} // namespace rmf_fleet_adapter
