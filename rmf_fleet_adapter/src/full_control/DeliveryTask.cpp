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

class DeliveryTask : public Task
{
public:

  using Delivery = rmf_task_msgs::msg::Delivery;

  DeliveryTask(
      FleetAdapterNode* node,
      FleetAdapterNode::RobotContext* context,
      const Delivery& request,
      std::size_t pickup_wp,
      std::string pickup_dispenser,
      std::size_t dropoff_wp,
      std::string dropoff_dispenser)
  : Task(node),
    _delivery(request),
    _node(node),
    _context(context)
  {
    _action_queue.push(
          make_move(node, context, this,
                    pickup_wp, node->get_parking_spots()));

    _action_queue.push(
          make_dispense(node, context, this, pickup_dispenser,
                        _delivery.items, _delivery.pickup_behavior));

    _action_queue.push(
          make_move(node, context, this,
                    dropoff_wp, node->get_parking_spots()));

    _action_queue.push(
          make_dispense(node, context, this, dropoff_dispenser,
                        _delivery.items, _delivery.dropoff_behavior));
  }

  void next()
  {
    if (!schedule.active())
      schedule.activate();

    if (!_start_time)
      _start_time = _node->get_clock()->now();

    if (_action_queue.empty())
    {
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
            "No action for this task [" + id() + "] to resume. This might "
            "indicate a bug!");
      return;
    }

    _action->resolve();
  }

  void report_status()
  {
    if (!_action)
    {
      RCLCPP_WARN(
            _node->get_logger(),
            "No action for this task [" + id() + "] to report a status for. "
            "This might indicate a bug!");
      return;
    }

    rmf_task_msgs::msg::TaskSummary summary;
    summary.task_id = id();
    summary.start_time = start_time();
    auto status = _action->get_status();
    summary.status = std::move(status.text);
    if (status.finish_estimate)
      summary.end_time = *status.finish_estimate;
    else
      summary.end_time = _node->now() + rclcpp::Duration(600);

    _node->task_summary_publisher->publish(summary);
  }

  void critical_failure(const std::string& error)
  {
    rmf_task_msgs::msg::TaskSummary summary;
    summary.task_id = id();
    summary.start_time = start_time();

    summary.status = "CRITICAL FAILURE: " + error;

    _node->task_summary_publisher->publish(summary);

    _context->discard_task(this);
  }

  const std::string& id() const
  {
    return _delivery.task_id;
  }

  const rclcpp::Time& start_time() const
  {
    return *_start_time;
  }

private:

  const Delivery _delivery;
  FleetAdapterNode* const _node;
  FleetAdapterNode::RobotContext* const _context;
  std::unique_ptr<Action> _action;
  std::queue<std::unique_ptr<Action>> _action_queue;
  rmf_utils::optional<rclcpp::Time> _start_time;

};

std::unique_ptr<Task> make_delivery(
    FleetAdapterNode* const node,
    FleetAdapterNode::RobotContext* const context,
    rmf_task_msgs::msg::Delivery delivery)
{
  const auto& waypoint_keys = node->get_fields().graph_info.keys;
  const auto& dispenser_names = node->get_fields().graph_info.workcell_names;

  const auto pickup_wp = waypoint_keys.find(delivery.pickup_place_name);
  if (pickup_wp == waypoint_keys.end())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Unknown pickup location [" + delivery.pickup_place_name + "] in "
          + "delivery request [" + delivery.task_id + "]");
    return nullptr;
  }

  const auto pickup_dispenser = dispenser_names.find(pickup_wp->second);
  if (pickup_dispenser == dispenser_names.end())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "No known workcell available at pickup location ["
          + delivery.pickup_place_name + "] in delivery request ["
          + delivery.task_id + "]");
    return nullptr;
  }

  const auto dropoff_wp = waypoint_keys.find(delivery.dropoff_place_name);
  if (dropoff_wp == waypoint_keys.end())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Unknown dropoff location [" + delivery.dropoff_place_name + "] in "
          + "delivery request [" + delivery.task_id + "]");
    return nullptr;
  }

  const auto dropoff_dispenser = dispenser_names.find(dropoff_wp->second);
  if (dropoff_dispenser == dispenser_names.end())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "No known workcell available at dropoff location ["
          + delivery.dropoff_place_name + "] in delivery request ["
          + delivery.task_id + "]");
    return nullptr;
  }

  return std::make_unique<DeliveryTask>(
        node, context, delivery,
        pickup_wp->second, pickup_dispenser->second,
        dropoff_wp->second, dropoff_dispenser->second);
}

} // namespace full_control
} // namespace rmf_fleet_adapter
