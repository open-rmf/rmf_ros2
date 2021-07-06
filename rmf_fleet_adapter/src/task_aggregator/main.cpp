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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rmf_task_msgs/msg/tasks.hpp>
#include <rmf_task_msgs/msg/task_summary.hpp>


class TaskAggregator : public rclcpp::Node
{

  using TaskSummary = rmf_task_msgs::msg::TaskSummary;
  using Tasks = rmf_task_msgs::msg::Tasks;

public:
  TaskAggregator(
    std::string node_name,
    std::string input_topic,
    double rate)
  : Node(node_name),
    _rate(rate)
  {
    // Create a wall timer to periodically publish Tasks msg
    const double period = 1.0/_rate;
    auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::ratio<1>>(period));
    _timer = this->create_wall_timer(
      timer_period, std::bind(&TaskAggregator::timer_callback, this));

    // Create publisher for Tasks msg
    _tasks_pub = this->create_publisher<Tasks>(
      "/tasks",
      rclcpp::ServicesQoS());

    // Create subscription to receive TaskSummary msgs from fleet adapters
    _cb_group_task_summary = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_map_opt = rclcpp::SubscriptionOptions();
    sub_map_opt.callback_group = _cb_group_task_summary;
    auto qos_profile = rclcpp::QoS(10);
    _task_summary_sub = this->create_subscription<TaskSummary>(
      input_topic,
      qos_profile,
      std::bind(
        &TaskAggregator::task_summary_cb,
        this,
        std::placeholders::_1),
      sub_map_opt);

    RCLCPP_INFO(
      get_logger(),
      "Listening for Task Summaries on topic /%s",
      input_topic.c_str());
  }

private:

  void timer_callback()
  {
    Tasks tasks;

    for (const auto& t : _db)
      tasks.tasks.push_back(t.second);

    _tasks_pub->publish(tasks);
  }

  void task_summary_cb(const TaskSummary::SharedPtr msg)
  {
    if (_db.find(msg->task_id) != _db.end())
    {
      _db[msg->task_id] = *msg;
    }
    else
    {
      _db.insert(std::make_pair(msg->task_id, *msg));
    }
  }

  double _rate;

  std::unordered_map<std::string, TaskSummary> _db;

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<Tasks>::SharedPtr _tasks_pub;
  rclcpp::Subscription<TaskSummary>::SharedPtr _task_summary_sub;
  rclcpp::CallbackGroup::SharedPtr _cb_group_task_summary;
};

bool get_arg(
  const std::vector<std::string>& args,
  const std::string& key,
  std::string& value,
  const std::string& desc,
  const bool mandatory = true)
{
  const auto key_arg = std::find(args.begin(), args.end(), key);
  if (key_arg == args.end())
  {
    if (mandatory)
    {
      std::cerr << "You must specify a " << desc <<" using the " << key
                << " argument!" << std::endl;
    }
    return false;
  }
  else if (key_arg+1 == args.end())
  {
    std::cerr << "The " << key << " argument must be followed by a " << desc
              << "!" << std::endl;
    return false;
  }

  value = *(key_arg+1);
  return true;
}

int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
    rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string node_name = "task_aggregator";
  get_arg(args, "-n", node_name, "node name", false);

  std::string input_topic = rmf_fleet_adapter::TaskSummaryTopicName;
  get_arg(args, "-t", input_topic, "node name", false);

  std::string rate_string;
  get_arg(args, "-r", rate_string, "rate", false);
  double rate = rate_string.empty() ? 1.0 : std::stod(rate_string);

  auto task_aggregator_node = std::make_shared<TaskAggregator>(
    node_name,
    input_topic,
    rate);

  rclcpp::spin(task_aggregator_node);

  RCLCPP_INFO(
    task_aggregator_node->get_logger(),
    "Closing down");

  rclcpp::shutdown();
}
