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

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

using RobotState = rmf_fleet_msgs::msg::RobotState;
using FleetState = rmf_fleet_msgs::msg::FleetState;

class RobotStateAggregator : public rclcpp::Node
{
public:

  static std::shared_ptr<RobotStateAggregator> make()
  {
    const auto node = std::shared_ptr<RobotStateAggregator>(
      new RobotStateAggregator);

    const auto prefix = node->declare_parameter("robot_prefix", "");
    const auto fleet_name = node->declare_parameter("fleet_name", "");
    if (fleet_name.empty())
    {
      RCLCPP_FATAL(
        node->get_logger(),
        "Missing required parameter: [fleet_adapter]");
      return nullptr;
    }

    node->_prefix = std::move(prefix);
    node->_fleet_name = std::move(fleet_name);

    return node;
  }

private:

  RobotStateAggregator()
  : rclcpp::Node("robot_state_aggregator")
  {
    const auto default_qos = rclcpp::SystemDefaultsQoS();
    const auto sensor_qos = rclcpp::SensorDataQoS();

    _fleet_state_pub = create_publisher<FleetState>(
      rmf_fleet_adapter::FleetStateTopicName, default_qos);

    _robot_state_sub = create_subscription<RobotState>(
      "robot_state", sensor_qos,
      [&](RobotState::UniquePtr msg)
      {
        _robot_state_update(std::move(msg));
      });
  }

  std::string _prefix;
  std::string _fleet_name;

  std::unordered_map<std::string, std::unique_ptr<RobotState>> _latest_states;

  rclcpp::Publisher<FleetState>::SharedPtr _fleet_state_pub;

  rclcpp::Subscription<RobotState>::SharedPtr _robot_state_sub;
  void _robot_state_update(RobotState::UniquePtr msg)
  {
    const std::string& name = msg->name;
    if (name.size() < _prefix.size())
      return;

    if (name.substr(0, _prefix.size()) != _prefix)
      return;

    const auto insertion = _latest_states.insert(std::make_pair(name, nullptr));
    const auto it = insertion.first;
    bool updated = false;
    if (insertion.second)
    {
      it->second = std::move(msg);
      updated = true;
    }
    else
    {
      if (rclcpp::Time(it->second->location.t) < rclcpp::Time(msg->location.t) )
      {
        it->second = std::move(msg);
        updated = true;
      }
    }

    if (updated)
    {
      FleetState fleet;
      fleet.name = _fleet_name;
      for (const auto& robot_state : _latest_states)
        fleet.robots.emplace_back(*robot_state.second);

      _fleet_state_pub->publish(fleet);
    }
  }

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto node = RobotStateAggregator::make();
  if (!node)
    return 1;

  rclcpp::spin(node);
  rclcpp::shutdown();
}
