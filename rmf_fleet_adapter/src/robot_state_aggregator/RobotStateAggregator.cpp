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
#include "rclcpp_components/register_node_macro.hpp"

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#ifdef FAILOVER_MODE
#include "stubborn_buddies_msgs/msg/status.hpp"
#endif

using RobotState = rmf_fleet_msgs::msg::RobotState;
using FleetState = rmf_fleet_msgs::msg::FleetState;


#ifdef FAILOVER_MODE
constexpr char DEFAULT_ACTIVE_STATUS_NAME[] = "status";
#endif

class RobotStateAggregator : public rclcpp::Node
{
public:

  explicit RobotStateAggregator(const rclcpp::NodeOptions& options)
  : rclcpp::Node("robot_state_aggregator", options)
  {
    RCLCPP_DEBUG(get_logger(), "RobotStateAggregator called");
    const auto default_qos = rclcpp::SystemDefaultsQoS();
    const auto state_qos = rclcpp::SystemDefaultsQoS().keep_last(100);

#ifdef FAILOVER_MODE
    _active_node = this->declare_parameter("active_node", true);
    _failover_mode = this->declare_parameter("failover_mode", false);
    _buddy_subns = this->declare_parameter("buddy_subns", "yang");
    _namespace = this->declare_parameter("namespace", "node_ns");

    _active_status_topic = "/" + _namespace +
      "/" + _buddy_subns + "/"
      + DEFAULT_ACTIVE_STATUS_NAME;
#endif

    _fleet_state_pub = create_publisher<FleetState>(
      rmf_fleet_adapter::FleetStateTopicName, default_qos);
// *INDENT-OFF*
#ifdef FAILOVER_MODE
    if (_active_node)
    {
#endif
      _robot_state_sub = create_subscription<RobotState>(
        "/robot_state", state_qos,
        [&](RobotState::UniquePtr msg)
        {
          _robot_state_update(std::move(msg));
        });
#ifdef FAILOVER_MODE
    }
    else
    {
      _inactive_state_sub = create_subscription
        <stubborn_buddies_msgs::msg::Status>(
        _active_status_topic,
        10,
        [this](const typename stubborn_buddies_msgs::msg::
        Status::SharedPtr msg) -> void
        {
          const auto state_qos = rclcpp::SensorDataQoS();
          RCLCPP_DEBUG(get_logger(), "Watchdog rised at %s, "
          "self activation triggered",
          _active_status_topic.c_str(),
          msg->stamp.sec);
          this->set_parameter(rclcpp::Parameter("active_node", true));
          _active_node = true;

          _robot_state_sub = create_subscription<RobotState>(
            "/robot_state", state_qos,
            [&](RobotState::UniquePtr msg)
            {
              _robot_state_update(std::move(msg));
            });
        });
    }
#endif
// *INDENT-ON*
    const auto prefix = this->declare_parameter("robot_prefix", "");
    const auto fleet_name = this->declare_parameter("fleet_name", "");

    if (fleet_name.empty())
    {
      RCLCPP_FATAL(
        this->get_logger(),
        "Missing required parameter: [fleet_name]");
      exit(1);
    }

    this->_prefix = std::move(prefix);
    this->_fleet_name = std::move(fleet_name);
  }

private:
  bool _active_node;

  std::string _prefix;
  std::string _fleet_name;
  bool _failover_mode;
#ifdef FAILOVER_MODE
  std::string _active_status_topic;
  std::string _buddy_subns;
  std::string _namespace;
#endif

  std::unordered_map<std::string, std::unique_ptr<RobotState>> _latest_states;

  rclcpp::Publisher<FleetState>::SharedPtr _fleet_state_pub;
  rclcpp::Subscription<RobotState>::SharedPtr _robot_state_sub;

#ifdef FAILOVER_MODE
  rclcpp::Subscription<stubborn_buddies_msgs::msg::Status>::SharedPtr
    _inactive_state_sub;
#endif

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

RCLCPP_COMPONENTS_REGISTER_NODE(RobotStateAggregator)
