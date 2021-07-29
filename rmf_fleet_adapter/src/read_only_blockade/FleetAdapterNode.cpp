/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "FleetAdapterNode.hpp"

#include <rmf_traffic/DetectConflict.hpp>

#include <rmf_traffic/agv/Interpolate.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/executors.hpp>

#include "../rmf_fleet_adapter/load_param.hpp"

#include "../rmf_fleet_adapter/make_trajectory.hpp"

namespace rmf_fleet_adapter {
namespace read_only_blockade {

std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make()
{
  auto node = std::shared_ptr<FleetAdapterNode>(new FleetAdapterNode);

  const auto wait_time =
    get_parameter_or_default_time(*node, "discovery_timeout", 10.0);

  const std::string nav_graph_param_name = "nav_graph_file";
  const std::string graph_file =
    node->declare_parameter(nav_graph_param_name, std::string());
  if (graph_file.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", nav_graph_param_name.c_str());

    return nullptr;
  }

  auto graph = rmf_fleet_adapter::agv::parse_graph(graph_file, node->_traits);
  auto planner = rmf_traffic::agv::Planner(
    rmf_traffic::agv::Planner::Configuration(std::move(graph), node->_traits),
    rmf_traffic::agv::Planner::Options(nullptr)
  );

  node->_delay_threshold =
    get_parameter_or_default_time(*node, "delay_threshold", 5.0);

  auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
    *node, rmf_traffic::schedule::query_all());

  auto writer = rmf_traffic_ros2::schedule::Writer::make(*node);

  using namespace std::chrono_literals;

  const auto stop_time = std::chrono::steady_clock::now() + wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    bool ready = true;
    ready &= writer->ready();
    ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

    if (ready)
    {
      auto mirror = mirror_future.get();
      auto snapshot = mirror.snapshot_handle();
      node->_connect = Connections{
        std::move(writer),
        rmf_traffic_ros2::blockade::Writer::make(*node),
        std::move(mirror),
        rmf_traffic_ros2::schedule::Negotiation(*node, snapshot),
        std::move(planner)
      };

      node->_fleet_state_subscription =
        node->create_subscription<FleetState>(
        FleetStateTopicName, rclcpp::SystemDefaultsQoS(),
        [self = node.get()](FleetState::UniquePtr msg)
        {
          self->fleet_state_update(std::move(msg));
        });

      return node;
    }
  }

  return nullptr;
}

//==============================================================================
bool FleetAdapterNode::ignore_fleet(const std::string& fleet_name) const
{
  if (!_fleet_name.empty() && fleet_name != _fleet_name)
    return true;

  return false;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode()
: rclcpp::Node("fleet_adapter"),
  _fleet_name(get_fleet_name_parameter(*this)),
  _traits(get_traits_or_default(*this, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5))
{
  // Do nothing
}

//==============================================================================
void FleetAdapterNode::fleet_state_update(FleetState::UniquePtr new_state)
{
  if (ignore_fleet(new_state->name))
    return;

  std::lock_guard<std::mutex> lock(_async_mutex);
  for (const auto& robot : new_state->robots)
  {
    const auto insertion = _robots.insert(
      std::make_pair(robot.name, nullptr));

    if (insertion.second)
      register_robot(robot);
    else if (insertion.first->second)
      update_robot(robot, insertion.first);
  }
}

//==============================================================================
void FleetAdapterNode::register_robot(const RobotState& state)
{
  rmf_traffic::schedule::ParticipantDescription description{
    state.name,
    _fleet_name,
    rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
    _traits.profile()
  };

  _connect->schedule_writer->async_make_participant(
    std::move(description),
    [this](rmf_traffic::schedule::Participant participant)
    {
      std::lock_guard<std::mutex> lock(this->_async_mutex);
      const auto radius =
        participant.description().profile()
        .vicinity()->get_characteristic_length();

      auto blockade = this->_connect->blockade_writer->make_participant(
        participant.id(), radius,
        [](const auto, const auto&)
        {
          // We ignore new ranges because we don't really care about getting
          // permission from the blockade moderator.
        });

      this->_robots.at(participant.description().name()) =
        std::make_unique<Robot>(
          Robot{
            std::move(participant),
            std::move(blockade),
            std::nullopt,
            std::nullopt
          });
    });
}

} // namespace read_only_blockade
} // namespace rmf_fleet_adapter
