/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP

#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/closed_lanes.hpp>

#include "Node.hpp"
#include <yaml-cpp/yaml.h>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class EasyCommandHandle
  : public RobotCommandHandle,
  public std::enable_shared_from_this<EasyCommandHandle>
{
public:

  using Navigate = EasyFullControl::Navigate;
  using GetRobotState = EasyFullControl::GetRobotState;
  using ProcessCompleted = EasyFullControl::ProcessCompleted;

  enum robotStatus {
    IDLE = 0,
    WAITING = 1,
    MOVING = 2
  };

  EasyCommandHandle(
    rclcpp::Node& node,
    std::string& fleet_name,
    std::string& robot_name,
    std::shared_ptr<const Graph> graph,
    std::shared_ptr<const VehicleTraits> traits,
    GetRobotState get_state,
    std::function<ProcessCompleted(const Navigate command)> navigate,
    std::function<ProcessCompleted()> stop,
    RobotUpdateHandle::ActionExecutor action_executor)
  : _node(&node),
    _robot_name(std::move(robot_name)),
    _fleet_name(std::move(fleet_name)),
    _graph(graph),
    _traits(traits),
    _get_state(std::move(get_state)),
    _navigate(std::move(navigate)),
    _stop(std::move(stop)),
    _action_executor(std::move(action_executor))
  {
    // Do nothing
  }

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) final
  {
    auto lock = _lock();
    _clear_last_command();

    RCLCPP_INFO(_node->get_logger(), "Received new path for %s", _robot_name.c_str());

    _remaining_waypoints = waypoints;
    assert(!next_arrival_estimator);
    assert(!path_finished_callback);
    _next_arrival_estimator = std::move(next_arrival_estimator);
    _path_finished_callback = std::move(path_finished_callback);
    _interrupted = false;

    while (!_remaining_waypoints.empty() || _status == MOVING || _status == WAITING)
    {
      // TODO: Check if we need to abort

      // State machine
      if (_status == IDLE)
      {
        // Assign the next waypoint
        _target_waypoint = _remaining_waypoints.front();
        _path_index = _target_waypoint->graph_index();
        // Move robot to next waypoint
        auto p = _target_waypoint->position();

        // The speed limit is set as the minimum of all the approach lanes' limits
        std::optional<double> speed_limit = std::nullopt;
        for (const auto& lane_idx : _target_waypoint->approach_lanes())
        {
          const auto& lane = _graph->get_lane(lane_idx);
          const auto& lane_limit = lane.properties().speed_limit();
          if (lane_limit.has_value())
          {
            if (speed_limit.has_value())
              speed_limit = std::min(speed_limit.value(), lane_limit.value());
            else
              speed_limit = lane_limit.value();
          }
        }
        Navigate current_command;
        current_command.pose = p;
        current_command.speed_limit = speed_limit;
        ProcessCompleted navigate_cb = _navigate(current_command);

        if (navigate_cb())
        {
          _remaining_waypoints.erase(_remaining_waypoints.begin());
          _status = MOVING;
        }
        else
        {
          RCLCPP_INFO(_node->get_logger(),
            "Robot %s failed to navigate to [%f, %f, %f] coordinates. Retrying...",
            _robot_name.c_str(), p.x(), p.y(), p.z());
        }

      }
      else if (_status == WAITING)
      {
        // TODO
      }
      else if (_status == MOVING)
      {
        // TODO
      }
    }

  }

  void stop() final
  {
    auto lock = _lock();
    _clear_last_command();
    ProcessCompleted stop_cb = _stop();

    if (!stop_cb())
      RCLCPP_INFO(_node->get_logger(),
        "Robot %s failed to stop.", _robot_name.c_str());
  }

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final
  {
    // TODO
  }

  void set_updater(RobotUpdateHandlePtr updater)
  {
    _updater = std::move(updater);

    // TODO

    // Set the action_executor for the robot
    // const auto teleop_executioner =
    //   [w = weak_from_this()](
    //   const std::string&,
    //   const nlohmann::json&,
    //   ActionExecution execution)
    //   {
    //     // We do not do anything here. The user can can move the robot by
    //     // sending PathRequest msgs. Instead we simply store the completed
    //     // callback which will be called when we receive a RobotModeRequest.
    //     const auto self = w.lock();
    //     if (!self)
    //       return;
    //     self->set_action_execution(execution);
    //   };
    // _updater->set_action_executor(teleop_executioner);
  }

  void newly_closed_lanes(const std::unordered_set<std::size_t>& closed_lanes)
  {
    // TODO
  }


private:
  rclcpp::Node* _node;
  const std::string& _robot_name;
  const std::string& _fleet_name;
  std::shared_ptr<const Graph> _graph;
  std::shared_ptr<const VehicleTraits> _traits;
  GetRobotState _get_state;
  std::function<ProcessCompleted(const Navigate command)> _navigate;
  std::function<ProcessCompleted()> _stop;
  RobotUpdateHandle::ActionExecutor _action_executor;
  RobotUpdateHandlePtr _updater;
  robotStatus _status;
  bool _interrupted = false;

  std::vector<rmf_traffic::agv::Plan::Waypoint> _requested_waypoints;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _remaining_waypoints;
  ArrivalEstimator _next_arrival_estimator;
  RequestCompleted _path_finished_callback;
  RequestCompleted _dock_finished_callback;
  rmf_utils::optional<std::size_t> _path_index;

  rmf_utils::optional<std::size_t> _last_known_lane_index;
  rmf_utils::optional<std::size_t> _last_known_waypoint_index;
  rmf_utils::optional<Graph::Waypoint> _on_waypoint;
  rmf_utils::optional<Graph::Lane> _on_lane;
  rmf_utils::optional<rmf_traffic::agv::Plan::Waypoint> _target_waypoint;
  rmf_utils::optional<std::size_t> _dock_waypoint_index;
  rmf_utils::optional<std::size_t> _action_waypoint_index;

  void _clear_last_command()
  {
    _next_arrival_estimator = nullptr;
    _path_finished_callback = nullptr;
    _dock_finished_callback = nullptr;
    _status = IDLE;
  }

  std::recursive_mutex _mutex;
  std::unique_lock<std::recursive_mutex> _lock()
  {
    std::unique_lock<std::recursive_mutex> lock(_mutex, std::defer_lock);
    while (!lock.try_lock())
    {
      // Intentionally busy wait
    }

    return lock;
  }
};

using EasyCommandHandlePtr = std::shared_ptr<EasyCommandHandle>;

//==============================================================================
class EasyFullControl::Implementation
{
public:

  Implementation(
    Configuration config_,
    AdapterPtr adapter_)
  : config{std::move(config_)}
  {
    adapter = adapter_;
    auto success = initialize_fleet();

    if (success)
    {
      RCLCPP_INFO(adapter_->node()->get_logger(), "Start Fleet Adapter");
      adapter_->start().wait();

      RCLCPP_INFO(adapter_->node()->get_logger(), "Closing Fleet Adapter");
      rclcpp::shutdown();
    }
    else
    {
      RCLCPP_ERROR(adapter_->node()->get_logger(), "Unable to initialize fleet");
    }
  }

  bool initialize_fleet();

// private:
  const Configuration config;
  AdapterPtr adapter;
  std::string fleet_name;
  FleetUpdateHandlePtr fleet_handle;
  YAML::Node fleet_config;
  std::shared_ptr<const Graph> graph;
  std::shared_ptr<const VehicleTraits> traits;
  rclcpp::Subscription<rmf_fleet_msgs::msg::LaneRequest>::SharedPtr lane_closure_request_sub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::ClosedLanes>::SharedPtr closed_lanes_pub;
  std::unordered_set<std::size_t> closed_lanes;
  std::unordered_map<std::string, EasyCommandHandlePtr> robots;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP
