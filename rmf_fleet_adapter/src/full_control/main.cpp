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

// Internal implementation-specific headers
#include "../rmf_fleet_adapter/ParseArgs.hpp"
#include "../rmf_fleet_adapter/load_param.hpp"

// Public rmf_fleet_adapter API headers
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>

// Fleet driver state/command messages
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>

// Utilities for making traffic predictions
#include <rmf_traffic/agv/Interpolate.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic_ros2/Time.hpp>

//==============================================================================
class FleetDriverRobotCommandHandle
    : public rmf_fleet_adapter::agv::RobotCommandHandle
{
public:

  using PathRequestPub =
      rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr;

  using ModeRequestPub =
      rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr;

  FleetDriverRobotCommandHandle(
      rclcpp::Node& node,
      std::string robot_name,
      std::string fleet_name,
      std::shared_ptr<const rmf_traffic::agv::Graph> graph,
      std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits,
      PathRequestPub path_request_pub,
      ModeRequestPub mode_request_pub)
    : _node(&node),
      _graph(std::move(graph)),
      _traits(std::move(traits)),
      _path_request_pub(std::move(path_request_pub)),
      _mode_request_pub(std::move(mode_request_pub))
  {
    _current_path_request.robot_name = robot_name;
    _current_path_request.fleet_name = fleet_name;

    _current_dock_request.robot_name = std::move(robot_name);
    _current_dock_request.fleet_name = std::move(fleet_name);

    rmf_fleet_msgs::msg::ModeParameter p;
    p.name = "docking";
    _current_dock_request.parameters.push_back(std::move(p));
  }

  void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      ArrivalEstimator next_arrival_estimator,
      RequestCompleted path_finished_callback) final
  {
    _clear_last_command();

    _current_waypoints = waypoints;
    _next_arrival_estimator = std::move(next_arrival_estimator);
    _path_finished_callback = std::move(path_finished_callback);

    _current_path_request.task_id = std::to_string(++_current_task_id);
    _current_path_request.path.clear();
    for (const auto& wp : waypoints)
    {
      rmf_fleet_msgs::msg::Location location;
      const Eigen::Vector3d p = wp.position();
      location.t = rmf_traffic_ros2::convert(wp.time());
      location.x = p.x();
      location.y = p.y();
      location.yaw = p.z();

      // Note: if the waypoint is not on a graph index, then we'll just leave
      // the level_name blank. That information isn't likely to get used by the
      // fleet driver anyway.
      if (wp.graph_index())
      {
        location.level_name =
            _graph->get_waypoint(*wp.graph_index()).get_map_name();
      }

      _current_path_request.path.emplace_back(std::move(location));
    }

    _path_requested_time = std::chrono::steady_clock::now();
    _path_request_pub->publish(_current_path_request);
  }

  void stop() final
  {
    // This is currently not used by the fleet drivers
  }

  void dock(
      const std::string& dock_name,
      RequestCompleted docking_finished_callback) final
  {
    _clear_last_command();

    _dock_finished_callback = std::move(docking_finished_callback);
    _current_dock_request.parameters.front().value = dock_name;
    _current_dock_request.task_id = ++_current_task_id;

    _dock_requested_time = std::chrono::steady_clock::now();
    _mode_request_pub->publish(_current_dock_request);
  }

  void update_state(const rmf_fleet_msgs::msg::RobotState& state)
  {
    if (_path_finished_callback)
    {
      // If we have a _path_finished_callback, then the robot should be
      // following a path

      // There should not be a docking command happening
      assert(!_dock_finished_callback);

      if (state.task_id != _current_path_request.task_id)
      {
        // The robot has not received our path request yet
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::milliseconds(200) < now - _path_requested_time)
        {
          // We published the request a while ago, so we'll send it again in
          // case it got dropped.
          _path_requested_time = now;
          _path_request_pub->publish(_current_path_request);
        }

        return _estimate_state(state);
      }

      if (state.path.empty())
        return _handle_path_finish(state);

      return _handle_path_traveling(state);
    }
    else if (_dock_finished_callback)
    {
      // If we have a _dock_finished_callback, then the robot should be docking
    }
    else
    {
      // If we don't have a finishing callback, then the robot is not under our
      // command
      _estimate_state(state);
    }
  }

private:

  rclcpp::Node* _node;
  std::shared_ptr<const rmf_traffic::agv::Graph> _graph;
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> _traits;

  rmf_fleet_msgs::msg::PathRequest _current_path_request;
  std::chrono::steady_clock::time_point _path_requested_time;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _current_waypoints;
  ArrivalEstimator _next_arrival_estimator;
  RequestCompleted _path_finished_callback;
  PathRequestPub _path_request_pub;
  rmf_utils::optional<std::size_t> _last_known_wp;
  rmf_utils::optional<std::size_t> _last_known_lane;

  rmf_fleet_msgs::msg::ModeRequest _current_dock_request;
  std::chrono::steady_clock::time_point _dock_requested_time;
  RequestCompleted _dock_finished_callback;
  ModeRequestPub _mode_request_pub;

  rmf_fleet_adapter::agv::RobotUpdateHandlePtr _updater;
  uint32_t _current_task_id = 0;

  void _clear_last_command()
  {
    _next_arrival_estimator = nullptr;
    _path_finished_callback = nullptr;
    _dock_finished_callback = nullptr;
  }

  void _handle_path_finish(const rmf_fleet_msgs::msg::RobotState& state)
  {
    // The robot believes it has reached the end of its path.
    const auto& wp = _current_waypoints.back();
    const auto& l = state.location;
    const Eigen::Vector2d p{l.x, l.y};
    const double dist = (p - wp.position().block<2,1>(0, 0)).norm();

    assert(wp.graph_index());
    _last_known_wp = *wp.graph_index();

    assert(_current_waypoints.size() > 2);

    if (dist > 2.0)
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "The robot is very far [%fm] from where it is supposed to be, but "
        "its remaining path is empty. This means the robot believes it is"
        "finished, but it is not where it is supposed to be.", dist);
      return _estimate_state(state);
    }

    if (dist > 0.5)
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "The robot is somewhat far [%fm] from where it is supposed to be, "
        "but we will proceed anyway.");

      const auto& last_wp = _current_waypoints[_current_waypoints.size()-2];
      _midlane_state(state.location, last_wp, wp);
    }
    else
    {
      // We are close enough to the goal that we will say the robot is
      // currently located there.
      _updater->update_position(*wp.graph_index(), l.yaw);
      _last_known_wp = *wp.graph_index();
    }

    _path_finished_callback();
    return _clear_last_command();
  }

  void _handle_path_traveling(const rmf_fleet_msgs::msg::RobotState& state)
  {
    const std::size_t remaining_count = state.path.size();
    const std::size_t i_target_wp = _current_waypoints.size() - remaining_count;
    const auto& target_wp = _current_waypoints[i_target_wp];

    const auto& l = state.location;
    const auto& p = target_wp.position();
    const auto interp = rmf_traffic::agv::Interpolate::positions(
          *_traits, std::chrono::steady_clock::now(), {{l.x, l.y, l.yaw}, p});
    const auto next_arrival = interp.back().time() - interp.front().time();
    _next_arrival_estimator(i_target_wp, next_arrival);

    if (i_target_wp > 1)
      return _midlane_state(l, _current_waypoints[i_target_wp-1], target_wp);


  }

  void _midlane_state(
      const rmf_fleet_msgs::msg::Location& l,
      const rmf_traffic::agv::Plan::Waypoint& last_wp,
      const rmf_traffic::agv::Plan::Waypoint& target_wp)
  {
    if (last_wp.graph_index())
    {
      std::vector<std::size_t> lanes;
      const auto last_gi = *last_wp.graph_index();
      const auto target_gi = *target_wp.graph_index();
      const auto* forward_lane = _graph->lane_from(last_gi, target_gi);
      assert(forward_lane);
      lanes.push_back(forward_lane->index());

      if (const auto* reverse_lane = _graph->lane_from(target_gi, last_gi))
        lanes.push_back(reverse_lane->index());

      _updater->update_position({l.x, l.y, l.yaw}, std::move(lanes));
      return;
    }

    // The target should always have a graph index, because only the first
    // waypoint in a command should ever be lacking a graph index.
    assert(target_wp.graph_index());
    _updater->update_position({l.x, l.y, l.yaw}, *target_wp.graph_index());
  }

  void _estimate_state(const rmf_fleet_msgs::msg::RobotState& state)
  {
    // This function is used when we need to estimate the state of the robot
    // because it's not obvious where it intends to be on the graph

  }
};

using FleetDriverRobotCommandHandlePtr =
  std::shared_ptr<FleetDriverRobotCommandHandle>;

//==============================================================================
/// This is an RAII class that keeps the connections to the fleet driver alive.
struct Connections
{
  /// The API for adding new robots to the adapter
  rmf_fleet_adapter::agv::FleetUpdateHandlePtr fleet;

  /// The navigation graph for the robot
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  /// The topic subscription for responding to new fleet states
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
  fleet_state_sub;

  /// The publisher for sending out path requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr
  path_request_pub;

  /// The publisher for sending out mode requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr
  mode_request_pub;

  /// The container for robot update handles
  std::unordered_map<std::string, FleetDriverRobotCommandHandlePtr>
  robots;
};

//==============================================================================
rmf_utils::optional<Connections> make_fleet(
    const rmf_fleet_adapter::agv::AdapterPtr& adapter)
{
  const auto& node = adapter->node();
  Connections connections;

  const std::string nav_graph_param_name = "nav_graph_file";
  const std::string graph_file =
      node->declare_parameter(nav_graph_param_name, std::string());
  if (graph_file.empty())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Missing [%s] parameter", nav_graph_param_name.c_str());

    return rmf_utils::nullopt;
  }

  const std::string fleet_name_param_name = "fleet_name";
  const std::string fleet_name = node->declare_parameter(
        "fleet_name", std::string());
  if (fleet_name.empty())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Missing [%s] parameter", fleet_name_param_name.c_str());

    return rmf_utils::nullopt;
  }

  auto traits = rmf_fleet_adapter::get_traits_or_default(
        *node, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5);

  auto graph = rmf_fleet_adapter::agv::parse_graph(graph_file, traits);

  connections.fleet = adapter->add_fleet(
        fleet_name, std::move(traits), std::move(graph));

  // If the perform_deliveries parameter is true, then we just blindly accept
  // all delivery requests.
  if (node->declare_parameter<bool>("perform_deliveries", false))
  {
    connections.fleet->accept_delivery_requests(
          [](const rmf_task_msgs::msg::Delivery&){ return true; });
  }



  return connections;
}

//==============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("fleet_adapter");
  if (!adapter)
    return 1;

  const auto fleet_connections = make_fleet(adapter);
  if (!fleet_connections)
    return 1;

  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Fleet Adapter");

  // Start running the adapter and wait until it gets stopped by SIGINT
  adapter->start().wait();

  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();
}
