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

// Standard topic names for communicating with fleet drivers
#include <rmf_fleet_adapter/StandardNames.hpp>

// Fleet driver state/command messages
#include <rmf_fleet_msgs/msg/location.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/pause_request.hpp>

// Loop request message
#include <rmf_task_msgs/msg/loop.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic_ros2/Time.hpp>

// Utility functions for estimating where a robot is on the graph based on
// the information provided by fleet drivers.
#include "../rmf_fleet_adapter/estimation.hpp"

// Utility to help with modular arithmetic
#include <rmf_utils/Modular.hpp>

#include <deque>

namespace {
//==============================================================================
rmf_fleet_msgs::msg::Location make_location(
  rclcpp::Time t,
  const Eigen::Vector3d& p,
  const std::string& map_name,
  const std::size_t index,
  const std::optional<double>& speed_limit)
{
  const double approach_speed_limit = speed_limit.has_value() ?
    speed_limit.value() : 0.0;
  return rmf_fleet_msgs::build<rmf_fleet_msgs::msg::Location>()
    .t(t)
    .x(p[0]).y(p[1]).yaw(p[2])
    .obey_approach_speed_limit(speed_limit.has_value())
    .approach_speed_limit(approach_speed_limit)
    .level_name(map_name)
    .index(index);
}

} // anonymous namespace

//==============================================================================
class MockTrafficLightCommandHandle
  : public std::enable_shared_from_this<MockTrafficLightCommandHandle>
{
public:

  using PathRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr;

  using PauseRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::PauseRequest>::SharedPtr;

  using EasyTrafficLight = rmf_fleet_adapter::agv::EasyTrafficLight;

  MockTrafficLightCommandHandle(
    rclcpp::Node& node,
    std::string fleet_name,
    std::string robot_name,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits,
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    PathRequestPub path_request_pub,
    PauseRequestPub pause_request_pub)
  : _node(&node),
    _path_request_pub(std::move(path_request_pub)),
    _pause_request_pub(std::move(pause_request_pub))
  {
    _current_path_request.fleet_name = fleet_name;
    _current_path_request.robot_name = robot_name;

    _pause_request = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::PauseRequest>()
      .fleet_name(fleet_name)
      .robot_name(robot_name)
      .mode_request_id(0)
      .type(_pause_request.TYPE_RESUME)
      .at_checkpoint(0);

    _travel_info.graph = std::move(graph);
    _travel_info.traits = std::move(traits);
    _travel_info.fleet_name = std::move(fleet_name);
    _travel_info.robot_name = std::move(robot_name);

    _planner = planner;
  }

  void set_update_handle(
    rmf_fleet_adapter::agv::EasyTrafficLightPtr update_handle,
    rmf_fleet_msgs::msg::RobotState state)
  {
    _update = std::move(update_handle);
    _last_state = state;
    _update->fleet_state_publish_period(std::nullopt);
  }

  const std::string& name() const
  {
    return _travel_info.robot_name;
  }

  void update_state(const rmf_fleet_msgs::msg::RobotState& state)
  {
    auto lock = _lock();
    if (_last_state.has_value())
    {
      // Skip over old messages
      if (rmf_utils::modular(state.seq).less_than(_last_state->seq))
        return;
    }

    _last_state = state;

    if (!_update)
    {
      throw std::runtime_error(
              "Update handle has not been received by [" + _travel_info.robot_name
              + "] owned by [" + _travel_info.fleet_name + "]!");
    }

    if (!_moving)
      return;

    if (_pending_path_publish)
    {
      // Make sure that the robot knows it should wait at checkpoint 0 before
      // we issue the path request.
      if (_pause_request.mode_request_id == state.mode.mode_request_id)
        _send_new_path_command();
      else
        _pause_request_pub->publish(_pause_request);

      return;
    }

    if (_pause_request.mode_request_id != state.mode.mode_request_id)
    {
      // If the robot has the wrong mode request, then republish the command
      _pause_request_pub->publish(_pause_request);
      return;
    }

    if (_current_path_request.task_id != state.task_id)
    {
      // If the robot has the wrong path request ID, then resend this
      _path_request_pub->publish(_current_path_request);
      return;
    }

    if (state.mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR)
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "Uh oh!! An adapter error has occurred for [%s]. This really shouldn't "
        "happen for the traffic light, but there is a TODO inside the source "
        "code for how to fix this if it ever does occur.",
        state.name.c_str());
      // TODO(MXG): If this error ever gets printed, we can fix this issue by
      // changing this code block to resend the path request, but with the
      // first waypoint of the path request replaced by whatever the robot's
      // current location is.
      //
      // However, I'm leaving this code to just issue a warning, because this
      // really shouldn't happen, and the most likely cause would be if multiple
      // independent sources are issuing commands to the same robot at the same
      // time, which probably indicates that a different--more serious--bug is
      // happening somewhere else in the system.
      return;
    }

    if (state.path.empty())
    {
      if (_end_waypoint_index.has_value())
      {
        // We can ignore the output of this function since there's no way to
        // continue once the robot reaches the end. But we'll still call this
        // function so that the traffic light knows our robot has reached its
        // end goal.
        (void)(_update->waiting_at(_end_waypoint_index.value()));
      }

      _moving = false;
      _go_to_next_waypoint();
      return;
    }

    // Skip over old messages
    if (state.path.front().index < _update->last_reached())
    {
      return;
    }

    const std::size_t target = state.path.front().index;
    _last_target = target;
    const auto& l = state.location;
    const Eigen::Vector3d p{l.x, l.y, l.yaw};
    if (state.mode.mode == state.mode.MODE_PAUSED)
    {
      if (target == 0)
      {
        // If the current target is 0, then let's just assume that's where the
        // robot is waiting.
        _handle_waiting_at_instruction(_update->waiting_at(0));
        return;
      }

      if (_pause_request.type == _pause_request.TYPE_PAUSE_IMMEDIATELY)
      {
        _handle_waiting_after_instruction(_update->waiting_after(target-1, p));
        return;
      }

      if (_pause_request.type == _pause_request.TYPE_PAUSE_AT_CHECKPOINT)
      {
        if (_pause_request.at_checkpoint == target)
        {
          _handle_waiting_at_instruction(_update->waiting_at(target));
          return;
        }
      }

      _handle_waiting_after_instruction(_update->waiting_after(target-1, p));
      return;
    }

    if (target == 0)
    {
      // If the robot is moving towards checkpoint 0 (which implies checkpoint 0
      // is not the location that it actually started, which is suspicious, but
      // whatever), then let's just tell the traffic light that the robot is
      // waiting at checkpoint 0.
      _handle_waiting_at_instruction(_update->waiting_at(0));
      return;
    }

    _handle_moving_instruction(_update->moving_from(target-1, p), target);
  }

  std::size_t queue_size() const
  {
    return _queue.size();
  }

  void add_to_queue(const std::vector<std::string>& new_waypoints)
  {
    auto lock = _lock();

    const bool empty_queue = _queue.empty();

    for (const auto& wp : new_waypoints)
      _queue.push_back(wp);

    if (!_moving && empty_queue)
      _go_to_next_waypoint();
  }

  void pause_immediately()
  {
    auto lock = _lock();
    _internal_pause_immediately();
  }

  void resume()
  {
    auto lock = _lock();
    _internal_resume();
  }

  rmf_utils::optional<std::string> start_waypoint() const
  {
    if (_moving || !_queue.empty() || !_last_state.has_value())
      return rmf_utils::nullopt;

    const auto& l = _last_state->location;
    const auto starts = rmf_traffic::agv::compute_plan_starts(
      _planner->get_configuration().graph(),
      l.level_name,
      {l.x, l.y, l.yaw},
      rmf_traffic_ros2::convert(_node->now()));

    if (starts.empty())
      return rmf_utils::nullopt;

    const auto* name = _planner->get_configuration().graph().get_waypoint(
      starts.front().waypoint()).name();

    if (name)
      return *name;

    return rmf_utils::nullopt;
  }

private:

  void _go_to_next_waypoint()
  {
    while (!_queue.empty())
    {
      const std::string next = _queue.front();

      const auto& l = _last_state.value().location;
      const auto starts = rmf_traffic::agv::compute_plan_starts(
        _planner->get_configuration().graph(),
        l.level_name,
        {l.x, l.y, l.yaw},
        rmf_traffic_ros2::convert(_node->now()));

      if (starts.empty())
      {
        RCLCPP_ERROR(
          _node->get_logger(),
          "Robot [%s] is lost!",
          _travel_info.robot_name.c_str());
        return;
      }

      const auto goal =
        _planner->get_configuration().graph().find_waypoint(next);
      if (!goal)
      {
        RCLCPP_ERROR(
          _node->get_logger(),
          "Could not find a goal named [%s] for robot [%s]",
          next.c_str(), _travel_info.robot_name.c_str());

        _queue.pop_front();
        continue;
      }

      const auto result = _planner->plan(starts, goal->index());
      if (!result)
      {
        RCLCPP_ERROR(
          _node->get_logger(),
          "Could not find a plan to get robot [%s] to waypoint [%s]",
          _travel_info.robot_name.c_str(), next.c_str());

        _queue.pop_front();
        continue;
      }

      if (result->get_waypoints().size() < 2)
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "[%s] owned by [%s] skipping queued waypoint goal of [%s] "
          "because it is already there.",
          _travel_info.robot_name.c_str(),
          _travel_info.fleet_name.c_str(),
          _queue.front().c_str());

        _queue.pop_front();
        continue;
      }

      _follow_new_path(result->get_waypoints());
      return;
    }
  }

  void _handle_moving_instruction(
    const EasyTrafficLight::MovingInstruction instruction,
    const std::size_t target)
  {
    if (instruction ==
      EasyTrafficLight::MovingInstruction::WaitAtNextCheckpoint)
    {
      _internal_pause_at_checkpoint(target);
      return;
    }

    if (instruction ==
      EasyTrafficLight::MovingInstruction::ContinueAtNextCheckpoint)
    {
      _internal_resume();
      return;
    }

    if (instruction == EasyTrafficLight::MovingInstruction::PauseImmediately)
    {
      _internal_pause_immediately();
      return;
    }

    if (instruction == EasyTrafficLight::MovingInstruction::MovingError)
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "Uh oh! Received a MovingError for robot [%s] of fleet [%s]",
        _travel_info.robot_name.c_str(), _travel_info.fleet_name.c_str());
    }
  }

  void _handle_waiting_instruction(
    const EasyTrafficLight::WaitingInstruction instruction,
    bool ready_for_next_target)
  {
    if (instruction == EasyTrafficLight::WaitingInstruction::Resume)
    {
      _internal_resume(ready_for_next_target);
      return;
    }

    if (instruction == EasyTrafficLight::WaitingInstruction::WaitingError)
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "Uh oh! Received a WaitingError for robot [%s] of fleet [%s]",
        _travel_info.robot_name.c_str(), _travel_info.fleet_name.c_str());
    }
  }

  void _handle_waiting_at_instruction(
    const EasyTrafficLight::WaitingInstruction instruction)
  {
    _handle_waiting_instruction(instruction, true);
  }

  void _handle_waiting_after_instruction(
    const EasyTrafficLight::WaitingInstruction instruction)
  {
    _handle_waiting_instruction(instruction, false);
  }

  void _follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints)
  {
    _moving = true;
    _last_target = 0;
    _current_path_request.path.clear();

    const auto& graph = _travel_info.graph;
    std::string first_level;
    for (const auto& wp : waypoints)
    {
      if (wp.graph_index())
      {
        first_level = graph->get_waypoint(wp.graph_index().value())
          .get_map_name();
        break;
      }
    }

    const auto get_map_name = [&](rmf_utils::optional<std::size_t> index)
      {
        if (index.has_value())
          return graph->get_waypoint(index.value()).get_map_name();

        return first_level;
      };

    std::vector<rmf_fleet_adapter::agv::Waypoint> new_path;
    std::vector<std::optional<double>> speed_limits;
    new_path.reserve(waypoints.size());


    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
      const auto& wp = waypoints[i];
      std::optional<double> speed_limit = std::nullopt;
      if (i > 0)
      {
        const auto last_x = new_path.back().position().x();
        const auto last_y = new_path.back().position().y();
        const auto delta =
          (wp.position().block<2, 1>(0, 0)
          - Eigen::Vector2d(last_x, last_y)).norm();

        if (delta < 0.01)
          continue;

        // Populate speed limits vector based on the minimum speed limit of
        // the lanes approaching this waypoint
        for (const auto& lane_idx : wp.approach_lanes())
        {
          const auto& lane = _travel_info.graph->get_lane(lane_idx);
          const auto& lane_limit = lane.properties().speed_limit();
          if (lane_limit.has_value())
          {
            if (speed_limit.has_value())
              speed_limit = std::min(speed_limit.value(), lane_limit.value());
            else
              speed_limit = lane_limit.value();
          }
        }
      }

      const auto p = wp.position();
      const auto map_name = get_map_name(wp.graph_index());
      speed_limits.push_back(speed_limit);
      new_path.emplace_back(map_name, p);
    }

    const auto now = _node->now();
    _current_path_request.path.push_back(_last_state.value().location);
    _current_path_request.path.front().index = 0;
    for (std::size_t i = 0; i < new_path.size(); ++i)
    {
      const auto& wp = new_path[i];
      _current_path_request.path.push_back(
        make_location(now, wp.position(), wp.map_name(), i, speed_limits[i]));
    }

    RCLCPP_INFO(
      _node->get_logger(),
      "Moving [%s] owned by [%s] towards waypoint [%s]",
      _travel_info.robot_name.c_str(),
      _travel_info.fleet_name.c_str(),
      _queue.front().c_str());

    _end_waypoint_index = new_path.size() - 1;
    _pending_path_publish = true;
    _update->follow_new_path(new_path);

    // We want the robot to wait at its first checkpoint until we get the
    // go-ahead for it to move forward. So first we issue this pause request,
    // and then once we know the robot will remain paused at waypoint 0, then
    // we will issue the path command.
    _internal_pause_at_checkpoint(0);
  }

  void _internal_pause_immediately()
  {
    if (_pause_request.type == _pause_request.TYPE_PAUSE_IMMEDIATELY)
      return;

    ++_pause_request.mode_request_id;
    _pause_request.type = _pause_request.TYPE_PAUSE_IMMEDIATELY;
    _pause_request_pub->publish(_pause_request);
  }

  void _internal_pause_at_checkpoint(const std::size_t checkpoint)
  {
    if (_pause_request.type == _pause_request.TYPE_PAUSE_AT_CHECKPOINT)
    {
      // Don't needlessly duplicate the command
      if (_pause_request.at_checkpoint == checkpoint)
        return;
    }

    ++_pause_request.mode_request_id;

    if (checkpoint == _current_path_request.path.back().index)
    {
      // The traffic light will always tell us to stop at the last waypoint, but
      // we don't want to instruct the slotcar plugin to pause there, because
      // then it will keep the last waypoint in its future path forever, and
      // we'll never be able to move to the next queue item. When we get this
      // command, we'll just tell the slotcar to resume until the end.
      _pause_request.type = _pause_request.TYPE_RESUME;
    }
    else
    {
      _pause_request.type = _pause_request.TYPE_PAUSE_AT_CHECKPOINT;
      _pause_request.at_checkpoint = checkpoint;
    }

    _pause_request_pub->publish(_pause_request);
  }

  void _internal_resume(bool to_next_target = false)
  {
    if (_pause_request.type == _pause_request.TYPE_RESUME)
      return;

    std::size_t target = to_next_target ?
      _last_target.value()+1 : _last_target.value();

    _internal_pause_at_checkpoint(target);
  }

  void _send_new_path_command()
  {
    _current_path_request.task_id = std::to_string(++_path_command_version);
    _path_request_pub->publish(_current_path_request);
    _pending_path_publish = false;
  }

  std::deque<std::string> _queue;

  rclcpp::Node* _node;
  PathRequestPub _path_request_pub;
  PauseRequestPub _pause_request_pub;
  rmf_fleet_msgs::msg::PathRequest _current_path_request;
  rmf_fleet_msgs::msg::PauseRequest _pause_request;

  bool _pending_path_publish = false;
  bool _moving = false;

  std::chrono::steady_clock::time_point _path_requested_time;
  TravelInfo _travel_info;
  std::shared_ptr<const rmf_traffic::agv::Planner> _planner;
  std::optional<rmf_fleet_msgs::msg::RobotState> _last_state;
  std::optional<std::size_t> _last_target;

  std::size_t _path_command_version = 0;

  std::optional<std::size_t> _end_waypoint_index;

  rmf_fleet_adapter::agv::EasyTrafficLightPtr _update;

  std::mutex _mutex;
  std::unique_lock<std::mutex> _lock()
  {
    std::unique_lock<std::mutex> lock(_mutex, std::defer_lock);
    while (!lock.try_lock())
    {
      // Intentionally busy wait
    }

    return lock;
  }
};

using MockTrafficLightCommandHandlePtr =
  std::shared_ptr<MockTrafficLightCommandHandle>;

//==============================================================================
/// This is an RAII class that keeps the connections to the fleet driver alive.
struct Connections : public std::enable_shared_from_this<Connections>
{
  /// The API for running the fleet adapter
  rmf_fleet_adapter::agv::AdapterPtr adapter;

  /// The navigation graph for the robot
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  std::shared_ptr<const rmf_traffic::agv::Planner> planner;

  /// The traits of the vehicles
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits;

  /// The topic subscription for responding to new fleet states
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
    fleet_state_sub;

  /// The topic subscription for receiving loop requests
  rclcpp::Subscription<rmf_task_msgs::msg::Loop>::SharedPtr
    loop_sub;

  /// The publisher for sending out path requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr
    path_request_pub;

  rclcpp::Publisher<rmf_fleet_msgs::msg::PauseRequest>::SharedPtr
    pause_request_pub;

  /// The publisher for sending out mode requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr
    mode_request_pub;

  std::unordered_map<std::string, MockTrafficLightCommandHandlePtr> robots;

  std::unordered_set<std::string> received_task_ids;

  void add_traffic_light(
    const std::string& fleet_name,
    const rmf_fleet_msgs::msg::RobotState& state)
  {
    const std::string robot_name = state.name;
    const auto command = std::make_shared<MockTrafficLightCommandHandle>(
      *adapter->node(), fleet_name, robot_name, graph, traits, planner,
      path_request_pub, pause_request_pub);

    auto pause = [w = command->weak_from_this()]()
      {
        if (const auto command = w.lock())
          command->pause_immediately();
      };

    auto resume = [w = command->weak_from_this()]()
      {
        if (const auto command = w.lock())
          command->resume();
      };

    adapter->add_easy_traffic_light(
      [c = weak_from_this(), command, robot_name = robot_name, state](
        rmf_fleet_adapter::agv::EasyTrafficLightPtr updater)
      {
        const auto connections = c.lock();
        if (!connections)
          return;

        auto  lock = connections->lock();
        command->set_update_handle(std::move(updater), state);
        connections->robots[robot_name] = command;
      },
      fleet_name, robot_name, *traits,
      std::move(pause), std::move(resume));
  }

  std::mutex _mutex;
  std::unique_lock<std::mutex> lock()
  {
    std::unique_lock<std::mutex> l(_mutex, std::defer_lock);
    while (!l.try_lock())
    {
      // Intentional busy wait
    }

    return l;
  }
};

//==============================================================================
std::shared_ptr<Connections> make_fleet(
  const rmf_fleet_adapter::agv::AdapterPtr& adapter)
{
  const auto& node = adapter->node();
  std::shared_ptr<Connections> connections = std::make_shared<Connections>();
  connections->adapter = adapter;

  const std::string fleet_name_param_name = "fleet_name";
  const std::string fleet_name = node->declare_parameter(
    "fleet_name", std::string());
  if (fleet_name.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", fleet_name_param_name.c_str());

    return nullptr;
  }

  connections->traits = std::make_shared<rmf_traffic::agv::VehicleTraits>(
    rmf_fleet_adapter::get_traits_or_default(
      *node, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5));

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

  auto graph =
    std::make_shared<rmf_traffic::agv::Graph>(
    rmf_fleet_adapter::agv::parse_graph(graph_file, *connections->traits));

  // We add pseudo-events on every lane to force the planner to include every
  // intermediate waypoint in its plan.
  for (std::size_t i = 0; i < graph->num_lanes(); ++i)
  {
    graph->get_lane(i).exit().event(
      rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::Wait(std::chrono::seconds(0))));
  }

  connections->graph = std::move(graph);
  std::cout << "The fleet [" << fleet_name
            << "] has the following named waypoints:\n";
  for (const auto& key : connections->graph->keys())
    std::cout << " -- " << key.first << std::endl;

  connections->planner =
    std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Planner::Configuration(
      *connections->graph,
      *connections->traits),
    rmf_traffic::agv::Planner::Options(nullptr));

  connections->path_request_pub = node->create_publisher<
    rmf_fleet_msgs::msg::PathRequest>(
    rmf_fleet_adapter::PathRequestTopicName, rclcpp::SystemDefaultsQoS());

  connections->pause_request_pub = node->create_publisher<
    rmf_fleet_msgs::msg::PauseRequest>(
    rmf_fleet_adapter::PauseRequestTopicName, rclcpp::SystemDefaultsQoS());

  connections->mode_request_pub = node->create_publisher<
    rmf_fleet_msgs::msg::ModeRequest>(
    rmf_fleet_adapter::ModeRequestTopicName, rclcpp::SystemDefaultsQoS());

  connections->fleet_state_sub = node->create_subscription<
    rmf_fleet_msgs::msg::FleetState>(
    rmf_fleet_adapter::FleetStateTopicName,
    rclcpp::SystemDefaultsQoS(),
    [c = std::weak_ptr<Connections>(connections), fleet_name](
      const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
    {
      if (msg->name != fleet_name)
        return;

      const auto connections = c.lock();
      if (!connections)
        return;

      for (const auto& state : msg->robots)
      {
        const auto insertion = connections->robots.insert(
          {state.name, nullptr});

        const bool new_robot = insertion.second;
        if (new_robot)
        {
          RCLCPP_INFO(
            connections->adapter->node()->get_logger(),
            "Adding robot [%s] to the fleet",
            state.name.c_str());

          // We have not seen this robot before, so let's add it to the fleet.
          connections->add_traffic_light(fleet_name, state);
        }

        const auto& command = insertion.first->second;
        if (command)
        {
          // We are commanding this robot, so let's update its state
          command->update_state(state);
        }
      }
    });

  connections->loop_sub = node->create_subscription<
    rmf_task_msgs::msg::Loop>(
    rmf_fleet_adapter::LoopRequestTopicName,
    rclcpp::SystemDefaultsQoS(),
    [c = std::weak_ptr<Connections>(connections), fleet_name](
      const rmf_task_msgs::msg::Loop::SharedPtr msg)
    {
      if (msg->robot_type != fleet_name)
        return;

      const auto connections = c.lock();
      if (!connections)
        return;

      if (!connections->received_task_ids.insert(msg->task_id).second)
        return;

      MockTrafficLightCommandHandlePtr best = nullptr;
      for (const auto& r : connections->robots)
      {
        const auto start = r.second->start_waypoint();
        if (!start)
          continue;

        if (*start == msg->start_name)
        {
          best = r.second;
          break;
        }
      }

      if (!best)
      {
        // This is a very dumb approach to assigning the tasks, but this adapter is
        // only meant for testing purposes anyway.
        for (const auto& r : connections->robots)
        {
          const auto& candidate = r.second;
          if (!best || candidate->queue_size() < best->queue_size())
            best = candidate;
        }

        if (!best)
        {
          RCLCPP_ERROR(
            connections->adapter->node()->get_logger(),
            "No robot is available to fulfill the loop task request [%s]",
            msg->task_id.c_str());
          return;
        }
      }

      std::vector<std::string> new_waypoints;
      new_waypoints.reserve(2*msg->num_loops);
      for (std::size_t i = 0; i < msg->num_loops; ++i)
      {
        new_waypoints.push_back(msg->start_name);
        new_waypoints.push_back(msg->finish_name);
      }

      RCLCPP_INFO(
        connections->adapter->node()->get_logger(),
        "Assigning task [%s] to robot [%s]",
        msg->task_id.c_str(),
        best->name().c_str());

      best->add_to_queue(new_waypoints);
    });

  return connections;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("fleet_adapter");
  if (!adapter)
    return 1;

  const auto fleet_connections = make_fleet(adapter);
  if (!fleet_connections)
    return 1;

  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Mock Traffic Light");

  adapter->start().wait();

  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Mock Traffic Light");

  rclcpp::shutdown();
}
