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


#include "LaneBlocker.hpp"
#include "IntersectionChecker.hpp"

#include <rmf_traffic_ros2/agv/Graph.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp_components/register_node_macro.hpp>

#include <thread>
#include <mutex>

#ifndef NDEBUG
#include <iostream>
#endif

//==============================================================================
namespace {

IntersectionChecker::CollisionGeometry make_collision_geometry(
  const rmf_traffic::agv::Graph& graph,
  const rmf_traffic::agv::Graph::Lane& lane,
  const double lane_width)
{
  IntersectionChecker::CollisionGeometry geometry;
  const Eigen::Vector2d& entry_loc =
    graph.get_waypoint(lane.entry().waypoint_index()).get_location();

  const Eigen::Vector2d& exit_loc =
    graph.get_waypoint(lane.exit().waypoint_index()).get_location();

  const auto& center_loc = (exit_loc + entry_loc) * 0.5;
  const auto& axis = (exit_loc - entry_loc);
  double theta = std::atan2(std::abs(axis[1]), std::abs(axis[0]));
  if (theta > M_PI)
    theta = M_PI -theta;
  if (theta < -M_PI)
    theta = M_PI + theta;
  const double length = axis.norm();

  geometry.center.x = center_loc[0];
  geometry.center.y = center_loc[1];
  geometry.center.theta = theta;
  geometry.size_y = lane_width;
  geometry.size_x = length;
  return geometry;
}

IntersectionChecker::CollisionGeometry make_collision_geometry(
  const LaneBlocker::BoundingBox& obstacle)
{
  IntersectionChecker::CollisionGeometry geometry;

  const auto& p = obstacle.center.position;
  const auto& q = obstacle.center.orientation;
  // Convert quaternion to yaw
  const double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  geometry.center.theta = std::atan2(siny_cosp, cosy_cosp);

  geometry.center.x = p.x;
  geometry.center.y = p.y;
  geometry.size_x = obstacle.size.x;
  geometry.size_y = obstacle.size.y;

  return geometry;
}

} //namespace anonymous
//==============================================================================
LaneBlocker::LaneBlocker(const rclcpp::NodeOptions& options)
: Node("lane_blocker_node", options)
{
  _tf2_buffer =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());

  if (_tf2_buffer != nullptr)
  {
    _transform_listener =
      std::make_shared<tf2_ros::TransformListener>(*_tf2_buffer);
  }

  _rmf_frame = this->declare_parameter("rmf_frame_id", "map");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter rmf_frame_id to %s", _rmf_frame.c_str()
  );

  _obstacle_lane_threshold = this->declare_parameter(
    "obstacle_lane_threshold", 0.25);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter obstacle_lane_threshold to %f", _obstacle_lane_threshold
  );

  _lane_width = this->declare_parameter(
    "lane_width", 0.5);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter lane_width to %f", _lane_width
  );

  _tf2_lookup_duration = this->declare_parameter(
    "tf2_lookup_duration", 0.5);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter tf2_lookup_duration to %f", _tf2_lookup_duration
  );

  const double process_rate = this->declare_parameter("process_rate", 1.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter process_rate to %f hz", process_rate
  );

  const double cull_rate = this->declare_parameter("cull_rate", 0.1);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter cull_rate to %f hz", cull_rate
  );

  std::size_t search_millis =
    this->declare_parameter("max_search_millis", 1000);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter max_search_millis to %ld milliseconds", search_millis
  );
  _max_search_duration = std::chrono::milliseconds(search_millis);

  _continuous_checker =
    this->declare_parameter("continuous_checker", false);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter continuous_checker to %s", _continuous_checker ? "true" : "false"
  );

  _lane_closure_threshold =
    this->declare_parameter("lane_closure_threshold", 5);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter lane_closure_threshold to %ld", _lane_closure_threshold
  );

  _speed_limit_threshold =
    this->declare_parameter("speed_limit_threshold", 3);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter speed_limit_threshold to %ld", _speed_limit_threshold
  );

  _speed_limit =
    this->declare_parameter("speed_limit", 0.5);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter speed_limit to %f", _speed_limit
  );

  auto process_timer_period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(1.0 / process_rate));
  _process_timer = this->create_wall_timer(
    std::move(process_timer_period),
    [=]()
    {
      this->process();
    });

  _cull_timer_period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(1.0 / cull_rate));
  _cull_timer = this->create_wall_timer(
    _cull_timer_period,
    [=]()
    {
      this->cull();
    });

  _lane_closure_pub = this->create_publisher<LaneRequest>(
    rmf_fleet_adapter::LaneClosureRequestTopicName,
    rclcpp::SystemDefaultsQoS());

  _speed_limit_pub = this->create_publisher<SpeedLimitRequest>(
    rmf_fleet_adapter::SpeedLimitRequestTopicName,
    rclcpp::SystemDefaultsQoS());

  _obstacle_sub = this->create_subscription<Obstacles>(
    "rmf_obstacles",
    rclcpp::QoS(10).best_effort(),
    [=](Obstacles::ConstSharedPtr msg)
    {
      obstacle_cb(*msg);
    });

  // Selectively disable intra-process comms for non-volatile subscriptions
  // so that this node can be run in a container with intra-process comms.
  auto transient_qos = rclcpp::QoS(10).transient_local();
  rclcpp::SubscriptionOptionsWithAllocator<
    std::allocator<void>> ipc_sub_options;
  ipc_sub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Disable;

  _graph_sub = this->create_subscription<NavGraph>(
    "nav_graphs",
    transient_qos,
    [=](NavGraph::ConstSharedPtr msg)
    {
      if (msg->name.empty())
        return;
      auto traffic_graph = rmf_traffic_ros2::convert(*msg);
      if (!traffic_graph.has_value())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Unable to convert NavGraph from fleet %s into a traffic graph",
          msg->name.c_str()
        );
      }
      _traffic_graphs[msg->name] = std::move(traffic_graph.value());
      for (std::size_t i = 0; i < _traffic_graphs[msg->name].num_lanes(); ++i)
      {
        std::string lane_key = get_lane_key(msg->name, i);
        if (_internal_lane_states.find(lane_key) == _internal_lane_states.end())
        {
          _internal_lane_states.insert({lane_key, LaneState::Normal});
        }
      }
    },
    ipc_sub_options);

  _lane_states_sub = this->create_subscription<LaneStates>(
    "lane_states",
    transient_qos,
    [=](LaneStates::ConstSharedPtr msg)
    {
      if (msg->fleet_name.empty())
        return;
      _lane_states[msg->fleet_name] = msg;
    },
    ipc_sub_options);

  RCLCPP_INFO(
    this->get_logger(),
    "Started lane_blocker node"
  );
}

//==============================================================================
void LaneBlocker::obstacle_cb(const Obstacles& msg)
{
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;

  if (msg.obstacles.empty() || _transform_listener == nullptr)
    return;

  // TODO(YV): Consider using tf2_ros::MessageFilter instead of this callback
  for (const auto& obstacle : msg.obstacles)
  {
    const auto& obstacle_frame = obstacle.header.frame_id;
    std::string tf2_error;
    const bool can_transform = _tf2_buffer->canTransform(
      _rmf_frame,
      obstacle_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(_tf2_lookup_duration),
      &tf2_error);

    // TODO(YV): Cache these transforms since most of them would be static
    if (!can_transform)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Unable to lookup transform between between obstacle frame %s and RMF "
        "frame %s.", obstacle_frame.c_str(), _rmf_frame.c_str()
      );
      continue;
    }

    const auto& transform = _tf2_buffer->lookupTransform(
      _rmf_frame,
      obstacle_frame,
      tf2::TimePointZero
    );

    // doTransform only works on Stamped messages
    const auto before_pose = geometry_msgs::build<PoseStamped>()
      .header(obstacle.header)
      .pose(obstacle.bbox.center);
    const auto before_size = geometry_msgs::build<Vector3Stamped>()
      .header(obstacle.header)
      .vector(obstacle.bbox.size);

    PoseStamped after_pose;
    Vector3Stamped after_size;
    tf2::doTransform(before_pose, after_pose, transform);
    tf2::doTransform(before_size, after_size, transform);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Pose of obstacle id %d in RMF %s frame is [%f, %f, %f]",
      obstacle.id, _rmf_frame.c_str(),
      after_pose.pose.position.x,
      after_pose.pose.position.y, after_pose.pose.position.z
    );
    auto new_box =
      rmf_obstacle_msgs::build<rmf_obstacle_msgs::msg::BoundingBox3D>()
      .center(std::move(after_pose.pose))
      .size(std::move(after_size.vector));

    ObstacleData obs{
      rclcpp::Time(obstacle.header.stamp) + rclcpp::Duration(obstacle.lifetime),
      obstacle.id,
      obstacle.source,
      std::move(new_box)
    };

    // Add to obstacle queue for processing in a separate thread/callback
    const auto& obs_key = LaneBlocker::get_obstacle_key(obs);
    _obstacle_buffer[obs_key] = std::move(obs);
  }
}

//==============================================================================
void LaneBlocker::process()
{
  if (_obstacle_buffer.empty())
    return;

  // Keep track of which lanes were updated to decided whether to modify lane
  // state.
  std::unordered_set<std::string> lanes_with_changes = {};
  // Map obstacle_id with list of lanes it is no longer in the vicinity of
  std::unordered_map<std::string,
    std::unordered_set<std::string>> obstacles_with_changes = {};

  for (const auto& [obstacle_key, obstacle] : _obstacle_buffer)
  {
    // If the lifetime of the obstacle has passed cull() will handle the purging
    if (obstacle.expiry_time < get_clock()->now())
    {
      continue;
    }

    // If this obstacle was previously assigned to a lane,
    // check if it is still in the vicinity of that lane
    auto obs_lane_it = _obstacle_to_lanes_map.find(obstacle_key);
    if (obs_lane_it != _obstacle_to_lanes_map.end())
    {
      const auto& lanes_keys = obs_lane_it->second;
      RCLCPP_INFO(
        this->get_logger(),
        "Obstacle %s was previously in the vicinity of %ld lanes",
        obstacle_key.c_str(), lanes_keys.size());

      // Check if obstacle is still in the vicinity of these lanes.
      for (const auto& lane_key : lanes_keys)
      {
        const auto [fleet_name, lane_id] = deserialize_key(lane_key);
        if (_traffic_graphs.find(fleet_name) == _traffic_graphs.end())
        {
          RCLCPP_ERROR(
            this->get_logger(),
            "Lane %s which belongs to fleet %s does not have a traffic graph "
            "This bug should be reported.",
            lane_key.c_str(), fleet_name.c_str()
          );
          continue;
        }
        double how_much;
        const auto& traffic_graph = _traffic_graphs.at(fleet_name);
        const auto& lane = traffic_graph.get_lane(lane_id);
        const auto& o1 = make_collision_geometry(
          traffic_graph,
          lane,
          _lane_width);
        const auto& o2 = make_collision_geometry(obstacle.transformed_bbox);
        auto intersect = IntersectionChecker::between(
          o1,
          o2,
          how_much
        );
        if (intersect || how_much <= _obstacle_lane_threshold)
        {
          // Obstacle is still in the vicinity of this lane
          RCLCPP_INFO(
            this->get_logger(),
            "Obstacle %s is still in the vicinity of lane %s",
            obstacle_key.c_str(), lane_key.c_str()
          );
        }
        else
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Obstacle %s is no longer in the vicinity of lane %s. "
            "Updating cache...", obstacle_key.c_str(), lane_key.c_str()
          );
          // Obstacle is no longer in the vicinity and needs to be removed
          // Remove from _obstacle_to_lanes_map
          obstacles_with_changes[obstacle_key].insert(lane_key);
        }
      }
    }

    if (obs_lane_it == _obstacle_to_lanes_map.end() || _continuous_checker)
    {
      // New obstacle or re-check current obstacle.
      // It needs to be assigned a lane if within the vicinity of one.
      RCLCPP_INFO(
        this->get_logger(),
        "Obstacle %s was not previously in the vicinity of any lane. Checking "
        "for any changes", obstacle_key.c_str()
      );
      std::unordered_set<std::string> vicinity_lane_keys = {};
      std::mutex mutex;
      auto search_vicinity_lanes =
        [&vicinity_lane_keys, &mutex](
        const std::string& fleet_name,
        const TrafficGraph& graph,
        const ObstacleData& obstacle,
        const double threshold,
        const double lane_width,
        const std::chrono::nanoseconds max_duration)
        {
          const auto start_time = std::chrono::steady_clock::now();
          const auto max_time = start_time + max_duration;
          for (std::size_t i = 0; i < graph.num_lanes(); ++i)
          {
            if (std::chrono::steady_clock::now() > max_time)
              return;
            const auto& lane = graph.get_lane(i);
            double how_much;
            const auto& o1 = make_collision_geometry(
              graph,
              lane,
              lane_width);
            const auto& o2 = make_collision_geometry(
              obstacle.transformed_bbox);
            auto intersect = IntersectionChecker::between(
              o1,
              o2,
              how_much
            );
            if (intersect || how_much < threshold)
            {
              std::lock_guard<std::mutex> lock(mutex);
              vicinity_lane_keys.insert(
                LaneBlocker::get_lane_key(fleet_name, i));
            }
          }
          #ifndef NDEBUG
          const auto finish_time = std::chrono::steady_clock::now();
          std::cout << "Obstacle " << obstacle.id
                    << " search in graph for fleet " << fleet_name << " took "
                    << (finish_time - start_time).count() /1e6
                    << " ms" << std::endl;
          #endif

        };
      std::vector<std::thread> search_threads = {};
      for (const auto& [fleet_name, graph] : _traffic_graphs)
      {
        search_threads.push_back(
          std::thread(
            search_vicinity_lanes,
            fleet_name,
            graph,
            obstacle,
            _obstacle_lane_threshold,
            _lane_width,
            _max_search_duration)
        );
      }

      for (auto& t : search_threads)
      {
        if (t.joinable())
          t.join();
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Search concluded with %ld lanes in the vicinity of obstacle %s",
        vicinity_lane_keys.size(), obstacle_key.c_str()
      );

      // Update caches
      for (const auto& lane_key : vicinity_lane_keys)
      {
        // new obstacle
        if (obs_lane_it == _obstacle_to_lanes_map.end())
        {
          _obstacle_to_lanes_map[obstacle_key].insert(lane_key);
          _lane_to_obstacles_map[lane_key].insert(obstacle_key);
          lanes_with_changes.insert(lane_key);
        }
        // current obstacle
        else
        {
          const auto& existing_lane_keys = obs_lane_it->second;
          if (existing_lane_keys.find(lane_key) == existing_lane_keys.end())
          {
            _obstacle_to_lanes_map[obstacle_key].insert(lane_key);
            _lane_to_obstacles_map[lane_key].insert(obstacle_key);
            lanes_with_changes.insert(lane_key);
          }
        }
      }
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "There are %ld lanes with changes to the number of obstacles in their "
    "vicinity", lanes_with_changes.size()
  );

  // Remove obstacles from lanes
  for (const auto& [obstacle_key, lane_ids] : obstacles_with_changes)
  {
    for (const auto& lane_id : lane_ids)
    {
      _lane_to_obstacles_map[lane_id].erase(obstacle_key);
      _obstacle_to_lanes_map[obstacle_key].erase(lane_id);
      if (_obstacle_to_lanes_map[obstacle_key].empty())
        _obstacle_to_lanes_map.erase(obstacle_key);
      lanes_with_changes.insert(lane_id);
    }
  }


  request_lane_modifications(std::move(lanes_with_changes));
}

//==============================================================================
void LaneBlocker::request_lane_modifications(
  const std::unordered_set<std::string>& changes)
{
  if (changes.empty())
    return;

  // A map to collate lanes per fleet that need to be closed
  std::unordered_map<std::string, std::unique_ptr<LaneRequest>> lane_req_msgs;
  // A map to collate lanes per fleet that need speed limits
  std::unordered_map<std::string, std::unique_ptr<SpeedLimitRequest>> speed_limit_req_msgs;
  // For now we implement a simple heuristic to decide whether to close a lane
  // or not.
  for (const auto& lane_key : changes)
  {
    if (_lane_to_obstacles_map.find(lane_key) == _lane_to_obstacles_map.end())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "[LaneBlocker::request_lane_modifications()]: key error. This is a "
        "bug and should be reported."
      );
      continue;
    }
    const auto& obstacles = _lane_to_obstacles_map.at(lane_key);
    const auto& lane_state = _internal_lane_states.at(lane_key);
    if (obstacles.size() >= _lane_closure_threshold && lane_state == LaneState::Normal)
    {
      transition_lane_state(lane_state, LaneState::Closed, lane_key, lane_req_msgs, speed_limit_req_msgs);
    }
    else
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Lane %s has %ld obstacles in its vicinity but will not be closed or speed limited as "
        "the closure threshold is %ld and the speed limit threshold is %ld",
        lane_key.c_str(), obstacles.size(), _lane_closure_threshold, _speed_limit_threshold
      );
      continue;
    }
  }

  // Publish lane closures
  for (auto& [_, msg] : lane_req_msgs)
  {
    if (msg->close_lanes.empty())
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "None of the lanes for fleet %s need to be closed",
        msg->fleet_name.c_str()
      );
      continue;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Requested %ld lanes to close for fleet %s",
      msg->close_lanes.size(), msg->fleet_name.c_str()
    );
    _lane_closure_pub->publish(std::move(msg));
  }
}

//==============================================================================
void LaneBlocker::transition_lane_state(
  LaneState old_state,
  LaneState new_state,
  std::string lane_key,
  std::unordered_map<std::string, std::unique_ptr<LaneRequest>> &lane_req_msgs,
  std::unordered_map<std::string, std::unique_ptr<SpeedLimitRequest>> &speed_limit_req_msgs)
{
  if (new_state == old_state)
  {
    return;
  }

  auto [fleet_name, lane_id] = deserialize_key(lane_key);

  if (old_state == LaneState::Normal && new_state == LaneState::Closed)
  {
    // construct Lane Closure msg
    auto msg_it = lane_req_msgs.insert({fleet_name, nullptr});
    if (msg_it.second)
    {
      LaneRequest request;
      request.fleet_name = std::move(fleet_name);
      request.close_lanes.push_back(std::move(lane_id));
      msg_it.first->second = std::make_unique<LaneRequest>(
        std::move(request)
      );
    }
    else
    {
      // Msg was created before. We simply append the new lane id
      msg_it.first->second->close_lanes.push_back(std::move(lane_id));
    }
  }
  else if (old_state == LaneState::Closed && new_state == LaneState::Normal)
  {
    // construct Lane Open msg
    auto msg_it = lane_req_msgs.insert({fleet_name, nullptr});
    if (msg_it.second)
    {
      LaneRequest request;
      request.fleet_name = std::move(fleet_name);
      request.open_lanes.push_back(std::move(lane_id));
      msg_it.first->second = std::make_unique<LaneRequest>(
        std::move(request)
      );
    }
    else
    {
      // Msg was created before. We simply append the new lane id
      msg_it.first->second->open_lanes.push_back(std::move(lane_id));
    }
  }
  else if (old_state == LaneState::Normal && new_state == LaneState::SpeedLimited)
  {
    // construct Speed Limit msg
  }
  else if (old_state == LaneState::SpeedLimited && new_state == LaneState::Normal)
  {
    // construct Speed Limit msg
  }
  else if (old_state == LaneState::SpeedLimited && new_state == LaneState::Closed)
  {
    // construct Speed Limit msg
  }
  else if (old_state == LaneState::Closed && new_state == LaneState::SpeedLimited)
  {
    // construct Speed Limit msg
  }

  // update lane state
  auto it = _internal_lane_states.find(lane_key);
  if (it != _internal_lane_states.end())
  {
    it->second = new_state;
  }
}

//==============================================================================
auto LaneBlocker::deserialize_key(
  const std::string& key) const-> std::pair<std::string, std::size_t>
{
  const std::string delimiter = "_";
  // This should not throw any errors if keys are constructed using get_key()
  // TODO(YV): Consider returning an optional instead
  try
  {
    std::string name = key.substr(0, key.find(delimiter));
    std::string id_str =
      key.substr(key.find(delimiter) + 1, key.size() - name.size());
    std::stringstream ss(id_str);
    std::size_t id; ss >> id;
    return std::make_pair(name, id);
  }
  catch (const std::exception& e)
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[LaneBlocker::deserialize_key] Unable to parse key. This is a bug and "
      "should be reported. Detailed error: " + std::string(e.what()));
    // *INDENT-ON*
  }
}

//==============================================================================
void LaneBlocker::purge_obstacles(
  const std::unordered_set<std::string>& obstacle_keys,
  const bool erase_from_buffer)
{
  for (const auto& obs : obstacle_keys)
  {
    auto lanes_it = _obstacle_to_lanes_map.find(obs);
    if (lanes_it != _obstacle_to_lanes_map.end())
    {
      const auto& lanes = lanes_it->second;
      for (const auto& lane_key : lanes)
      {
        auto obs_it = _lane_to_obstacles_map.find(lane_key);
        if (obs_it != _lane_to_obstacles_map.end())
        {
          obs_it->second.erase(obs);
        }
      }
    }
    _obstacle_to_lanes_map.erase(obs);
    if (erase_from_buffer)
      _obstacle_buffer.erase(obs);
  }
}


//==============================================================================
void LaneBlocker::cull()
{
  // Cull obstacles that are past their expiry times.
  // Also decide whether previously closed lanes should be re-opened.
  std::unordered_set<std::string> obstacles_to_cull;
  std::unordered_set<std::string> lanes_with_changes;

  const auto now = this->get_clock()->now();
  for (const auto& [obstacle_key, lanes] : _obstacle_to_lanes_map)
  {
    auto it = _obstacle_buffer.find(obstacle_key);
    if (it == _obstacle_buffer.end())
    {
      // TODO(YV): Purge
      obstacles_to_cull.insert(obstacle_key);
      continue;
    }
    const auto& obstacle = it->second;
    if (now - obstacle.expiry_time > _cull_timer_period)
    {
      obstacles_to_cull.insert(obstacle_key);
      // Then remove this obstacles from lanes map which is used to decide
      // whether to open/close
      for (const auto& lane : lanes)
      {
        _lane_to_obstacles_map[lane].erase(obstacle_key);
        lanes_with_changes.insert(lane);
      }
    }
  }

  // Cull
  purge_obstacles(obstacles_to_cull);

  // Open lanes if needed
  // A map to collate lanes per fleet that need to be opened
  std::unordered_map<std::string, std::unique_ptr<LaneRequest>> lane_req_msgs;
  std::unordered_map<std::string, std::unique_ptr<SpeedLimitRequest>> speed_limit_req_msgs;

  for (const auto& [lane_key, lane_state] : _internal_lane_states)
  {
    if (lane_state == LaneState::Normal)
    {
      // Normal lane states are handled in request_lane_modifications()
      continue;
    }

    const auto& obstacles = _lane_to_obstacles_map.at(lane_key);
    if (obstacles.size() < _lane_closure_threshold && lane_state == LaneState::Closed)
    {
      transition_lane_state(lane_state, LaneState::Normal, lane_key, lane_req_msgs, speed_limit_req_msgs);
    }
  }

  // Publish lane opening
  for (auto& [_, msg] : lane_req_msgs)
  {
    if (msg->open_lanes.empty())
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "None of the lanes for fleet %s need to be opened",
        msg->fleet_name.c_str()
      );
      continue;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Requested %ld lanes to open for fleet %s",
      msg->open_lanes.size(), msg->fleet_name.c_str()
    );
    _lane_closure_pub->publish(std::move(msg));
  }


}

RCLCPP_COMPONENTS_REGISTER_NODE(LaneBlocker)
