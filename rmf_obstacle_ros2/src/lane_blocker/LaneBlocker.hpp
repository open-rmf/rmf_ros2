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

#ifndef SRC__LANE_BLOCKER__LANEBLOCKER_HPP
#define SRC__LANE_BLOCKER__LANEBLOCKER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic/agv/Graph.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <rmf_building_map_msgs/msg/graph.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/speed_limit_request.hpp>
#include <rmf_fleet_msgs/msg/speed_limited_lane.hpp>
#include <rmf_fleet_msgs/msg/lane_states.hpp>

#include <rmf_obstacle_msgs/msg/bounding_box3_d.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include <unordered_map>
#include <unordered_set>
#include <functional>

//==============================================================================
/// Modify states of lanes for fleet adapters based on density of obstacles
class LaneBlocker : public rclcpp::Node
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;
  using Obstacle = rmf_obstacle_msgs::msg::Obstacle;
  using NavGraph = rmf_building_map_msgs::msg::Graph;
  using TrafficGraph = rmf_traffic::agv::Graph;
  using LaneRequest = rmf_fleet_msgs::msg::LaneRequest;
  using SpeedLimitRequest = rmf_fleet_msgs::msg::SpeedLimitRequest;
  using SpeedLimitedLane = rmf_fleet_msgs::msg::SpeedLimitedLane;
  using LaneStates = rmf_fleet_msgs::msg::LaneStates;
  using BoundingBox = rmf_obstacle_msgs::msg::BoundingBox3D;
  using Header = std_msgs::msg::Header;

  /// Constructor
  LaneBlocker(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void obstacle_cb(const message_filters::MessageEvent<Obstacles const>& evt);
  void process();
  void cull();

  // Internal data struct
  struct ObstacleData
  {
    rclcpp::Time expiry_time;
    int id;
    std::string source;
    BoundingBox transformed_bbox;

    ObstacleData() {}

    ObstacleData(
      rclcpp::Time expiry_time_,
      int id_,
      const std::string& source_,
      BoundingBox transformed_bbox_)
    : expiry_time(expiry_time_),
      id(id_),
      source(std::move(source_)),
      transformed_bbox(std::move(transformed_bbox_))
    {}

    // Overload == for hashing
    inline bool operator==(const ObstacleData& other)
    const
    {
      const auto lhs_key = LaneBlocker::get_obstacle_key(source, id);
      const auto rhs_key =
        LaneBlocker::get_obstacle_key(other.source, other.id);
      return lhs_key == rhs_key;
    }
  };
  using ObstacleDataConstSharedPtr = std::shared_ptr<const ObstacleData>;

  static inline std::string get_obstacle_key(
    const std::string& source, const std::size_t id)
  {
    return source + "_" + std::to_string(id);
  }

  static inline std::string get_obstacle_key(const ObstacleData& obstacle)
  {
    return LaneBlocker::get_obstacle_key(
      obstacle.source, obstacle.id);
  }

  static inline std::string get_lane_key(
    const std::string& fleet_name,
    const std::size_t lane_index)
  {
    return fleet_name + "_" + std::to_string(lane_index);
  }

  struct ObstacleHash
  {
    std::size_t operator()(
      const ObstacleData& obstacle) const
    {
      const std::string key = LaneBlocker::get_obstacle_key(obstacle);
      return std::hash<std::string>()(key);
    }
  };

  std::optional<std::pair<std::string, std::size_t>>
  deserialize_key(const std::string& key) const;

  // Modify lanes with changes in number of vicinity obstacles
  void request_lane_modifications(
    const std::unordered_set<std::string>& changes);

  enum class LaneState : uint8_t
  {
    Normal = 0,
    Closed,
    SpeedLimited,
    SpeedUnlimited
  };

  std::unordered_map<
    std::string,
    LaneState> _internal_lane_states = {};

  void transition_lane_state(
    const LaneState& old_state,
    const LaneState& new_state,
    const std::string& lane_key);

  void add_lane_open_close_req(
    const std::string& lane_key,
    const LaneState& desired_state);

  void add_speed_limit_req(
    const std::string& lane_key,
    const LaneState& desired_state);

  void publish_lane_req_msgs();

  void publish_speed_limit_req_msgs();

  void purge_obstacles(
    const std::unordered_set<std::string>& obstacle_keys,
    const bool erase_from_buffer = true);

  // Store obstacle after transformation into RMF frame.
  // Generate key using get_obstacle_key()
  // We cache them based on source + id so that we keep only the latest
  // version of that obstacle.
  std::unordered_map<std::string, ObstacleData>
  _obstacle_buffer = {};

  // TODO(YV): Based on the current implementation, we should be able to
  // cache obstacle_key directly
  // Map an obstacle key to the lanes keys in its vicinity
  std::unordered_map<
    std::string,
    std::unordered_set<std::string>> _obstacle_to_lanes_map = {};

  // Map lane to a set of obstacles in its vicinity. This is only used to
  // check the number of obstacles in the vicinity of a lane. The obstacles
  // are represented as their obstacle keys.
  std::unordered_map<
    std::string,
    std::unordered_set<std::string>>
  _lane_to_obstacles_map = {};

  rclcpp::Subscription<NavGraph>::SharedPtr _graph_sub;
  rclcpp::Subscription<LaneStates>::SharedPtr _lane_states_sub;
  rclcpp::Publisher<LaneRequest>::SharedPtr _lane_closure_pub;
  rclcpp::Publisher<SpeedLimitRequest>::SharedPtr _speed_limit_pub;
  double _tf2_lookup_duration;

  std::string _rmf_frame;
  std::shared_ptr<message_filters::Subscriber<Obstacles>> _obstacle_sub;
  std::shared_ptr<tf2_ros::MessageFilter<Obstacles>> _tf2_filter_obstacles;
  std::unique_ptr<tf2_ros::Buffer> _tf2_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _transform_listener;

  std::unordered_map<std::string, TrafficGraph> _traffic_graphs;
  std::unordered_map<std::string, LaneStates::ConstSharedPtr> _lane_states;
  double _lane_width;
  double _obstacle_lane_threshold;
  std::chrono::nanoseconds _max_search_duration;
  std::chrono::nanoseconds _cull_timer_period;
  bool _continuous_checker;
  std::size_t _lane_closure_threshold;
  std::size_t _speed_limit_threshold;
  double _speed_limit;

  rclcpp::TimerBase::SharedPtr _process_timer;
  rclcpp::TimerBase::SharedPtr _cull_timer;

  std::mutex _mutex_msgs;
  // A map to collate lanes per fleet that need to be opened or closed
  std::unordered_map<std::string, std::unique_ptr<LaneRequest>> _lane_req_msgs;
  // A map to collate lanes per fleet that need to be speed limited or unlimited
  std::unordered_map<std::string,
    std::unique_ptr<SpeedLimitRequest>> _speed_limit_req_msgs;
};

#endif // SRC__LANE_BLOCKER__LANEBLOCKER_HPP
