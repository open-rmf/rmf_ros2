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

#include <rmf_traffic_ros2/agv/Graph.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp_components/register_node_macro.hpp>

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
    "obstacle_lane_threshold", 0.5);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter obstacle_lane_threshold to %f", _obstacle_lane_threshold
  );

  _tf2_lookup_duration = this->declare_parameter(
    "tf2_lookup_duration", 0.5);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter tf2_lookup_duration to %f", _tf2_lookup_duration
  );

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
    RCLCPP_INFO(
      this->get_logger(),
      "Pose of obstacle id %d in RMF %s frame is [%f, %f, %f]",
      obstacle.id, _rmf_frame.c_str(),
      after_pose.pose.position.x,
      after_pose.pose.position.y, after_pose.pose.position.z
    );
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LaneBlocker)
