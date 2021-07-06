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

#include <rmf_traffic_ros2/blockade/Node.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic/blockade/Moderator.hpp>

#include <rmf_traffic_msgs/msg/blockade_cancel.hpp>
#include <rmf_traffic_msgs/msg/blockade_heartbeat.hpp>
#include <rmf_traffic_msgs/msg/blockade_reached.hpp>
#include <rmf_traffic_msgs/msg/blockade_ready.hpp>
#include <rmf_traffic_msgs/msg/blockade_release.hpp>
#include <rmf_traffic_msgs/msg/blockade_set.hpp>
#include <rmf_traffic_msgs/msg/blockade_status.hpp>

namespace rmf_traffic_ros2 {
namespace blockade {

//==============================================================================
class BlockadeNode : public rclcpp::Node
{
public:

  using Checkpoint = rmf_traffic::blockade::Writer::Checkpoint;
  using Reservation = rmf_traffic::blockade::Writer::Reservation;

  BlockadeNode(
    const std::string& node_name,
    const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options),
    moderator(std::make_shared<rmf_traffic::blockade::Moderator>())
  {
    blockade_set_sub =
      create_subscription<SetMsg>(
      BlockadeSetTopicName,
      rclcpp::SystemDefaultsQoS().best_effort(),
      [=](const SetMsg::UniquePtr msg)
      {
        this->blockade_set(*msg);
      });

    blockade_ready_sub =
      create_subscription<ReadyMsg>(
      BlockadeReadyTopicName,
      rclcpp::SystemDefaultsQoS().best_effort(),
      [=](const ReadyMsg::UniquePtr msg)
      {
        this->blockade_ready(*msg);
      });

    blockade_reached_sub =
      create_subscription<ReachedMsg>(
      BlockadeReachedTopicName,
      rclcpp::SystemDefaultsQoS().best_effort(),
      [=](const ReachedMsg::UniquePtr msg)
      {
        this->blockade_reached(*msg);
      });

    blockade_release_sub =
      create_subscription<ReleaseMsg>(
      BlockadeReleaseTopicName,
      rclcpp::SystemDefaultsQoS().best_effort(),
      [=](const ReleaseMsg::UniquePtr msg)
      {
        this->blockade_release(*msg);
      });

    blockade_cancel_sub =
      create_subscription<CancelMsg>(
      BlockadeCancelTopicName,
      rclcpp::SystemDefaultsQoS().best_effort(),
      [=](const CancelMsg::UniquePtr msg)
      {
        this->blockade_cancel(*msg);
      });

    heartbeat_pub = create_publisher<HeartbeatMsg>(
      BlockadeHeartbeatTopicName,
      rclcpp::SystemDefaultsQoS().reliable());

    heartbeat_timer = create_wall_timer(
      std::chrono::seconds(1),
      [this]()
      {
        this->publish_status();
      });
  }

  using SetMsg = rmf_traffic_msgs::msg::BlockadeSet;
  rclcpp::Subscription<SetMsg>::SharedPtr blockade_set_sub;
  void blockade_set(const SetMsg& set)
  {
    std::vector<Checkpoint> path;
    for (const auto& c : set.path)
    {
      path.push_back(
        Checkpoint{
          Eigen::Vector2d{c.position[0], c.position[1]},
          c.map_name,
          c.can_hold
        });
    }

    try
    {
      moderator->set(
        set.participant, set.reservation,
        Reservation{std::move(path), set.radius});
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception due to [set] update: %s", e.what());
    }

    check_for_updates();
  }

  using ReadyMsg = rmf_traffic_msgs::msg::BlockadeReady;
  rclcpp::Subscription<ReadyMsg>::SharedPtr blockade_ready_sub;
  void blockade_ready(const ReadyMsg& ready)
  {
    try
    {
      moderator->ready(ready.participant, ready.reservation, ready.checkpoint);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception due to [ready] update: %s", e.what());
    }

    check_for_updates();
  }

  using ReleaseMsg = rmf_traffic_msgs::msg::BlockadeRelease;
  rclcpp::Subscription<ReleaseMsg>::SharedPtr blockade_release_sub;
  void blockade_release(const ReleaseMsg& release)
  {
    try
    {
      moderator->release(
        release.participant, release.reservation, release.checkpoint);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception due to [release] update: %s", e.what());
    }

    check_for_updates();
  }

  using ReachedMsg = rmf_traffic_msgs::msg::BlockadeReached;
  rclcpp::Subscription<ReachedMsg>::SharedPtr blockade_reached_sub;
  void blockade_reached(const ReachedMsg& reached)
  {
    try
    {
      moderator->reached(
        reached.participant, reached.reservation, reached.checkpoint);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception due to [reached] update: %s", e.what());
    }

    check_for_updates();
  }

  using CancelMsg = rmf_traffic_msgs::msg::BlockadeCancel;
  rclcpp::Subscription<CancelMsg>::SharedPtr blockade_cancel_sub;
  void blockade_cancel(const CancelMsg& cancel)
  {
    try
    {
      if (cancel.all_reservations)
        moderator->cancel(cancel.participant);
      else
        moderator->cancel(cancel.participant, cancel.reservation);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        get_logger(), "Exception due to [cancel] update: %s", e.what());
    }

    check_for_updates();
  }

  void check_for_updates()
  {
    const std::size_t current_version = moderator->assignments().version();
    if (current_version == last_assignment_version)
      return;

    last_assignment_version = current_version;
    publish_status();
  }

  using HeartbeatMsg = rmf_traffic_msgs::msg::BlockadeHeartbeat;
  rclcpp::Publisher<HeartbeatMsg>::SharedPtr heartbeat_pub;
  using StatusMsg = rmf_traffic_msgs::msg::BlockadeStatus;
  void publish_status()
  {
    const auto& ranges = moderator->assignments().ranges();

    std::vector<StatusMsg> statuses;
    for (const auto& s : moderator->statuses())
    {
      const std::size_t participant = s.first;
      const auto& range = ranges.at(participant);
      const auto& status = s.second;

      statuses.emplace_back(
        rmf_traffic_msgs::build<StatusMsg>()
        .participant(participant)
        .reservation(status.reservation)
        .any_ready(status.last_ready.has_value())
        .last_ready(status.last_ready.value_or(0))
        .last_reached(status.last_reached)
        .assignment_begin(range.begin)
        .assignment_end(range.end));
    }

    auto msg = rmf_traffic_msgs::build<HeartbeatMsg>()
      .statuses(std::move(statuses))
      .has_gridlock(moderator->has_gridlock());

    heartbeat_pub->publish(msg);
  }

  std::shared_ptr<rmf_traffic::blockade::Moderator> moderator;
  std::size_t last_assignment_version = 0;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
};

//==============================================================================
std::shared_ptr<rclcpp::Node> make_node(const rclcpp::NodeOptions& options)
{
  return make_node("rmf_traffic_blockade_node", options);
}

//==============================================================================
std::shared_ptr<rclcpp::Node> make_node(
  const std::string& node_name,
  const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<BlockadeNode>(node_name, options);

  node->moderator->info_logger(
    [w = node->weak_from_this()](std::string msg)
    {
      if (const auto n = w.lock())
      {
        RCLCPP_INFO(n->get_logger(), msg.c_str());
      }
    });

  node->moderator->debug_logger(
    [w = node->weak_from_this()](std::string msg)
    {
      if (const auto n = w.lock())
      {
        RCLCPP_DEBUG(n->get_logger(), msg.c_str());
      }
    });

  node->moderator->minimum_conflict_angle(15.0*M_PI/180.0);

  return node;
}

} // namespace blockade
} // namespace rmf_traffic_ros2
