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

#include <chrono>

#include <rmf_traffic_ros2/schedule/internal_Node.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rclcpp/rclcpp.hpp>

#include "../src/rmf_traffic_ros2/schedule/internal_ParticipantRegistry.hpp"

using namespace std::chrono_literals;

class ChangedParticipantScheduleNode : public rmf_traffic_ros2::schedule::ScheduleNode
{
public:
  ChangedParticipantScheduleNode(const rclcpp::NodeOptions& options)
    : ScheduleNode(0, options)
  {
    modify_lists_timer = create_wall_timer(
      11s, [this]() { modify_participants_list(); });

    modify_description_timer = create_wall_timer(
      15s, [this]() { modify_participant_description(); });

    modify_both_timer = create_wall_timer(
      19s, [this]() { modify_both(); });
  }

  void modify_participants_list()
  {
    const auto mocked =
      rmf_traffic_ros2::schedule::mock::mockup_modify_last_participant_id(
        *participant_registry);

    if (!mocked)
      return;

    RCLCPP_WARN(get_logger(), "Modifying and broadcasting participants list");
    broadcast_participants();
  }

  void modify_participant_description()
  {
    const auto mocked =
      rmf_traffic_ros2::schedule::mock
      ::mockup_modify_last_participant_description(*participant_registry);

    if (!mocked)
      return;

    RCLCPP_WARN(
      get_logger(),
      "Modifying a description and broadcasting participants list");
    broadcast_participants();
  }

  void modify_both()
  {
    const auto mocked =
      rmf_traffic_ros2::schedule::mock::mockup_modify_last_participant_id(
        *participant_registry);

    if (!mocked)
      return;

    rmf_traffic_ros2::schedule::mock
    ::mockup_modify_last_participant_description(*participant_registry);

    RCLCPP_WARN(
      get_logger(),
      "Modifying a description and also the list, then broadcasting it");
    broadcast_participants();
  }

  rclcpp::TimerBase::SharedPtr modify_lists_timer;
  rclcpp::TimerBase::SharedPtr modify_description_timer;
  rclcpp::TimerBase::SharedPtr modify_both_timer;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChangedParticipantScheduleNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
}
