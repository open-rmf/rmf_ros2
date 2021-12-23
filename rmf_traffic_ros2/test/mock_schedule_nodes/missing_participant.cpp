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
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MissingParticipantScheduleNode : public rmf_traffic_ros2::schedule::ScheduleNode
{
public:
  MissingParticipantScheduleNode(const rclcpp::NodeOptions& options)
    : ScheduleNode(0, options)
  {
    timer = create_wall_timer(30s, [this]() -> void
      {
        RCLCPP_WARN(get_logger(), "Deleting participant 0");
        std::unique_lock<std::mutex> lock(database_mutex);
        database->unregister_participant(0);
        broadcast_participants();
      });
  }

  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissingParticipantScheduleNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
