// Copyright (C) 2021 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include <rmf_traffic_ros2/schedule/internal_Node.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class ChangedParticipantScheduleNode : public rmf_traffic_ros2::schedule::ScheduleNode
{
public:
  ChangedParticipantScheduleNode(const rclcpp::NodeOptions& options)
    : ScheduleNode(0, options)
  {
    timer = create_wall_timer(30s, [this]() -> void
      {
        broadcast_modified_participants_list();
      });
  }

  void broadcast_modified_participants_list()
  {
    RCLCPP_WARN(get_logger(), "Broadcasting modified participants list");
    ++current_participants_version;
    ParticipantsInfo msg;

    bool first_participant = true;
    for (const auto& id: database->participant_ids())
    {
      SingleParticipantInfo participant;
      if (first_participant)
      {
        first_participant = false;
        participant.id = id + 100;
      }
      else
      {
        participant.id = id;
      }
      participant.description = rmf_traffic_ros2::convert(
        *database->get_participant(id));
      msg.participants.push_back(participant);
    }
    participants_info_pub->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChangedParticipantScheduleNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
