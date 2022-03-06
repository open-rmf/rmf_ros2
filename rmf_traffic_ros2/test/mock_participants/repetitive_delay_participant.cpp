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

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_traffic/geometry/Circle.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("repetitive_delay_participant");
  const auto writer = rmf_traffic_ros2::schedule::Writer::make(node);

  auto future_participant = writer->make_participant(
    rmf_traffic::schedule::ParticipantDescription(
      "repetitive_delay_participant",
      "mock",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      rmf_traffic::Profile(
        rmf_traffic::geometry::make_final_convex(
          rmf_traffic::geometry::Circle(1.0)))));

  using namespace std::chrono_literals;
  std::optional<rmf_traffic::schedule::Participant> participant;
  const auto timer = node->create_wall_timer(
    1s, [&future_participant, &participant, &node]()
    {
      if (!participant.has_value())
      {
        if (future_participant.wait_for(0s) == std::future_status::timeout)
          return;

        const auto now = rmf_traffic_ros2::convert(node->get_clock()->now());
        participant = future_participant.get();
        rmf_traffic::Trajectory traj;
        traj.insert(
          now,
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero());

        traj.insert(
          now + 30s,
          Eigen::Vector3d::UnitX(),
          Eigen::Vector3d::Zero());

        participant->set(
          participant->plan_id_assigner()->assign(), {{"test_map", traj}});

        return;
      }

      std::cout << "Applying delay" << std::endl;
      participant->delay(1s);
    });

  rclcpp::spin(node);
}
