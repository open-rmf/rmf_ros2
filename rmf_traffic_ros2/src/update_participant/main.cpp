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

#include <rmf_traffic_msgs/srv/register_participant.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include "../rmf_traffic_ros2/schedule/internal_YamlSerialization.hpp"

#include <iostream>

int main(int argc, char* argv[])
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (args.size() < 2)
  {
    std::cerr << "You need to specify a description file!" << std::endl;
    return 1;
  }

  const auto description_file = args[1];

  std::optional<rmf_traffic::schedule::ParticipantDescription> description;
  try
  {
    const auto yaml_description = YAML::LoadFile(description_file);
    description =
      rmf_traffic_ros2::schedule::participant_description(yaml_description);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to parse description file [" << description_file
              << "]: " << e.what() << std::endl;
    return 1;
  }

  using Service = rmf_traffic_msgs::srv::RegisterParticipant;
  using Request = Service::Request;
  const auto request =
    rmf_traffic_msgs::build<Request>()
    .description(rmf_traffic_ros2::convert(description.value()));

  auto node = std::make_shared<rclcpp::Node>(
    "update_participant_" + request.description.name);

  const auto client = node->create_client<Service>(
    rmf_traffic_ros2::RegisterParticipantSrvName);

  using namespace std::chrono_literals;

  const auto start = std::chrono::steady_clock::now();
  while (!client->service_is_ready())
  {
    if (std::chrono::steady_clock::now() - start > 10s)
    {
      std::cerr << "Unable to find the schedule node :(" << std::endl;
      return 1;
    }

    rclcpp::spin_some(node);
  }

  auto future_response = client->async_send_request(
    std::make_shared<Request>(request));

  rclcpp::spin_until_future_complete(node, future_response, 10s);

  if (future_response.wait_for(0s) != std::future_status::ready)
  {
    std::cerr << "Unable to deliver request to the schedule node :("
              << std::endl;
    return 1;
  }

  std::cout << "Description updated!" << std::endl;
}
