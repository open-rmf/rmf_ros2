/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>

#include <rmf_traffic_msgs/srv/mirror_update.hpp>
#include <rmf_traffic_msgs/srv/register_query.hpp>

#include <rclcpp/logging.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

using MirrorUpdateClient = rclcpp::Client<rmf_traffic_msgs::srv::MirrorUpdate>;

//==============================================================================
class MirrorManager::Implementation
{
public:

  Options options;
  uint64_t query_id;
  MirrorUpdateClient::SharedPtr mirror_update_client;

  Implementation(
      Options _options,
      uint64_t _query_id,
      MirrorUpdateClient::SharedPtr _mirror_update_client)
    : options(std::move(_options)),
      query_id(_query_id),
      mirror_update_client(std::move(_mirror_update_client))
  {
  }

  ~Implementation()
  {

  }

};

//==============================================================================
MirrorManager::MirrorManager()
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<MirrorManager> make_mirror(
    rclcpp::Node& node,
    const rmf_traffic::schedule::Query::Spacetime& spacetime,
    MirrorManager::Options options)
{
  const auto registration_client = node.create_client<
      rmf_traffic_msgs::srv::RegisterQuery>(RegisterQueryService);

  if(!registration_client->wait_for_service(options.get_discovery_wait_time()))
  {
    RCLCPP_ERROR(
          node.get_logger(),
          "Unable to contact rmf_traffic schedule database to "
          "register a mirror query");
    return nullptr;
  }

  rmf_traffic_msgs::srv::RegisterQuery::Request register_query_request;
  register_query_request.query = convert(spacetime);
  registration_client->async_send_request(
        )
}

} // namespace schedule
} // namespace rmf_traffic_ros2
