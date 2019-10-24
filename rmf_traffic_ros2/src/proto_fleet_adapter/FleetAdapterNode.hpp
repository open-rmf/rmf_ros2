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


#ifndef SRC__RMF_TRAFFIC_ROS2__PROTO_FLEET_ADAPTER__FLEETADAPTERNODE_HPP
#define SRC__RMF_TRAFFIC_ROS2__PROTO_FLEET_ADAPTER__FLEETADAPTERNODE_HPP

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>

#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rclcpp/node.hpp>

namespace proto_fleet_adapter {

//==============================================================================
class FleetAdapterNode : public rclcpp::Node
{
public:

  FleetAdapterNode(
      std::string graph_file,
      rmf_traffic::agv::VehicleTraits vehicle_traits);

};

} // namespace proto_fleet_adapter

#endif // SRC__RMF_TRAFFIC_ROS2__PROTO_FLEET_ADAPTER__FLEETADAPTERNODE_HPP
