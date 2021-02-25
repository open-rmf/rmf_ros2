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

#include <rmf_traffic_ros2/Profile.hpp>
#include <rmf_traffic_ros2/geometry/ConvexShape.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

#include <rmf_traffic/schedule/Database.hpp>

#include <yaml-cpp/yaml.h>

#include <string>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
uint8_t shape_type(YAML::Node node);

//==============================================================================
rmf_traffic_msgs::msg::ConvexShape convex_shape(YAML::Node node);

//==============================================================================
rmf_traffic_msgs::msg::ConvexShapeContext shape_context(YAML::Node node);

//==============================================================================
rmf_traffic::Profile profile(YAML::Node node);

//==============================================================================
ParticipantDescription participant_description(YAML::Node node);

//==============================================================================
AtomicOperation atomic_operation(YAML::Node node);

//==============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShapeContext context);

//==============================================================================
std::string serialize_shape_type(uint8_t shape_type);

//==============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShape shape);

//==============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShapeContext shape);

//==============================================================================
YAML::Node serialize(rmf_traffic::Profile profile);

//==============================================================================
std::string serialize_responsiveness(ParticipantDescription::Rx resp);

//==============================================================================
YAML::Node serialize(ParticipantDescription participant);

//==============================================================================
YAML::Node serialize(AtomicOperation atomOp);

} // namespace schedule
} // namespace rmf_traffic_ros2
