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

#include <rmf_traffic/Route.hpp>

#include <rmf_traffic_msgs/msg/route.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::Route convert(const rmf_traffic_msgs::msg::Route& from);

//==============================================================================
rmf_traffic_msgs::msg::Route convert(const rmf_traffic::Route& from);

//==============================================================================
std::vector<rmf_traffic::Route> convert(
  const std::vector<rmf_traffic_msgs::msg::Route>& from);

//==============================================================================
std::vector<rmf_traffic_msgs::msg::Route> convert(
  const std::vector<rmf_traffic::Route>& from);

} // namespace rmf_traffic_ros2
