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

#include <rmf_traffic_ros2/geometry/Box.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic_msgs::msg::Box convert(const rmf_traffic::geometry::Box& box)
{
  rmf_traffic_msgs::msg::Box output;
  output.dimensions = {box.get_x_length(), box.get_y_length()};
  return output;
}

//==============================================================================
rmf_traffic::geometry::Box convert(const rmf_traffic_msgs::msg::Box& box)
{
  return rmf_traffic::geometry::Box(box.dimensions[0], box.dimensions[1]);
}

} // namespace rmf_traffic_ros2
