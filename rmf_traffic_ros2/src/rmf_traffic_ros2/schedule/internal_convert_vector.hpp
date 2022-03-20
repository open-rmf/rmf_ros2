/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef SRC__RMF_TRAFFIC_ROS2__SCHEDULE__INTERNAL_CONVERT_VECTOR_HPP
#define SRC__RMF_TRAFFIC_ROS2__SCHEDULE__INTERNAL_CONVERT_VECTOR_HPP

#include <vector>

namespace rmf_traffic_ros2 {

//==============================================================================
template<typename T_out, typename T_in>
void convert_vector(
  std::vector<T_out>& output,
  const std::vector<T_in>& input)
{
  output.reserve(input.size());
  for (const auto& i : input)
    output.emplace_back(convert(i));
}

//==============================================================================
template<typename T_out, typename T_in>
std::vector<T_out> convert_vector(
  const std::vector<T_in>& input)
{
  std::vector<T_out> output;
  convert_vector(output, input);
  return output;
}

} // namespace rmf_traffic_ros2

#endif // INTERNAL_CONVERT_VECTOR_HPP
