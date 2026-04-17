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

#include "Utils.hpp"

#include <random>
#include <sstream>

namespace rmf_fleet_adapter {
namespace phases {

bool is_newer(const builtin_interfaces::msg::Time& a,
  const builtin_interfaces::msg::Time& b)
{
  return a.sec > b.sec || (a.sec == b.sec && a.nanosec >= b.nanosec);
}

namespace {
// Produces a length*2 character zero-padded hex string from random bytes.
std::string generate_random_hex_string(const std::size_t length)
{
  static thread_local std::mt19937 gen{std::random_device{}()};
  static thread_local std::uniform_int_distribution<int> dis(0, 255);
  std::stringstream ss;
  for (std::size_t i = 0; i < length; ++i)
  {
    const auto random_char = dis(gen);
    std::stringstream hexstream;
    hexstream << std::hex << random_char;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}
} // anonymous namespace

std::string generate_zone_request_id(
  const std::string& fleet,
  const std::string& robot,
  const std::string& zone)
{
  return fleet + "_" + robot + "_" + zone + "_"
    + generate_random_hex_string(5);
}

} // namespace phases
} // namespace rmf_fleet_adapter

