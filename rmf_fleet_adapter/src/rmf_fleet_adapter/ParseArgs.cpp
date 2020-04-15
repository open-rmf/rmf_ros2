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

#include "ParseArgs.hpp"

#include <algorithm>
#include <iostream>

namespace rmf_fleet_adapter {

//==============================================================================
bool get_arg(
  const std::vector<std::string>& args,
  const std::string& key,
  std::string& value,
  const std::string& desc,
  const bool mandatory)
{
  const auto key_arg = std::find(args.begin(), args.end(), key);
  if (key_arg == args.end())
  {
    // TODO(MXG): See if there's a way to use RCLCPP_ERROR here without first
    // constructing a node. If not, we could consider constructing the
    // FleetAdapterNode in two parts.
    if (mandatory)
    {
      std::cerr << "You must specify a " << desc <<" using the " << key
                << " argument!" << std::endl;
    }
    return false;
  }
  else if (key_arg+1 == args.end())
  {
    std::cerr << "The " << key << " argument must be followed by a " << desc
              << "!" << std::endl;
    return false;
  }

  value = *(key_arg+1);
  return true;
}

//==============================================================================
double get_double_arg(
  const std::vector<std::string>& args,
  const std::string& key,
  const std::string& desc,
  const double default_value)
{
  std::string cli_value;
  if (get_arg(args, key, cli_value, desc, false))
    return std::stod(cli_value);

  std::cout << "No " << key << " flag to specify " << desc << ". The default ["
            << default_value << "] will be used." << std::endl;

  return default_value;
}

//==============================================================================
std::chrono::nanoseconds get_time_arg(
  const std::vector<std::string>& args,
  const std::string& key,
  const std::string& desc,
  const double default_value)
{
  const double value = get_double_arg(args, key, desc, default_value);

  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(value));
}

} // namespace rmf_fleet_adapter
