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

#ifndef SRC__RMF_FLEET_ADAPTER__PARSEARGS_HPP
#define SRC__RMF_FLEET_ADAPTER__PARSEARGS_HPP

#include <chrono>
#include <string>
#include <vector>

namespace rmf_fleet_adapter {

// TODO(MXG): Use something like boost::program_options instead of a custom CLI
// interpreter
bool get_arg(
  const std::vector<std::string>& args,
  const std::string& key,
  std::string& value,
  const std::string& desc,
  const bool mandatory = true);

double get_double_arg(
  const std::vector<std::string>& args,
  const std::string& key,
  const std::string& desc,
  const double default_value);

std::chrono::nanoseconds get_time_arg(
  const std::vector<std::string>& args,
  const std::string& key,
  const std::string& desc,
  const double default_value);

} // namespace rmf_fleet_adapter

#endif // PARSEARGS_HPP
