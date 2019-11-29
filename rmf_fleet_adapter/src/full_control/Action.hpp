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

#ifndef SRC__FULL_CONTROL__ACTION_HPP
#define SRC__FULL_CONTROL__ACTION_HPP

#include <rclcpp/time.hpp>

#include <string>

#include <rmf_utils/optional.hpp>

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
class Action
{
public:

  virtual void execute() = 0;

  virtual void interrupt() = 0;

  virtual void resume() = 0;

  virtual void resolve() = 0;

  struct Status
  {
    std::string text;
    rmf_utils::optional<rclcpp::Time> finish_estimate = rmf_utils::nullopt;
  };

  virtual Status get_status() const = 0;

  virtual ~Action() = default;
};

} // namespace full_control
} // namespace rmf_fleet_adapter

#endif // SRC__FULL_CONTROL__ACTION_HPP
