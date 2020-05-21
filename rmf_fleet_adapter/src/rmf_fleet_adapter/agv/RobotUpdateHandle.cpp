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

#include "internal_RobotUpdateHandle.hpp"

#include <iostream>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
std::shared_ptr<RobotContext> RobotUpdateHandle::Implementation::get_context()
{
  auto output = context.lock();
  if (output)
    return output;

  if (reported_loss)
    return nullptr;

  std::cerr << "ERROR: [RobotUpdateHandle] Robot named [" << name << "] is no "
            << "longer available" << std::endl;
  reported_loss = true;
  return nullptr;
}

//==============================================================================
void RobotUpdateHandle::add_delay(rmf_traffic::Duration duration)
{
  if (const auto context = _pimpl->get_context())
    context->schedule().delay(context->now(), duration);
}

//==============================================================================
void RobotUpdateHandle::interrupted()
{

}

//==============================================================================
RobotUpdateHandle::RobotUpdateHandle()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
