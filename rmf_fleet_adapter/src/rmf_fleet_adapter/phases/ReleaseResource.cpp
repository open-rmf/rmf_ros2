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

#include "ReleaseResource.hpp"
#include "RxOperators.hpp"
#include "SupervisorHasSession.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
std::shared_ptr<ReleaseResource::ActivePhase> ReleaseResource::ActivePhase::make(
  agv::RobotContextPtr context,
  std::string door_name,
  std::string request_id)
{
  RCLCPP_INFO(
    context->node()->get_logger(),
    "Releasing  [%s] for [%s]",
    door_name.c_str(),
    context->requester_id().c_str());
  context->_release_door(door_name);
  auto inst = std::shared_ptr<ActivePhase>(new ActivePhase(
        std::move(context),
        std::move(door_name),
        std::move(request_id)
  ));
  inst->_init_obs();
  return inst;
}

//==============================================================================
const rxcpp::observable<LegacyTask::StatusMsg>&
ReleaseResource::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration ReleaseResource::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void ReleaseResource::ActivePhase::emergency_alarm(bool /*on*/)
{
  // TODO: implement
}

//==============================================================================
void ReleaseResource::ActivePhase::cancel()
{
  // Don't actually cancel anything here. We just release the space.
}

//==============================================================================
const std::string& ReleaseResource::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
void ReleaseResource::ActivePhase::_init_obs()
{
 
}

//==============================================================================
void ReleaseResource::ActivePhase::_publish_release_spot()
{
}

//==============================================================================
ReleaseResource::ActivePhase::ActivePhase(
  agv::RobotContextPtr context,
  std::string door_name,
  std::string request_id)
: _context(std::move(context)),
  _door_name(std::move(door_name)),
  _request_id(std::move(request_id))
{
  _description = "Closing [door:" + _door_name + "]";
}

//==============================================================================
ReleaseResource::PendingPhase::PendingPhase(
  agv::RobotContextPtr context)
: _context(std::move(context))
{
  _description = "Releasing Resource";
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> ReleaseResource::PendingPhase::begin()
{
  return ReleaseResource::ActivePhase::make(
    _context,
    _door_name,
    _request_id);
}

//==============================================================================
rmf_traffic::Duration ReleaseResource::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& ReleaseResource::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
