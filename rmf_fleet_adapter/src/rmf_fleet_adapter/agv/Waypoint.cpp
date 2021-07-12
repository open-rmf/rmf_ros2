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

#include <rmf_fleet_adapter/agv/Waypoint.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Waypoint::Implementation
{
public:

  std::string map_name;
  Eigen::Vector3d position;
  rmf_traffic::Duration mandatory_delay;
  bool yield;

};

//==============================================================================
Waypoint::Waypoint(
  std::string map_name,
  Eigen::Vector3d position,
  rmf_traffic::Duration mandatory_delay,
  bool yield)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(map_name),
        std::move(position),
        mandatory_delay,
        yield
      }))
{
  // Do nothing
}

//==============================================================================
const std::string& Waypoint::map_name() const
{
  return _pimpl->map_name;
}

//==============================================================================
Waypoint& Waypoint::map_name(std::string new_map_name)
{
  _pimpl->map_name = std::move(new_map_name);
  return *this;
}

//==============================================================================
const Eigen::Vector3d& Waypoint::position() const
{
  return _pimpl->position;
}

//==============================================================================
Waypoint& Waypoint::position(Eigen::Vector3d new_position)
{
  _pimpl->position = new_position;
  return *this;
}

//==============================================================================
rmf_traffic::Duration Waypoint::mandatory_delay() const
{
  return _pimpl->mandatory_delay;
}

//==============================================================================
Waypoint& Waypoint::mandatory_delay(rmf_traffic::Duration duration)
{
  _pimpl->mandatory_delay = duration;
  return *this;
}

//==============================================================================
bool Waypoint::yield() const
{
  return _pimpl->yield;
}

//==============================================================================
Waypoint& Waypoint::yield(bool on)
{
  _pimpl->yield = on;
  return *this;
}

} // namespace agv
} // namespace rmf_fleet_adapter
