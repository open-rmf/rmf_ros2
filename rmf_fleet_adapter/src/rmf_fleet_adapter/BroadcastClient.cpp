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

#include "BroadcastClient.hpp"


namespace rmf_fleet_adapter {
//==============================================================================
std::shared_ptr<BroadcastClient> BroadcastClient::make(
  const std::string& uri,
  std::shared_ptr<agv::FleetUpdateHandle> fleet_handle)
{
  std::shared_ptr<BroadcastClient> client(new BroadcastClient());
  client->_fleet_handle = std::move(fleet_handle);
  client->_shutdown = false;

}

//==============================================================================
BroadcastClient::BroadcastClient()
{
  // Do nothing
}

//==============================================================================
BroadcastClient::~BroadcastClient()
{
  if (_thread.joinable)
  {
    _thread.join();
  }
}

} // namespace rmf_fleet_adapter
