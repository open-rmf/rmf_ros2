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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP

#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>

#include "Node.hpp"
#include "RobotContext.hpp"

#include <rmf_traffic/schedule/Snapshot.hpp>

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class FleetUpdateHandle::Implementation
{
public:

  std::string name;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  std::shared_ptr<Node> node;
  rxcpp::schedulers::worker worker;
  std::shared_ptr<rmf_traffic_ros2::schedule::Writer> writer;
  std::shared_ptr<rmf_traffic::schedule::Snappable> snappable;
  std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation;

  std::vector<RobotContextPtr> robots = {};

  template<typename... Args>
  static std::shared_ptr<FleetUpdateHandle> make(Args&&... args)
  {
    FleetUpdateHandle handle;
    handle._pimpl = rmf_utils::make_unique_impl<Implementation>(
          Implementation{std::forward<Args>(args)...});
    return std::make_shared<FleetUpdateHandle>(std::move(handle));
  }

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
