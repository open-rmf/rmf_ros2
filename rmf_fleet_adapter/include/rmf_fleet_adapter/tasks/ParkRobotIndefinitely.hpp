/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef RMF_FLEET_ADAPTER__TASKS__PARKROBOT_HPP
#define RMF_FLEET_ADAPTER__TASKS__PARKROBOT_HPP

#include <rmf_task/RequestFactory.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
/// Use this task factory to make finisher tasks (idle tasks) that will move the
/// robot to a parking spot.
class ParkRobotIndefinitely : public rmf_task::RequestFactory
{
public:
  /// Constructor
  ///
  /// \param[in] requester
  ///   The identifier of the entity that owns this RequestFactory, that will be
  ///   the designated requester of each new request.
  ///
  /// \param[in] time_now_cb
  ///   Callback function that returns the current time.
  ///
  /// \param[in] parking_waypoint
  ///   The graph index of the waypoint assigned to this AGV for parking.
  ///   If nullopt, the AGV will return to its charging_waypoint and remain idle
  ///   there. It will not wait for its battery to charge up before undertaking
  ///   new tasks.
  ParkRobotIndefinitely(
    const std::string& requester,
    std::function<rmf_traffic::Time()> time_now_cb,
    std::optional<std::size_t> parking_waypoint = std::nullopt);

  rmf_task::ConstRequestPtr make_request(
    const rmf_task::State& state) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace tasks
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__PARKROBOT_HPP
