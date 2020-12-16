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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_TRAFFICLIGHT_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_TRAFFICLIGHT_HPP

#include <rmf_fleet_adapter/agv/TrafficLight.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_traffic/schedule/Negotiator.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic_ros2/blockade/Writer.hpp>

#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rxcpp/rx.hpp>

#include "Node.hpp"

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class TrafficLight::UpdateHandle::Implementation
{
public:

  class Data;
  class Negotiator;

  std::size_t received_version = 0;

  std::shared_ptr<Data> data;

  std::shared_ptr<void> negotiation_license;

  void new_range(
      rmf_traffic::blockade::ReservationId reservation_id,
      const rmf_traffic::blockade::ReservedRange& new_range);

  Implementation(
      std::shared_ptr<CommandHandle> command_,
      rmf_traffic::schedule::Participant itinerary_,
      std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer,
      rmf_traffic::agv::VehicleTraits traits_,
      std::shared_ptr<rmf_traffic::schedule::Snappable> schedule_,
      rxcpp::schedulers::worker worker_,
      std::shared_ptr<Node> node_);

  static std::shared_ptr<UpdateHandle> make(
      std::shared_ptr<CommandHandle> command,
      rmf_traffic::schedule::Participant itinerary,
      std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer,
      rmf_traffic::agv::VehicleTraits traits,
      std::shared_ptr<rmf_traffic::schedule::Snappable> schedule,
      rxcpp::schedulers::worker worker,
      std::shared_ptr<Node> node,
      rmf_traffic_ros2::schedule::Negotiation* negotiation);

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_TRAFFICLIGHT_HPP
