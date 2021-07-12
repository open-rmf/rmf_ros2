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

#ifndef SRC__FULL_CONTROL__SCHEDULEMANAGER_HPP
#define SRC__FULL_CONTROL__SCHEDULEMANAGER_HPP

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/schedule/Version.hpp>

#include "Listener.hpp"

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rclcpp/node.hpp>

#include <unordered_set>

namespace rmf_fleet_adapter {

//==============================================================================
class ScheduleManager
{
public:

  ScheduleManager(
    rclcpp::Node& node,
    rmf_traffic::schedule::Participant participant,
    rmf_traffic_ros2::schedule::Negotiation* negotiation);

  using TrajectorySet = std::vector<rmf_traffic::Trajectory>;

  void push_routes(const std::vector<rmf_traffic::Route>& routes);

  void push_delay(const rmf_traffic::Duration duration);

  void set_negotiator(
    std::function<void(
      const rmf_traffic::schedule::Negotiation::Table::ViewerPtr&,
      const rmf_traffic::schedule::Negotiator::ResponderPtr&)>
    negotiation_callback);

  rmf_traffic::schedule::Participant& participant();

  rmf_traffic::schedule::ParticipantId participant_id() const;

  const rmf_traffic::schedule::ParticipantDescription& description() const;

private:

  class Negotiator : public rmf_traffic::schedule::Negotiator
  {
  public:

    void respond(
      const rmf_traffic::schedule::Negotiation::Table::ViewerPtr& table,
      const ResponderPtr& responder) final;

    std::function<void(
        rmf_traffic::schedule::Negotiation::Table::ViewerPtr,
        const ResponderPtr&)> callback;
  };

  rclcpp::Node* _node;
  rmf_traffic::schedule::Participant _participant;
  Negotiator* _negotiator;
  std::shared_ptr<void> _negotiator_handle;
};

//==============================================================================
std::future<ScheduleManager> make_schedule_manager(
  rclcpp::Node& node,
  rmf_traffic_ros2::schedule::Writer& writer,
  rmf_traffic_ros2::schedule::Negotiation* negotiation,
  rmf_traffic::schedule::ParticipantDescription description);

//==============================================================================
void async_make_schedule_manager(
  rclcpp::Node& node,
  rmf_traffic_ros2::schedule::Writer& writer,
  rmf_traffic_ros2::schedule::Negotiation* negotiation,
  rmf_traffic::schedule::ParticipantDescription description,
  std::function<void(ScheduleManager manager)> ready_callback,
  std::mutex& ready_mutex);

} // namespace rmf_fleet_adapter


#endif // SRC__FULL_CONTROL__SCHEDULEMANAGER_HPP
