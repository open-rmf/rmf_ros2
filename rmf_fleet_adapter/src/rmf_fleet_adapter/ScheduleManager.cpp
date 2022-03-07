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

#include "ScheduleManager.hpp"

#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

namespace rmf_fleet_adapter {

//==============================================================================
ScheduleManager::ScheduleManager(
  rclcpp::Node& node,
  rmf_traffic::schedule::Participant participant,
  rmf_traffic_ros2::schedule::Negotiation* negotiation)
: _node(&node),
  _participant(std::move(participant)),
  _negotiator(nullptr)
{
  if (negotiation)
  {
    auto negotiator = std::make_unique<Negotiator>();
    _negotiator = negotiator.get();
    _negotiator_handle = negotiation->register_negotiator(
      _participant.id(), std::move(negotiator));
  }
}

//==============================================================================
void ScheduleManager::push_routes(const std::vector<rmf_traffic::Route>& routes)
{
  // TODO(MXG): Be smarter here. If there are no trajectories then erase the
  // current schedule? Or have the robot stand in place?
  std::vector<rmf_traffic::Route> valid_routes;
  valid_routes.reserve(routes.size());
  for (const auto& r : routes)
  {
    if (r.trajectory().size() < 2)
      continue;

    valid_routes.push_back(r);
  }

  // If there are no valid trajectories to push to the schedule, then erase the
  // current trajectories from the schedule and approve.
  // TODO(MXG): Consider putting some debug output here.
  if (valid_routes.empty())
  {
    _participant.clear();
    return;
  }

  _participant.set(
    _participant.plan_id_assigner()->assign(),
    std::move(valid_routes));
}

//==============================================================================
void ScheduleManager::push_delay(const rmf_traffic::Duration duration)
{
  _participant.delay(duration);
}

//==============================================================================
void ScheduleManager::set_negotiator(
  std::function<void(
    const rmf_traffic::schedule::Negotiation::Table::ViewerPtr&,
    const Negotiator::ResponderPtr&)> negotiation_callback)
{
  if (_negotiator)
    _negotiator->callback = std::move(negotiation_callback);
}

//==============================================================================
rmf_traffic::schedule::Participant& ScheduleManager::participant()
{
  return _participant;
}

//==============================================================================
rmf_traffic::schedule::ParticipantId ScheduleManager::participant_id() const
{
  return _participant.id();
}

//==============================================================================
const rmf_traffic::schedule::ParticipantDescription&
ScheduleManager::description() const
{
  return _participant.description();
}

//==============================================================================
void ScheduleManager::Negotiator::respond(
  const rmf_traffic::schedule::Negotiation::Table::ViewerPtr& table,
  const ResponderPtr& responder)
{
  if (!callback)
    return;

  callback(table, responder);
}

//==============================================================================
std::future<ScheduleManager> make_schedule_manager(
  rclcpp::Node& node,
  rmf_traffic_ros2::schedule::Writer& writer,
  rmf_traffic_ros2::schedule::Negotiation* negotiation,
  rmf_traffic::schedule::ParticipantDescription description,
  std::function<void()> revision_callback)
{
  return std::async(
    std::launch::async,
    [&node,
    &writer,
    negotiation,
    description = std::move(description),
    revision_callback = std::move(revision_callback)]() -> ScheduleManager
    {
      return ScheduleManager(
        node,
        writer.make_participant(std::move(description)).get(),
        negotiation);
    });
}

//==============================================================================
void async_make_schedule_manager(
  rclcpp::Node& node,
  rmf_traffic_ros2::schedule::Writer& writer,
  rmf_traffic_ros2::schedule::Negotiation* negotiation,
  rmf_traffic::schedule::ParticipantDescription description,
  std::function<void(ScheduleManager)> ready_callback,
  std::mutex& ready_mutex)
{
  writer.async_make_participant(
    std::move(description),
    [&node,
    negotiation,
    ready_callback = std::move(ready_callback),
    &ready_mutex](
      rmf_traffic::schedule::Participant participant)
    {
      std::unique_lock<std::mutex> lock(ready_mutex, std::defer_lock);
      while (!lock.try_lock())
      {
        // Intentional busy wait
      }

      ready_callback(
        ScheduleManager{
          node,
          std::move(participant),
          negotiation
        });
    });
}

} // namespace rmf_fleet_adapter
