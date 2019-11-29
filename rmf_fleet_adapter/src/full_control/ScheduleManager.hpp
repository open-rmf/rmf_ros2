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

namespace rmf_fleet_adapter {
namespace full_control {

class FleetAdapterNode;

//==============================================================================
// TODO(MXG): Move this into rmf_traffic_ros2 as a generalized utility class
class ScheduleManager
{
public:

  ScheduleManager(
      FleetAdapterNode* node,
      std::function<void()> revision_callback);

  bool active() const;

  void activate();

  using TrajectorySet = std::vector<rmf_traffic::Trajectory>;

  void push_trajectories(
      const TrajectorySet& trajectories,
      std::function<void()> approval_callback);

  void push_delay(
      const rmf_traffic::Duration duration,
      const rmf_traffic::Time from_time);

  bool waiting() const;

  ~ScheduleManager();

private:

  void submit_trajectories(
      const TrajectorySet& trajectories,
      std::function<void()> approval_callback);

  void replace_trajectories(
      const TrajectorySet& trajectories,
      std::function<void()> approval_callback);

  void resolve_trajectories(
      const TrajectorySet& trajectories,
      std::function<void()> approval_callback);

  bool process_queues();

  class ConflictListener;

  FleetAdapterNode* _node;
  std::function<void()> _revision_callback;

  std::function<void()> _queued_change;
  std::vector<std::function<void()>> _queued_delays;

  bool _active = false;

  std::vector<rmf_traffic::schedule::Version> _schedule_ids;
  bool _waiting_for_schedule = false;

  std::unique_ptr<ConflictListener> _conflict_listener;
  bool _have_conflict = false;
  rmf_traffic::schedule::Version _last_conflict_version;
  rmf_traffic::schedule::Version _last_revised_version;
};

} // namespace full_control
} // namespace rmf_fleet_adapter


#endif // SRC__FULL_CONTROL__SCHEDULEMANAGER_HPP
