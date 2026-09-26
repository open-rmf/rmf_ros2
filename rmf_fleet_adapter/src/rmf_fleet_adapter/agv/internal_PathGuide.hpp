/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <rmf_fleet_adapter/agv/PathGuide.hpp>

#include "internal_FleetUpdateHandle.hpp"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>

namespace rmf_fleet_adapter {
namespace agv {

class RobotContext;

class PathGuideCommandHandle;
using PathGuideCommandHandlePtr = std::shared_ptr<PathGuideCommandHandle>;

//==============================================================================
/// Enforces that the path index reported to the traffic schedule never
/// backtracks, for the lifetime of one path.
///
/// RMF forbids this outright: once the fleet adapter has said the robot is
/// approaching path index 5, it may never again report 4 or less for that path.
/// See https://github.com/open-rmf/rmf_ros2/issues/530#issuecomment-5099640879
///
/// The reason is that `phases::MoveRobot` turns the reported index into an
/// itinerary delay — `now + estimate - waypoints[path_index].time()` — with no
/// clamping. A lower index has an earlier planned time, so the delay jumps
/// larger and the itinerary is pushed into the future. Other robots negotiate
/// against that itinerary, so a phantom delay makes this robot appear to occupy
/// space long after it has left, which can deadlock them.
/// (`Participant::reached` is safe by comparison: `Progress::update` clamps.)
///
/// PathGuide needs an explicit guard because the integrator, not the adapter,
/// chooses which index gets reported — it picks the ActivityIdentifier passed to
/// `PathRobotUpdateHandle::update`, and every command from the current one
/// onwards has a live update function. Reporting 7 and then 3 is therefore
/// possible, and must be caught here rather than trusted to integrator code.
///
/// This deliberately holds no context or command handle: the reaction is
/// injected, which keeps the type unit-testable with no ROS graph, fleet or
/// plan. \sa test/agv/test_PathIndexGuard.cpp
struct PathIndexGuard
{
  /// Invoked exactly once, on the first backtrack, as
  /// `on_backtrack(attempted_index, highest_reported)`. PathGuide supplies the
  /// log plus stop-and-replan; tests supply a recorder.
  std::function<void(std::size_t, std::size_t)> on_backtrack;

  /// The highest index ever reported for this path.
  std::size_t highest_reported = 0;

  /// False until the first successful report, so that index 0 is accepted
  /// without being mistaken for a backtrack against the default above.
  bool reported_any = false;

  /// Latches on the first backtrack so the reaction fires once and every
  /// subsequent report is refused.
  bool violated = false;

  /// Whether `index` may be reported to the arrival estimator.
  ///
  /// Returns false — and triggers `on_backtrack` the first time — if `index`
  /// would move the reported progress backwards.
  bool allow(std::size_t index)
  {
    if (violated)
    {
      // A stop and replan is already in flight; stay quiet.
      return false;
    }

    if (reported_any && index < highest_reported)
    {
      violated = true;
      if (on_backtrack)
      {
        on_backtrack(index, highest_reported);
      }
      return false;
    }

    if (!reported_any || index > highest_reported)
    {
      highest_reported = index;
    }
    reported_any = true;
    return true;
  }
};
using PathIndexGuardPtr = std::shared_ptr<PathIndexGuard>;

//==============================================================================
/// How far off its lane RMF can still localize this robot, at this waypoint.
///
/// **This is not an arrival radius.** PathGuide has no opinion about when a
/// waypoint counts as reached — that is the integrator's call, exactly as it is
/// under EasyFullControl, where the integrator decides arrival outright by
/// choosing when to call `finished()`. What the adapter can usefully say is how
/// far the robot may stray and still be found on the graph, which is an **upper
/// bound** on how loose an arrival test may safely be.
///
/// Why the bound matters: crediting a path's final waypoint ends the movement
/// phase, so the robot stops wherever it happens to be. If the integrator
/// credits it further out than this, RMF may be unable to merge the stopped
/// robot back onto its lane — it is declared lost, replans, and a lift already
/// summoned for it can be re-summoned to the wrong floor.
///
/// Two inputs: the robot's `max_merge_lane_distance`, widened to the graph
/// vertex's own `merge_radius` where the author set a larger one. A graph may
/// be more permissive at an unusually open vertex, never stricter.
///
/// Deciding arrival — including the far harder question of when a robot is far
/// enough inside a lift cabin for the doors to close safely — belongs to the
/// integrator, which knows its true footprint shape, its heading on arrival and
/// its controller's real stopping accuracy. The adapter knows none of those.
/// `PathGuide::Destination::inside_lift()` is what that decision is made from;
/// mind the coordinate frames noted there.
/// \sa test/agv/test_MergeRadius.cpp
inline double compute_merge_radius(
  double robot_max_merge_lane_distance,
  std::optional<double> vertex_merge_radius)
{
  return std::max(
    robot_max_merge_lane_distance, vertex_merge_radius.value_or(0.0));
}

//==============================================================================
class PathGuide::Implementation
{
public:
  std::shared_ptr<FleetUpdateHandle> fleet_handle;
  // Map robot name to its PathGuideCommandHandle
  std::unordered_map<std::string, PathGuideCommandHandlePtr> cmd_handles;
  NavParams nav_params;
  bool default_responsive_wait;
  bool use_parking_reservation;

  static std::shared_ptr<PathGuide> make(
    std::shared_ptr<FleetUpdateHandle> fleet_handle,
    std::shared_ptr<TransformDictionary> transforms_to_robot_coords,
    std::unordered_set<std::size_t> strict_lanes,
    bool default_responsive_wait,
    double default_max_merge_waypoint_distance,
    double default_max_merge_lane_distance,
    double default_min_lane_length,
    bool use_parking_reservation)
  {
    auto handle = std::shared_ptr<PathGuide>(new PathGuide);
    handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
      Implementation{
        fleet_handle,
        {},
        NavParams{
          // PathGuide never emits a rotate-in-place waypoint, so rotation
          // command skipping is always on. See PathGuide.hpp for why this is
          // not configurable.
          true,
          std::move(transforms_to_robot_coords),
          std::move(strict_lanes),
          default_max_merge_waypoint_distance,
          default_max_merge_lane_distance,
          default_min_lane_length,
        },
        default_responsive_wait,
        use_parking_reservation
      });
    return handle;
  }

  const std::shared_ptr<Node>& node() const
  {
    return FleetUpdateHandle::Implementation::get(*fleet_handle).node;
  }
};

//==============================================================================
class PathGuide::Destination::Implementation
{
public:
  std::string map;
  Eigen::Vector3d position;
  std::optional<std::size_t> graph_index;
  std::string name;
  std::optional<double> speed_limit;
  rmf_traffic::agv::Graph::LiftPropertiesPtr lift;
  std::optional<std::string> dock = std::nullopt;

  template<typename... Args>
  static Destination make(Args&&... args)
  {
    Destination output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::forward<Args>(args)...});
    return output;
  }

  static Implementation& get(Destination& self)
  {
    return *self._pimpl;
  }
};

//==============================================================================
class PathGuide::CommandExecution::Implementation
{
public:
  class Data;
  using DataPtr = std::shared_ptr<Data>;

  std::weak_ptr<RobotContext> w_context;
  std::shared_ptr<Data> data;

  /// Called once an acknowledgement has been accepted, to advance the path.
  std::function<void()> finisher;

  /// Gate for out-of-order acknowledgements. Returns true if this command is
  /// the one the path is currently tracking, and logs a warning otherwise. A
  /// nullptr means "always accept" and is used for the localization hold
  /// command, which does not belong to a path.
  std::function<bool()> accept_ack;

  ActivityIdentifierPtr identifier;

  void finish();

  Stubbornness override_schedule(
    std::string map,
    std::vector<Eigen::Vector3d> path,
    rmf_traffic::Duration hold);

  /// Stop this command from making any further updates to the robot's location
  /// or the traffic schedule. Used when a path is superseded or the robot is
  /// stopped, so that a late acknowledgement from the integrator is inert.
  void invalidate();

  static CommandExecution make(
    const std::shared_ptr<RobotContext>& context,
    Data data_);

  static CommandExecution make_hold(
    const std::shared_ptr<RobotContext>& context,
    rmf_traffic::Time expected_time,
    rmf_traffic::PlanId plan_id,
    std::function<void()> finisher);

  static Implementation& get(CommandExecution& cmd)
  {
    return *cmd._pimpl;
  }
};

//==============================================================================
class PathGuide::PathWaypoint::Implementation
{
public:
  Destination destination;
  double merge_radius;
  std::size_t index;
  CommandExecution execution;

  static PathWaypoint make(
    Destination destination,
    double merge_radius,
    std::size_t index,
    CommandExecution execution)
  {
    PathWaypoint output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(destination),
        merge_radius,
        index,
        std::move(execution)
      });
    return output;
  }
};

//==============================================================================
class PathGuide::Path::Implementation
{
public:
  std::vector<PathWaypoint> waypoints;
  std::size_t plan_id;
  std::string map;

  static Path make(
    std::vector<PathWaypoint> waypoints,
    std::size_t plan_id,
    std::string map)
  {
    Path output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::move(waypoints), plan_id, std::move(map)});
    return output;
  }
};

} // namespace agv
} // namespace rmf_fleet_adapter
