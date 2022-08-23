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

#include <rmf_traffic_ros2/Time.hpp>

#include "internal_EasyTrafficLight.hpp"

#include <rmf_utils/Modular.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
void EasyTrafficLight::Implementation::DependencyTracker::add(
  const rmf_traffic::Dependency& dep,
  const std::shared_ptr<const rmf_traffic::schedule::Mirror>& mirror)
{
  _subscriptions.push_back(
    std::make_shared<Dependency>(
      mirror->watch_dependency(dep, []() {}, []() {})));
}

//==============================================================================
void EasyTrafficLight::Implementation::DependencyTracker::set_time(
  const rmf_traffic::Time expected_time)
{
  if (!_expected_time.has_value() || *_expected_time < expected_time)
    _expected_time = expected_time;
}

//==============================================================================
bool EasyTrafficLight::Implementation::DependencyTracker::ready() const
{
  for (const auto& dep : _subscriptions)
  {
    if (!dep->reached())
      return false;
  }

  return true;
}

//==============================================================================
bool EasyTrafficLight::Implementation::DependencyTracker::deprecated(
  const rmf_traffic::Time current_time) const
{
  for (const auto& dep : _subscriptions)
  {
    if (dep->deprecated())
      return true;
  }

  // TODO(MXG): Make the limit configurable.
  // TODO(MXG): Instead of having a timeout limit, we should look for circular
  // dependencies in the blockade+schedule. Perhaps the blockade and schedule
  // systems should be merged somehow.
  const auto limit = std::chrono::minutes(1);
  if (_expected_time.has_value() && *_expected_time + limit < current_time)
    return true;

  return false;
}

//==============================================================================
std::unique_lock<std::recursive_mutex>
EasyTrafficLight::Implementation::Shared::lock()
{
  std::unique_lock<std::recursive_mutex> lock_(mutex, std::defer_lock);
  while (!lock_.try_lock())
  {
    // Intentionally busy wait
  }

  return lock_;
}

//==============================================================================
EasyTrafficLight::Implementation::Shared::Shared(Hooks hooks)
: state{},
  hooks{std::move(hooks)},
  path_version{0}
{
  // Do nothing
}

//==============================================================================
void EasyTrafficLight::Implementation::State::clear()
{
  proposal.reset();
  current_plan.reset();
  checkpoints.clear();
  range = {0, 0};
  last_reached = 0;
  last_passed.reset();
  idle_location.reset();
  planner = nullptr;
  itinerary->clear();
  blockade->cancel();
  find_path_service = nullptr;
  find_path_subscription.unsubscribe();
}

//==============================================================================
struct Range
{
  std::optional<std::size_t> lower;
  std::optional<std::size_t> upper;

  void consider(std::size_t c)
  {
    if (!lower.has_value() || c < *lower)
      lower = c;

    if (!upper.has_value() || *upper < c)
      upper = c;
  }
};

//==============================================================================
rmf_traffic::schedule::Itinerary
EasyTrafficLight::Implementation::State::current_itinerary_slice() const
{
  const auto& itin = itinerary->itinerary();
  const bool include_initial_wps = [&]() -> bool
    {
      for (const auto& wp : current_plan->plan.get_waypoints())
      {
        if (wp.graph_index().has_value())
          return wp.graph_index() > last_passed;
      }

      // *INDENT-OFF*
      throw std::runtime_error(
        "[EasyTrafficLight::Implementation::Shared::respond] Internal bug: "
        "Current plan of size ["
        + std::to_string(current_plan->plan.get_waypoints().size())
        + "] does not contain any graph indices. Please report this to the "
        "RMF developers.");
      // *INDENT-ON*
    } ();

  const auto include = [&](std::optional<std::size_t> opt_wp) -> bool
    {
      if (!opt_wp.has_value())
        return include_initial_wps;

      const auto wp = *opt_wp;
      if (!last_passed.has_value())
        return wp == 0;

      return (*last_passed <= wp) && (wp <= *last_passed + 1);
    };

  const auto should_break = [&](std::optional<std::size_t> opt_wp) -> bool
    {
      // If this point has no value then definitely do not break here
      if (!opt_wp.has_value())
        return false;

      // If we have never passed a point, but this point has a value, then we
      // should break here.
      if (!last_passed.has_value())
        return true;

      const auto wp = *opt_wp;
      return wp >= (*last_passed + 1);
    };

  // We need to reject any proposal that would interfere with the segment of
  // the plan that we are already traversing.
  std::vector<Range> slice_ranges;
  slice_ranges.resize(itin.size());
  for (const auto& wp : current_plan->plan.get_waypoints())
  {
    for (const auto& [progress_wp, progress_cp, _] : wp.progress_checkpoints())
    {
      if (include(progress_wp))
      {
        for (const auto& c : progress_cp)
        {
          slice_ranges.at(c.route_id).consider(c.checkpoint_id);
        }
      }
    }

    if (include(wp.graph_index()))
    {
      for (const auto& c : wp.arrival_checkpoints())
      {
        slice_ranges.at(c.route_id).consider(c.checkpoint_id);
      }
    }

    if (should_break(wp.graph_index()))
      break;
  }

  std::vector<rmf_traffic::Route> slices;
  for (std::size_t i = 0; i < itin.size(); ++i)
  {
    const auto& range = slice_ranges.at(i);
    if (!range.lower.has_value() || !range.upper.has_value())
    {
      continue;
    }

    rmf_traffic::Trajectory partial_trajectory;
    const auto& route = itin.at(i);
    for (std::size_t j = *range.lower; j <= *range.upper; ++j)
    {
      partial_trajectory.insert(route.trajectory().at(j));
    }

    if (partial_trajectory.size() > 1)
    {
      slices.push_back({route.map(), std::move(partial_trajectory)});
    }
  }

  return slices;
}

//==============================================================================
auto EasyTrafficLight::Implementation::State::location() const
-> std::optional<Location>
{
  if (idle_location.has_value())
    return idle_location;

  if (!last_known_location.has_value())
    return std::nullopt;

  if (!planner)
    return std::nullopt;

  const auto& graph = planner->get_configuration().graph();
  const auto wp = last_known_location->waypoint();
  if (graph.num_waypoints() <= wp)
    return std::nullopt;

  const auto& g_wp = graph.get_waypoint(wp);
  const auto p = g_wp.get_location();
  return Location{g_wp.get_map_name(), {p[0], p[1], 0.0}};
}

//==============================================================================
void EasyTrafficLight::Implementation::Shared::follow_new_path(
  const std::vector<Waypoint>& new_path)
{
  ++path_version;
  state.clear();

  if (new_path.empty())
  {
    RCLCPP_INFO(
      hooks.node->get_logger(),
      "Traffic light controlled by robot [%s] in group [%s] is being taken off "
      "the schedule because it is idle.",
      state.itinerary->description().name().c_str(),
      state.itinerary->description().owner().c_str());
    return;
  }
  else if (new_path.size() == 1)
  {
    RCLCPP_ERROR(
      hooks.node->get_logger(),
      "Traffic light controlled robot [%s] owned by [%s] was given only "
      "one waypoint. The traffic light controller requires the robot to "
      "start and end at separate locations which are both designated safe "
      "zones for the robot.",
      state.itinerary->description().name().c_str(),
      state.itinerary->description().owner().c_str());
    assert(false);
    return;
  }

  for (std::size_t i = 1; i < new_path.size(); ++i)
  {
    const auto& wp0 = new_path[i-1];
    const auto& wp1 = new_path[i];

    const auto p0 = wp0.position();
    const auto p1 = wp1.position();

    const double dist = (p1 - p0).norm();
    if (dist < 1e-3 && wp0.map_name() == wp1.map_name())
    {
      RCLCPP_ERROR(
        hooks.node->get_logger(),
        "Traffic light controlled robot [%s] owned by [%s] was given waypoints "
        "[%ld, %ld] that are too close together [%fm]",
        state.itinerary->description().name().c_str(),
        state.itinerary->description().owner().c_str(),
        i-1,
        i,
        dist);
      assert(false);
      return;
    }
  }

  rmf_traffic::agv::Graph graph;
  for (std::size_t i = 0; i < new_path.size(); ++i)
  {
    const auto& wp = new_path[i];
    graph.add_waypoint(wp.map_name(), wp.position().block<2, 1>(0, 0))
    .set_passthrough_point(!wp.yield())
    .set_holding_point(wp.yield());

    if (i > 0)
    {
      const auto& last_wp = new_path[i-1];
      rmf_traffic::agv::Graph::Lane::EventPtr event = nullptr;
      if (last_wp.mandatory_delay() > std::chrono::nanoseconds(0))
      {
        // We use DoorOpen for lack of a better placeholder
        event = rmf_traffic::agv::Graph::Lane::Event::make(
          rmf_traffic::agv::Graph::Lane::Wait(last_wp.mandatory_delay()));
      }

      graph.add_lane(rmf_traffic::agv::Graph::Lane::Node(i-1, event), i);
    }

    state.checkpoints.push_back(
      {wp.position().block<2, 1>(0, 0), wp.map_name(), wp.yield()});
  }

  state.blockade->set(state.checkpoints);

  state.planner = std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Plan::Configuration(graph, hooks.traits),
    rmf_traffic::agv::Plan::Options(nullptr));

  const auto now = rmf_traffic_ros2::convert(hooks.node->now());
  rmf_traffic::agv::Plan::Start start{now, 0, new_path.front().position()[2]};
  state.last_known_location = start;
  make_plan(path_version, std::move(start));
}

//==============================================================================
void EasyTrafficLight::Implementation::Shared::make_plan(
  const std::size_t request_path_version,
  rmf_traffic::agv::Plan::Start start)
{
  if (path_version != request_path_version)
  {
    return;
  }

  if (state.find_path_service)
  {
    return;
  }

  if (!negotiate_services.empty())
  {
    return;
  }

  const auto plan_id = state.itinerary->assign_plan_id();

  rmf_traffic::agv::Plan::Goal goal(
    state.planner->get_configuration().graph().num_waypoints()-1);

  state.find_path_service = std::make_shared<services::FindPath>(
    state.planner, rmf_traffic::agv::Plan::StartSet{std::move(start)},
    std::move(goal), hooks.schedule->snapshot(), state.itinerary->id(),
    hooks.profile);

  state.find_path_subscription =
    rmf_rxcpp::make_job<services::FindPath::Result>(
    state.find_path_service)
    .observe_on(rxcpp::identity_same_worker(hooks.worker))
    .subscribe(
    [w = weak_from_this(), request_path_version, plan_id](
      const services::FindPath::Result& result)
    {
      const auto self = w.lock();
      if (!self)
        return;

      if (!result.success())
      {
        RCLCPP_ERROR(
          self->hooks.node->get_logger(),
          "Failed to find any itinerary for submitted path #%ld of robot [%s] "
          "in group [%s]. This is a critical bug and should be reported to the "
          "RMF developers.",
          self->path_version,
          self->state.itinerary->description().name().c_str(),
          self->state.itinerary->description().owner().c_str());
        return;
      }

      self->state.find_path_service = nullptr;
      self->receive_plan(request_path_version, plan_id, *result);
    });
}

//==============================================================================
std::optional<rmf_traffic::schedule::ItineraryVersion>
EasyTrafficLight::Implementation::Shared::receive_plan(
  const std::size_t request_path_version,
  const rmf_traffic::PlanId plan_id,
  const rmf_traffic::agv::Plan& plan)
{
  const auto l = lock();

  if (request_path_version != path_version)
    return std::nullopt;

  if (state.proposal.has_value())
  {
    if (rmf_utils::modular(plan_id).less_than(state.proposal->id))
      return std::nullopt;
  }

  if (state.current_plan.has_value())
  {
    if (rmf_utils::modular(plan_id).less_than(state.current_plan->id))
      return std::nullopt;
  }

  if (rmf_utils::modular(plan_id).less_than(state.itinerary->current_plan_id()))
    return std::nullopt;

  Plan new_plan{plan_id, plan, {}, {}, {}};
  for (const auto& wp : plan.get_waypoints())
  {
    if (wp.graph_index().has_value())
    {
      auto& dep_tracker = new_plan.dependencies[*wp.graph_index()];
      dep_tracker.set_time(wp.time());

      for (const auto& dep : wp.dependencies())
      {
        dep_tracker.add(dep, hooks.schedule);
      }

      for (const auto& [c, progress, t] : wp.progress_checkpoints())
      {
        for (const auto& arr : progress)
          new_plan.arrivals[c].push_back(arr);
      }

      for (const auto& arr : wp.arrival_checkpoints())
        new_plan.arrivals[*wp.graph_index()].push_back(arr);
    }
    else
    {
      for (const auto& dep : wp.dependencies())
        new_plan.immediate_stop_dependencies.add(dep, hooks.schedule);
      new_plan.immediate_stop_dependencies.set_time(wp.time());
    }
  }

  state.proposal = std::move(new_plan);

  if (!state.proposal->immediate_stop_dependencies.ready())
  {
    // If there are dependencies calling for an immediate stop, then we should
    // immediately trigger the pause callback.
    hooks.pause_callback();
  }

  state.itinerary->set(plan_id, plan.get_itinerary());
  return state.itinerary->version();
}

//==============================================================================
void EasyTrafficLight::Implementation::Shared::update_immediate_stop(
  std::size_t checkpoint,
  std::optional<Eigen::Vector3d> location)
{
  // A proposed plan was rejected so we will create an itinerary that says the
  // robot will stand in place for a while.
  const auto& wp = state.planner->get_configuration()
    .graph().get_waypoint(checkpoint);
  const auto& map_name = wp.get_map_name();

  auto route = rmf_traffic::Route(map_name, rmf_traffic::Trajectory());

  if (!location.has_value())
  {
    const auto p = wp.get_location();
    // TODO(MXG): See if we can infer the yaw in a nice way
    location = Eigen::Vector3d(p[0], p[1], 0.0);
  }

  const auto now = hooks.node->rmf_now();
  route.trajectory().insert(now, *location, Eigen::Vector3d::Zero());
  route.trajectory().insert(
    now + std::chrono::seconds(30), *location, Eigen::Vector3d::Zero());

  state.itinerary->set(state.itinerary->assign_plan_id(), {std::move(route)});
}

//==============================================================================
void EasyTrafficLight::Implementation::Shared::update_delay(
  const std::size_t checkpoint,
  const std::optional<Eigen::Vector3d> location)
{
  // We update the last_reacheed and last_passed here because we need to make
  // sure they are set correctly before we call state.current_itinerary_slice()

  std::optional<rmf_traffic::Duration> new_delay;
  if (location.has_value())
  {
    const auto slices = state.current_itinerary_slice();

    for (const auto& slice : slices)
    {
      try
      {
        const auto [expected_time, _] =
          rmf_traffic::agv::interpolate_time_along_quadratic_straight_line(
          slice.trajectory(), location->block<2, 1>(0, 0));

        new_delay = hooks.node->rmf_now() - expected_time;
        break;
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(
          hooks.node->get_logger(),
          "[EasyTrafficLight::Implementation::Shared::update_location] "
          "Issue while interpolating time: %s", e.what());
      }
    }
  }
  else
  {
    for (const auto& wp : state.current_plan->plan.get_waypoints())
    {
      if (!wp.graph_index().has_value())
        continue;

      for (const auto& [progress, _, time] : wp.progress_checkpoints())
      {
        if (progress == checkpoint)
        {
          new_delay = hooks.node->rmf_now() - time - state.itinerary->delay();
          break;
        }
      }

      if (new_delay.has_value())
        break;

      if (*wp.graph_index() != checkpoint)
        continue;

      new_delay =
        (hooks.node->rmf_now() - wp.time()) - state.itinerary->delay();
    }
  }

  if (new_delay.has_value())
  {
    if (std::chrono::abs(*new_delay) > std::chrono::seconds(1))
    {
      if (std::chrono::abs(*new_delay) > std::chrono::hours(1))
      {
        const auto t = rmf_traffic::time::to_seconds(*new_delay);
        // If this happens, there may be an edge case that
        // interpolate_time_along_quadratic_straight_line is not accounting for.
        // *INDENT-OFF*
        throw std::runtime_error(
          "[EasyTrafficLight::Implementation::Shared::update_delay] "
          "Excessive delay was calculated: " + std::to_string(t)
          + "s. This is likely an internal bug in RMF. Please report this to "
          "the RMF developers.");
        // *INDENT-ON*
      }

      state.itinerary->delay(*new_delay);
    }
  }
  else
  {
    RCLCPP_ERROR(
      hooks.node->get_logger(),
      "[EasyTrafficLight::Implementation::Shared::update_location] "
      "Failed to interpolate the time delay for [%s]",
      state.itinerary->description().name().c_str());
  }
}

//==============================================================================
bool EasyTrafficLight::Implementation::Shared::update_location(
  const std::size_t checkpoint,
  const std::optional<Eigen::Vector3d> location)
{
  if (state.checkpoints.empty())
    return false;

  if (!state.current_plan.has_value())
  {
    // If a plan has never been found, then we can skip updating the location
    // because there will be no itinerary to adjust the time for.
    return true;
  }

  if (location.has_value())
  {
    if (checkpoint >= state.range.end)
      return false;
  }
  else
  {
    if (checkpoint > state.range.end)
      return false;
  }

  state.blockade->reached(checkpoint);
  for (const auto& arr : state.current_plan->arrivals[checkpoint])
  {
    state.itinerary->reached(
      state.current_plan->id, arr.route_id, arr.checkpoint_id);
  }

  state.last_reached = checkpoint;
  if (location.has_value())
    state.last_passed = checkpoint;
  else if (checkpoint > 0)
    state.last_passed = checkpoint-1;

  if (checkpoint >= state.checkpoints.size() - 1)
  {
    state.last_known_location = rmf_traffic::agv::Plan::Start{
      hooks.node->rmf_now(), state.checkpoints.size() - 1, 0.0
    };
    return true;
  }

  if (location.has_value())
  {
    state.last_known_location = rmf_traffic::agv::Plan::Start{
      hooks.node->rmf_now(), checkpoint+1, (*location)[2],
      location->block<2, 1>(0, 0)
    };
  }
  else if (state.last_known_location.has_value())
  {
    state.last_known_location->time(hooks.node->rmf_now());
    state.last_known_location->waypoint(checkpoint);
  }
  else
  {
    return false;
  }

  return true;
}

//==============================================================================
bool EasyTrafficLight::Implementation::Shared::consider_proposal(
  const std::size_t checkpoint,
  std::optional<Eigen::Vector3d> location)
{
  if (!state.proposal.has_value())
    return state.current_plan.has_value();

  if (state.proposal->id != state.itinerary->current_plan_id())
  {
    // A new plan slipped in while this proposal was waiting, so we will clear
    // out the proposal.
    state.proposal = std::nullopt;
    if (!state.current_plan.has_value())
    {
      update_immediate_stop(checkpoint, location);
      make_plan(path_version, state.last_known_location.value());
      return false;
    }

    return true;
  }

  const auto& deps = state.proposal->dependencies;
  auto d_it = deps.begin();
  for (; d_it != deps.end() && d_it->first <= state.last_passed; ++d_it)
  {
    // If there are any dependencies on a passed checkpoint and those
    // dependencies were not ready, then we need to reject the proposal.
    if (!d_it->second.ready())
    {
      state.current_plan = std::nullopt;
      update_immediate_stop(checkpoint, location);
      make_plan(path_version, state.last_known_location.value());
      return false;
    }
  }

  for (std::size_t i = state.range.begin; i < state.range.end; ++i)
  {
    d_it = deps.find(i);
    if (d_it == deps.end())
      continue;

    if (!d_it->second.ready())
    {
      // If our dependencies changed, then we should release the portion of the
      // range that has unmet dependencies.
      state.blockade->release(i);
      state.range.end = i;
      break;
    }
  }

  state.current_plan = state.proposal;
  state.proposal = std::nullopt;
  return true;
}

//==============================================================================
bool EasyTrafficLight::Implementation::Shared::finish_immediate_stop()
{
  if (state.current_plan->immediate_stop_dependencies.deprecated(
      hooks.node->rmf_now()))
  {
    make_plan(path_version, state.last_known_location.value());
    return false;
  }

  return state.current_plan->immediate_stop_dependencies.ready();
}

//==============================================================================
bool EasyTrafficLight::Implementation::Shared::check_if_ready(
  std::size_t to_move_past_checkpoint)
{
  if (to_move_past_checkpoint < state.range.end)
    return true;

  const auto& dependency =
    state.current_plan.value().dependencies[to_move_past_checkpoint];

  if (dependency.deprecated(hooks.node->rmf_now()))
  {
    make_plan(path_version, state.last_known_location.value());
    return false;
  }

  if (dependency.ready())
  {
    // Notify the blockade that we are ready to move past the next checkpoint.
    state.blockade->ready(to_move_past_checkpoint);
  }

  // Always return false here because if it were actually ready to move past
  // the next waypoint, we would have returned true at the start of this
  // function. We need to wait until the blockade approves.
  return false;
}

//==============================================================================
auto EasyTrafficLight::Implementation::Shared::moving_from(
  std::size_t checkpoint,
  Eigen::Vector3d location) -> MovingInstruction
{
  const auto l = lock();

  if (!update_location(checkpoint, location))
    return MovingInstruction::MovingError;

  if (!consider_proposal(checkpoint, location))
    return MovingInstruction::PauseImmediately;

  update_delay(checkpoint, location);

  if (!finish_immediate_stop())
    return MovingInstruction::PauseImmediately;

  if (!check_if_ready(checkpoint + 1))
    return MovingInstruction::WaitAtNextCheckpoint;

  return MovingInstruction::ContinueAtNextCheckpoint;
}

//==============================================================================
auto EasyTrafficLight::Implementation::Shared::waiting_at(
  std::size_t checkpoint) -> WaitingInstruction
{
  const auto l = lock();

  if (!update_location(checkpoint, std::nullopt))
    return WaitingInstruction::WaitingError;

  if (!consider_proposal(checkpoint, std::nullopt))
    return WaitingInstruction::Wait;

  update_delay(checkpoint, std::nullopt);

  if (!finish_immediate_stop())
    return WaitingInstruction::Wait;

  if (!check_if_ready(checkpoint))
    return WaitingInstruction::Wait;

  return WaitingInstruction::Resume;
}

//==============================================================================
auto EasyTrafficLight::Implementation::Shared::waiting_after(
  std::size_t checkpoint,
  Eigen::Vector3d location) -> WaitingInstruction
{
  const auto l = lock();

  if (!update_location(checkpoint, location))
    return WaitingInstruction::WaitingError;

  if (!consider_proposal(checkpoint, location))
    return WaitingInstruction::Wait;

  update_delay(checkpoint, location);

  if (!finish_immediate_stop())
    return WaitingInstruction::Wait;

  // We don't need to check if the next waypoint is ready.
  // When the robot is waiting after a checkpoint, we only care about whether
  // or not it can resume moving towards it next immediate target.
  return WaitingInstruction::Resume;
}

//==============================================================================
void EasyTrafficLight::Implementation::Shared::update_idle_location(
  std::string map_name,
  Eigen::Vector3d position)
{
  const auto l = lock();

  if (!state.checkpoints.empty())
    state.clear();

  state.idle_location = Location{std::move(map_name), position};
}

//==============================================================================
void EasyTrafficLight::Implementation::Shared::receive_new_range(
  const rmf_traffic::blockade::ReservationId reservation_id,
  const rmf_traffic::blockade::ReservedRange& new_range)
{
  const auto l = lock();

  if (reservation_id != state.blockade->reservation_id())
    return;

  hooks.worker.schedule(
    [w = weak_from_this(), new_range](const auto&)
    {
      if (const auto self = w.lock())
        self->state.range = new_range;
    });
}

//==============================================================================
bool rejected(
  const EasyTrafficLight::Implementation::State& state,
  const rmf_traffic::schedule::Negotiator::TableViewerPtr& viewer,
  const rmf_traffic::schedule::Negotiator::ResponderPtr& responder)
{
  if (state.current_plan.has_value() && state.last_passed > state.last_reached)
  {
    const auto slices = state.current_itinerary_slice();

    auto validators =
      rmf_traffic::agv::NegotiatingRouteValidator::Generator(viewer).all();

    const auto start_t = viewer->earliest_base_proposal_time();
    const auto finish_t = viewer->latest_base_proposal_time();
    if (!start_t.has_value() || !finish_t.has_value())
      return false;

    bool reject_table = false;
    for (const auto& slice : slices)
    {
      if (slice.trajectory().size() < 2)
        continue;

      const auto p = slice.trajectory().back().position();
      rmf_traffic::Trajectory endcap;
      endcap.insert(*start_t, p, Eigen::Vector3d::Zero());
      endcap.insert(*finish_t, p, Eigen::Vector3d::Zero());
      rmf_traffic::Route endcap_route(slice.map(), std::move(endcap));

      bool at_least_one_valid = false;
      for (const auto& v : validators)
      {
        if (v->find_conflict(slice).has_value())
          continue;

        if (v->find_conflict(endcap_route).has_value())
          continue;

        at_least_one_valid = true;
      }

      if (!at_least_one_valid)
      {
        // We must reject the table because it interferes with a region that
        // the robot is already travelling down.
        reject_table = true;
        break;
      }
    }

    if (reject_table)
    {
      responder->reject({std::move(slices)});
      return true;
    }
  }

  return false;
}

//==============================================================================
void EasyTrafficLight::Implementation::Shared::respond(
  const rmf_traffic::schedule::Negotiator::TableViewerPtr& viewer,
  const rmf_traffic::schedule::Negotiator::ResponderPtr& responder)
{
  if (state.last_reached >= state.checkpoints.size())
  {
    // We have reached the end of the path, so there is no need to negotiate.
    return responder->forfeit({});
  }

  if (!state.last_known_location.has_value())
  {
    // This should never happen...
    RCLCPP_WARN(
      hooks.node->get_logger(),
      "[EasyTrafficLight::Implementation::Shared::respond] Responding to "
      "negotiation without a last known location. This should not happen. "
      "Please report this to the RMF developers.");
    return responder->forfeit({});
  }

  if (rejected(state, viewer, responder))
  {
    // The current proposal on the table is unacceptable.
    return;
  }

  rmf_traffic::agv::Plan::Goal goal(
    state.planner->get_configuration().graph().num_waypoints()-1);

  auto approval_cb =
    [w = weak_from_this(), request_path_version = path_version](
    const rmf_traffic::PlanId plan_id,
    const rmf_traffic::agv::Plan& plan,
    const auto&)
    -> std::optional<rmf_traffic::schedule::ItineraryVersion>
    {
      if (const auto self = w.lock())
        return self->receive_plan(request_path_version, plan_id, plan);

      return std::nullopt;
    };

  // TODO(MXG): The management of negotiation services should probably get
  // wrapped in its own class to be shared between this module and the GoToPlace
  // phase implementation.
  services::ProgressEvaluator evaluator;
  if (viewer->parent_id())
  {
    const auto& s = viewer->sequence();
    assert(s.size() >= 2);
    evaluator.compliant_leeway_base *= s[s.size()-2].version + 1;
  }

  auto negotiate = services::Negotiate::path(
    state.itinerary->assign_plan_id(), state.planner,
    {*state.last_known_location}, std::move(goal), {},
    viewer, responder, std::move(approval_cb), evaluator);

  auto negotiate_sub =
    rmf_rxcpp::make_job<services::Negotiate::Result>(negotiate)
    .observe_on(rxcpp::identity_same_worker(hooks.worker))
    .subscribe(
    [w = weak_from_this()](const auto& result)
    {
      if (const auto self = w.lock())
      {
        result.respond();
        self->negotiate_services.erase(result.service);
      }
      else
      {
        result.service->responder()->forfeit({});
      }
    });

  using namespace std::chrono_literals;
  const auto wait_duration = 2s + viewer->sequence().back().version * 10s;

  auto negotiate_timer = hooks.node->try_create_wall_timer(
    wait_duration,
    [s = negotiate->weak_from_this()]()
    {
      if (const auto service = s.lock())
        service->interrupt();
    });

  negotiate_services[negotiate] = NegotiateManagers{
    std::move(negotiate_sub),
    std::move(negotiate_timer)
  };
}

//==============================================================================
void EasyTrafficLight::Implementation::Shared::publish_fleet_state() const
{
  const auto reported_location = state.location();
  if (!reported_location.has_value())
    return;

  auto robot_mode = [&]()
    {
      if (state.range.begin == state.range.end)
      {
        return rmf_fleet_msgs::build<rmf_fleet_msgs::msg::RobotMode>()
          .mode(rmf_fleet_msgs::msg::RobotMode::MODE_WAITING)
          // NOTE(MXG): This field is currently only used by the fleet drivers.
          // For now, we will just fill it with a zero.
          .mode_request_id(0);
      }

      return rmf_fleet_msgs::build<rmf_fleet_msgs::msg::RobotMode>()
        .mode(rmf_fleet_msgs::msg::RobotMode::MODE_MOVING)
        // NOTE(MXG): This field is currently only used by the fleet drivers.
        // For now, we will just fill it with a zero.
        .mode_request_id(0);
    } ();

  const auto& map = reported_location->map;
  const auto p = reported_location->position;
  auto location = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::Location>()
    .t(hooks.node->now())
    .x(p.x())
    .y(p.y())
    .yaw(p[2])
    .obey_approach_speed_limit(false)
    .approach_speed_limit(0.0)
    .level_name(map)
    .index(0);

  const auto& fleet_name = state.itinerary->description().owner();
  auto robot_state = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::RobotState>()
    .name(state.itinerary->description().name())
    .model(fleet_name)
    // TODO(MXG): Have a way to fill this in
    .task_id("")
    // TODO(MXG): We could keep track of the seq value and increment it once
    // with each publication. This is not currently an important feature
    // outside of the fleet driver, so for now we just set it to zero.
    .seq(0)
    .mode(std::move(robot_mode))
    // We multiply by 100 to convert from the [0.0, 1.0] range to percentage
    .battery_percent(battery_soc*100.0)
    .location(std::move(location))
    .path({});
}

//==============================================================================
EasyTrafficLight::Implementation::Negotiator::Negotiator(
  const std::shared_ptr<Shared>& shared)
: _shared(shared)
{
  // Do nothing
}

//==============================================================================
void EasyTrafficLight::Implementation::Negotiator::respond(
  const TableViewerPtr& table_viewer,
  const ResponderPtr& responder)
{
  if (const auto shared = _shared.lock())
    return shared->respond(table_viewer, responder);

  responder->forfeit({});
}

//==============================================================================
rmf_traffic::blockade::Participant make_blockade(
  rmf_traffic_ros2::blockade::Writer& writer,
  const std::shared_ptr<EasyTrafficLight::Implementation::Shared>& shared)
{
  const double radius = shared->state.itinerary->description().profile()
    .vicinity()->get_characteristic_length();

  auto new_range_cb =
    [w = shared->weak_from_this()](
    const rmf_traffic::blockade::ReservationId reservation,
    const rmf_traffic::blockade::ReservedRange& range)
    {
      if (const auto self = w.lock())
        self->receive_new_range(reservation, range);
    };

  return writer.make_participant(
    shared->state.itinerary->id(), radius, std::move(new_range_cb));
}

//==============================================================================
EasyTrafficLightPtr EasyTrafficLight::Implementation::make(
  std::function<void()> pause_,
  std::function<void()> resume_,
  std::function<void(std::vector<Blocker>)> blocker_,
  std::shared_ptr<const rmf_traffic::schedule::Mirror> schedule_,
  rxcpp::schedulers::worker worker_,
  std::shared_ptr<Node> node_,
  rmf_traffic::agv::VehicleTraits traits_,
  rmf_traffic::schedule::Participant itinerary_,
  std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer_,
  rmf_traffic_ros2::schedule::Negotiation* negotiation_)
{
  std::shared_ptr<EasyTrafficLight> handle(new EasyTrafficLight);
  handle->_pimpl = rmf_utils::make_unique_impl<Implementation>();

  handle->_pimpl->shared = std::make_shared<Shared>(
    Hooks{
      std::move(pause_),
      std::move(resume_),
      std::move(blocker_),
      std::move(schedule_),
      std::move(worker_),
      node_,
      std::move(traits_),
      std::make_shared<rmf_traffic::Profile>(
        itinerary_.description().profile()),
      node_->fleet_state(),
      nullptr
    });

  handle->_pimpl->shared->hooks.fleet_update_timer =
    handle->_pimpl->shared->hooks.node->try_create_wall_timer(
    std::chrono::seconds(1),
    [w = handle->_pimpl->shared->weak_from_this()]()
    {
      if (const auto self = w.lock())
        self->publish_fleet_state();
    });

  handle->_pimpl->shared->state.itinerary =
    std::make_shared<rmf_traffic::schedule::Participant>(std::move(itinerary_));

  handle->_pimpl->shared->name =
    handle->_pimpl->shared->state.itinerary->description().name();

  handle->_pimpl->shared->state.blockade =
    std::make_shared<rmf_traffic::blockade::Participant>(
    make_blockade(*blockade_writer_, handle->_pimpl->shared));

  if (negotiation_)
  {
    handle->_pimpl->shared->negotiation_license =
      negotiation_->register_negotiator(
      handle->_pimpl->shared->state.itinerary->id(),
      std::make_unique<Negotiator>(handle->_pimpl->shared));
  }

  return handle;
}

//==============================================================================
void EasyTrafficLight::follow_new_path(const std::vector<Waypoint>& new_path)
{
  _pimpl->shared->hooks.worker.schedule(
    [w = _pimpl->shared->weak_from_this(), new_path](const auto&)
    {
      if (const auto self = w.lock())
        self->follow_new_path(new_path);
    });
}

//==============================================================================
auto EasyTrafficLight::moving_from(
  std::size_t checkpoint,
  Eigen::Vector3d location) -> MovingInstruction
{
  return _pimpl->shared->moving_from(checkpoint, location);
}

//==============================================================================
auto EasyTrafficLight::waiting_at(std::size_t checkpoint) -> WaitingInstruction
{
  return _pimpl->shared->waiting_at(checkpoint);
}

//==============================================================================
auto EasyTrafficLight::waiting_after(
  std::size_t checkpoint, Eigen::Vector3d location) -> WaitingInstruction
{
  return _pimpl->shared->waiting_after(checkpoint, location);
}

//==============================================================================
std::size_t EasyTrafficLight::last_reached() const
{
  auto l = _pimpl->shared->lock();
  return _pimpl->shared->state.last_reached;
}

//==============================================================================
EasyTrafficLight& EasyTrafficLight::update_idle_location(
  std::string map_name,
  Eigen::Vector3d position)
{
  _pimpl->shared->hooks.worker.schedule(
    [w = _pimpl->shared->weak_from_this(), map_name, position](const auto&)
    {
      if (const auto self = w.lock())
        self->update_idle_location(map_name, position);
    });

  return *this;
}

//==============================================================================
EasyTrafficLight& EasyTrafficLight::update_battery_soc(double battery_soc)
{
  _pimpl->shared->hooks.worker.schedule(
    [w = _pimpl->shared->weak_from_this(), battery_soc](const auto&)
    {
      if (const auto self = w.lock())
        self->battery_soc = battery_soc;
    });

  return *this;
}

//==============================================================================
EasyTrafficLight& EasyTrafficLight::replan()
{
  _pimpl->shared->hooks.worker.schedule(
    [w = _pimpl->shared->weak_from_this()](const auto&)
    {
      if (const auto self = w.lock())
      {
        if (self->state.last_known_location.has_value())
        {
          self->make_plan(
            self->path_version, self->state.last_known_location.value());
        }
        else
        {
          RCLCPP_ERROR(
            self->hooks.node->get_logger(),
            "[EasyTrafficLight::replan] A replan was requested when the fleet "
            "adapter does not know the current location of the robot.");
        }
      }
    });

  return *this;
}

//==============================================================================
EasyTrafficLight& EasyTrafficLight::fleet_state_publish_period(
  std::optional<rmf_traffic::Duration> value)
{
  if (value.has_value())
  {
    _pimpl->shared->hooks.fleet_update_timer =
      _pimpl->shared->hooks.node->try_create_wall_timer(
      *value, [w = _pimpl->shared->weak_from_this()]()
      {
        if (const auto self = w.lock())
          self->publish_fleet_state();
      });
  }
  else
  {
    _pimpl->shared->hooks.fleet_update_timer = nullptr;
  }

  return *this;
}

//==============================================================================
class EasyTrafficLight::Blocker::Implementation
{
public:
  // TODO(MXG): We are not currently reporting the permanent blockers
  rmf_traffic::schedule::ParticipantId id;
  rmf_traffic::schedule::ParticipantDescription description;
};

//==============================================================================
rmf_traffic::schedule::ParticipantId
EasyTrafficLight::Blocker::participant_id() const
{
  return _pimpl->id;
}

//==============================================================================
const rmf_traffic::schedule::ParticipantDescription&
EasyTrafficLight::Blocker::description() const
{
  return _pimpl->description;
}

//==============================================================================
EasyTrafficLight::Blocker::Blocker()
{
  // Do nothing
}

//==============================================================================
EasyTrafficLight::EasyTrafficLight()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
