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

#include "internal_TrafficLight.hpp"

#include "../services/FindPath.hpp"
#include "../services/Negotiate.hpp"

#include <rmf_utils/Modular.hpp>
#include <rmf_utils/math.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/schedule/StubbornNegotiator.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class TrafficLight::UpdateHandle::Implementation::Data
    : public std::enable_shared_from_this<Data>
{
public:

  std::shared_ptr<CommandHandle> command;

  rmf_traffic::schedule::Participant itinerary;

  rmf_traffic::agv::VehicleTraits traits;

  std::shared_ptr<rmf_traffic::schedule::Snappable> schedule;

  rxcpp::schedulers::worker worker;

  std::shared_ptr<rclcpp::Node> node;

  std::size_t current_version = 0;

  std::shared_ptr<rmf_traffic::Profile> profile;

  std::vector<Waypoint> path;

  rmf_utils::optional<rmf_traffic::agv::Plan> plan;

  std::shared_ptr<rmf_traffic::agv::Planner> planner;

  std::vector<rmf_traffic::Time> arrival_timing;
  std::vector<rclcpp::Time> departure_timing;
  std::vector<std::size_t> plan_index;

  std::size_t processing_version = 0;

  std::shared_ptr<services::FindPath> find_path_service;
  rxcpp::subscription find_path_subscription;

  struct NegotiateManagers
  {
    rmf_rxcpp::subscription_guard subscription;
    rclcpp::TimerBase::SharedPtr timer;
  };
  using NegotiatePtr = std::shared_ptr<services::Negotiate>;
  using NegotiateServiceMap =
      std::unordered_map<NegotiatePtr, NegotiateManagers>;
  NegotiateServiceMap negotiate_services;

  void update_path(
      std::size_t version,
      const std::vector<Waypoint>& new_path);

  void plan_timing(
    rmf_traffic::agv::Plan::Start start,
    std::size_t version,
    std::vector<Waypoint> new_path,
    std::shared_ptr<rmf_traffic::agv::Planner> new_planner);

  rmf_utils::optional<rmf_traffic::schedule::ItineraryVersion> update_timing(
      std::size_t version,
      std::vector<Waypoint> new_path,
      rmf_traffic::agv::Plan new_plan,
      std::shared_ptr<rmf_traffic::agv::Planner> new_planner);

  void update_location(
      std::size_t version,
      std::size_t path_index,
      Eigen::Vector3d location);

  rmf_utils::optional<rmf_traffic::agv::Plan::Start> estimate_location() const;

  Data(
      std::shared_ptr<CommandHandle> command_,
      rmf_traffic::schedule::Participant itinerary_,
      rmf_traffic::agv::VehicleTraits traits_,
      std::shared_ptr<rmf_traffic::schedule::Snappable> schedule_,
      rxcpp::schedulers::worker worker_,
      std::shared_ptr<rclcpp::Node> node_)
    : command(std::move(command_)),
      itinerary(std::move(itinerary_)),
      traits(std::move(traits_)),
      schedule(std::move(schedule_)),
      worker(std::move(worker_)),
      node(std::move(node_)),
      profile(std::make_shared<rmf_traffic::Profile>(
                itinerary.description().profile()))
  {
    // Do nothing
  }
};

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::update_path(
    const std::size_t version,
    const std::vector<Waypoint>& new_path)
{
  if (rmf_utils::modular(version).less_than(processing_version))
    return;

  assert(version != processing_version);
  processing_version = version;

  const bool clear_itinerary = new_path.empty()
      || (new_path.size() == 1
          && new_path.front().mandatory_delay() <= std::chrono::nanoseconds(0));

  const auto now = rmf_traffic_ros2::convert(node->now());

  if (clear_itinerary)
  {
    // Clear out this robot's itinerary because it is done moving
    RCLCPP_INFO(
      node->get_logger(),
      "Traffic light controlled robot [%s] owned by [%s] is being taken off "
      "the schedule after being given a null path.",
      itinerary.description().name().c_str(),
      itinerary.description().owner().c_str());

    plan = rmf_utils::nullopt;
    planner = nullptr;
    path.clear();
    departure_timing.clear();
    itinerary.clear();
    return;
  }
  else if (new_path.size() == 1)
  {
    // Have the robot sit in place if it was given a single-point trajectory
    // with a mandatory delay
    plan = rmf_utils::nullopt;
    planner = nullptr;

    rmf_traffic::Trajectory sit;
    sit.insert(
      now,
      new_path.front().position(),
      Eigen::Vector3d::Zero());

    sit.insert(
      now + new_path.front().mandatory_delay(),
      new_path.front().position(),
      Eigen::Vector3d::Zero());

    itinerary.set({{new_path.front().map_name(), sit}});
  }

  for (std::size_t i=1; i < new_path.size(); ++i)
  {
    const auto& wp0 = new_path[i-1];
    const auto& wp1 = new_path[i];

    const auto p0 = wp0.position();
    const auto p1 = wp1.position();

    const double dist = (p1 - p0).norm();
    if (dist < 1e-3 && wp0.map_name() == wp1.map_name())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Traffic light controlled robot [%s] owned by [%s] was given waypoints "
        "[%d, %d] that are too close together [%fm]",
        itinerary.description().name().c_str(),
        itinerary.description().owner().c_str(),
        i-1, i, dist);
      assert(false);
      return;
    }
  }

  rmf_traffic::agv::Graph graph;
  for (std::size_t i=0; i < new_path.size(); ++i)
  {
    const auto& wp = new_path[i];
    graph.add_waypoint(wp.map_name(), wp.position().block<2,1>(0,0))
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
  }

  auto new_planner = std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Plan::Configuration(graph, traits),
        rmf_traffic::agv::Plan::Options(nullptr));

  rmf_traffic::agv::Plan::Start start{now, 0, new_path.front().position()[2]};
  plan_timing(std::move(start), version, new_path, std::move(new_planner));
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::plan_timing(
    rmf_traffic::agv::Plan::Start start,
    const std::size_t version,
    std::vector<Waypoint> new_path,
    std::shared_ptr<rmf_traffic::agv::Planner> new_planner)
{
  rmf_traffic::agv::Plan::Goal goal(
        new_planner->get_configuration().graph().num_waypoints()-1);

  find_path_service = std::make_shared<services::FindPath>(
    new_planner, rmf_traffic::agv::Plan::StartSet({std::move(start)}),
    std::move(goal), schedule->snapshot(), itinerary.id(), profile);

  find_path_subscription = rmf_rxcpp::make_job<services::FindPath::Result>(
        find_path_service)
      .observe_on(rxcpp::identity_same_worker(worker))
      .subscribe(
        [w = weak_from_this(),
         version,
         new_path = std::move(new_path),
         new_planner](
        const services::FindPath::Result& result)
  {
    const auto data = w.lock();
    if (!data)
      return;

    if (version != data->processing_version)
    {
      // This means the path that triggered this service is deprecated.
      return;
    }

    if (!result)
    {
      RCLCPP_ERROR(
        data->node->get_logger(),
        "Failed to find any itinerary for submitted path #%d of robot [%s] "
        "owned by [%s]. This is a critical bug and should be reported to the "
        "rmf_core developers.",
        version,
        data->itinerary.description().name().c_str(),
        data->itinerary.description().owner().c_str());

      return;
    }

    data->current_version = version;

    data->update_timing(
      version,
      std::move(new_path),
      std::move(*result),
      std::move(new_planner));
  });
}

namespace {

//==============================================================================
rmf_utils::optional<rmf_traffic::Time> linear_interpolate_time(
    const rmf_traffic::Trajectory::Waypoint& wp0,
    const rmf_traffic::Trajectory::Waypoint& wp1,
    const Eigen::Vector2d& p)
{
  const Eigen::Vector2d p0 = wp0.position().block<2,1>(0,0);
  const Eigen::Vector2d p1 = wp1.position().block<2,1>(0,0);
  if ((p1 - p0).dot(p - p0) < 0)
    return rmf_utils::nullopt;

  if ((p1 - p0).dot(p1 - p) < 0)
    return rmf_utils::nullopt;

  const double v = wp0.velocity().block<2,1>(0,0).norm();
  assert(std::abs(v - wp1.velocity().block<2,1>(0,0).norm()) < 1e-2);

  const double r = (p1 - p0).normalized().dot(p - p0);
  const double s = r/v;

  assert(s <= rmf_traffic::time::to_seconds(wp1.time() - wp0.time()));
  return wp0.time() + rmf_traffic::time::from_seconds(s);
}

//==============================================================================
rmf_utils::optional<rmf_traffic::Time> parabolic_interpolate_time(
    const rmf_traffic::Trajectory::Waypoint& wp0,
    const rmf_traffic::Trajectory::Waypoint& wp1,
    const Eigen::Vector2d& p)
{
  const Eigen::Vector2d p0 = wp0.position().block<2,1>(0,0);
  const Eigen::Vector2d p1 = wp1.position().block<2,1>(0,0);
  if ((p1 - p0).dot(p - p0) < 0)
    return rmf_utils::nullopt;

  if ((p1 - p0).dot(p1 - p) < 0)
    return rmf_utils::nullopt;

  const double r = (p - p0).norm();
  const double v0 = wp0.velocity().block<2,1>(0,0).norm();
  const double v1 = wp1.velocity().block<2,1>(0,0).norm();
  const double dt = rmf_traffic::time::to_seconds(wp1.time() - wp0.time());
  const double a = (v1 - v0)/dt;

  const double D = v0*v0 + 2*a*r;

  const double t_minus = (-v0 - std::sqrt(D))/a;
  if (0 <= t_minus && t_minus <= dt)
    return wp0.time() + rmf_traffic::time::from_seconds(t_minus);

  const double t_plus = (-v0 + std::sqrt(D))/a;
  if (0 <= t_plus && t_plus <= dt)
    return wp0.time() + rmf_traffic::time::from_seconds(t_plus);

  return rmf_utils::nullopt;
}

//==============================================================================
rmf_traffic::Time interpolate_time(
  const rmf_traffic::agv::VehicleTraits& traits,
  const rmf_traffic::agv::Plan::Waypoint& wp0,
  const rmf_traffic::agv::Plan::Waypoint& wp1,
  const Eigen::Vector2d& p)
{
  std::vector<Eigen::Vector3d> positions;
  positions.resize(2, Eigen::Vector3d::Zero());
  positions[0].block<2,1>(0,0) = wp0.position().block<2,1>(0,0);
  positions[1].block<2,1>(0,0) = wp1.position().block<2,1>(0,0);

  const auto interp = rmf_traffic::agv::Interpolate::positions(
        traits, wp0.time(), positions);

  // The interpolation will either be two parabolas or it will be two parabolas
  // with a linear component in the middle.
  assert(interp.size() == 3 || interp.size() == 4);
  const double r = (p - wp0.position().block<2,1>(0,0)).norm();
  if (r < 1e-3)
    return wp0.time();

  const auto end = interp.end();
  if (interp.size() == 3)
  {
    for (auto it = ++interp.begin(); it != end; ++it)
    {
      const auto last = --rmf_traffic::Trajectory::const_iterator(it);
      if (const auto t = parabolic_interpolate_time(*last, *it, p))
        return *t;
    }
  }
  else if(interp.size() == 4)
  {
    std::size_t count = 0;
    for (auto it = ++interp.begin(); it != end; ++it, ++count)
    {
      const auto last = --rmf_traffic::Trajectory::const_iterator(it);
      if (count == 0 || count == 2)
      {
        if (const auto t = parabolic_interpolate_time(*last, *it, p))
          return *t;
      }
      else
      {
        if (const auto t = linear_interpolate_time(*last, *it, p))
          return *t;
      }
    }
  }

  throw std::runtime_error(
    "[rmf_fleet_adapter::agv::TrafficLight] Failed to interpolate an "
    "intermediary waypoint.");
}

//==============================================================================
rmf_utils::optional<rmf_traffic::Time> interpolate_time(
  const rmf_traffic::Time now,
  const rmf_traffic::agv::VehicleTraits& traits,
  const rmf_traffic::agv::Plan& plan,
  const std::size_t plan_target,
  const Eigen::Vector3d& location)
{
  const Eigen::Vector2d p = location.block<2,1>(0,0);
  const auto& waypoints = plan.get_waypoints();

  assert(plan_target < waypoints.size());

  // If we end up receiving a plan_target that initiates the plan, just pretend
  // we were given the next plan target
  const std::size_t search_up_to = plan_target == 0? 1 : plan_target;

  for (std::size_t i=0; i < search_up_to; ++i)
  {
    const auto index = search_up_to - i;
    const auto& wp0 = waypoints[index-1];
    const auto& wp1 = waypoints[index];

    const auto gi_0 = wp0.graph_index();
    const auto gi_1 = wp1.graph_index();

    if (gi_1 && (!gi_0 || (*gi_0 != *gi_1)))
    {
      // Check if the robot is traversing a lane
      const Eigen::Vector2d p0 = wp0.position().block<2,1>(0,0);
      const Eigen::Vector2d p1 = wp1.position().block<2,1>(0,0);
      const Eigen::Vector2d n = p1 - p0;
      const double length = n.norm();
      const Eigen::Vector2d dir = n/length;

      const Eigen::Vector2d r = p - p0;
      const double traversal = r.dot(dir);
      const double deviation = (r - traversal*dir).norm();

      // TODO(MXG): Make this threshold configurable
      const double deviation_threshold = 1.0;

      // If the vehicle has deviated significantly from the path, then we should
      // recompute the timing information.
      if (deviation > deviation_threshold)
        return rmf_utils::nullopt;

      // If the vehicle is behind the lane then we should recompute the timing
      // information.
      if (traversal < 0.0)
        return rmf_utils::nullopt;

      try
      {
        return interpolate_time(traits, wp0, wp1, location.block<2,1>(0,0));
      }
      catch (const std::exception&)
      {
        // TODO(MXG): This is kind of suspicious. Should we escalate the issue
        // at this point?
        return rmf_utils::nullopt;
      }
    }
    else
    {
      // Check if the robot is on the target waypoint
      const Eigen::Vector2d p0 = wp0.position().block<2,1>(0,0);

      // TODO(MXG): Make this threshold configurable
      const double distance_threshold = 0.5;
      if ((p - p0).norm() > distance_threshold)
        continue;

      const double yaw = location[2];
      const auto yaw0 = wp0.position()[2];
      const auto yaw1 = wp1.position()[2];
      const double dyaw = rmf_utils::wrap_to_pi(yaw1 - yaw0);

      const auto t_min = wp0.time();
      const auto t_max = wp1.time();

      // TODO(MXG): Make this threshold configurable
      const double waiting_threshold = 1.0*M_PI/180.0;
      if (std::abs(dyaw) < waiting_threshold)
      {
        // This is a waiting segment

        // TODO(MXG): Make this threshold configurable
        const double yaw_threshold = 20.0*M_PI/180.0;

        if (std::abs(yaw - yaw0) < yaw_threshold)
        {
          // We are waiting

          // If the current time has not yet reached the end of the waiting time
          // then we just return the current time to imply that the robot is
          // on-time. This may leave a false shadow of an itinerary on the
          // schedule, but that should usually be okay.
          if (now <= t_max)
            return now;

          return t_max;
        }
      }
      else
      {
        // This is a turning segment

        // NOTE: For now we'll just approximate turns as constant-velocities
        // while estimating. We could make this estimation more accurate in
        // the future if this turns out to not be good enough.
        const double s = rmf_utils::wrap_to_pi(yaw - yaw0)/dyaw;
        const double dt = rmf_traffic::time::to_seconds(t_max - t_min);
        const auto t = t_min + rmf_traffic::time::from_seconds(s*dt);
        if (now <= t)
          return now;

        return t;
      }
    }
  }

  return rmf_utils::nullopt;
}

//==============================================================================
double calculate_orientation(
    const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1)
{
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}
} // anonymous namespace

//==============================================================================
rmf_utils::optional<rmf_traffic::schedule::ItineraryVersion>
TrafficLight::UpdateHandle::Implementation::Data::update_timing(
    const std::size_t version,
    std::vector<Waypoint> path_,
    rmf_traffic::agv::Plan plan_,
    std::shared_ptr<rmf_traffic::agv::Planner> planner_)
{
  if (version != current_version)
    return rmf_utils::nullopt;

  path = std::move(path_);
  plan = std::move(plan_);
  planner = std::move(planner_);

  assert(!path.empty());

  const rclcpp::Time initial_t =
      rmf_traffic_ros2::convert(plan->get_waypoints().front().time());

  // For time that should be considered in the past
  const rclcpp::Time passed_t = initial_t - rclcpp::Duration(3600.0);

  const auto& plan_waypoints = plan->get_waypoints();
  const auto& graph = planner->get_configuration().graph();

  arrival_timing.resize(path.size(), rmf_traffic_ros2::convert(initial_t));
  departure_timing.resize(path.size(), initial_t);
  plan_index.resize(path.size());

  rmf_utils::optional<std::size_t> current_wp;

  const std::size_t first_wp = [&]() -> std::size_t
  {
    for (const auto& wp : plan_waypoints)
    {
      if (wp.graph_index())
        return wp.graph_index().value();
    }

    // This should never happen
    assert(false);
    return 0;
  }();

  for (std::size_t i=0; i < plan_waypoints.size(); ++i)
  {
    const auto& wp = plan_waypoints[i];

    if (!wp.graph_index())
    {
      for (std::size_t k=0; k < first_wp; ++k)
      {
        departure_timing[k] = passed_t;
        plan_index[k] = i;
      }

      continue;
    }

    if (!current_wp)
      current_wp = *wp.graph_index();

    if (*wp.graph_index() != *current_wp)
    {
      const std::size_t next_wp = *wp.graph_index();
      for (std::size_t k=*current_wp+1; k < next_wp; ++k)
      {
        // For any intermediate waypoints that will be passed over without
        // stopping, we should try to calculate the time that the robot is
        // expected to pass over it.
        const auto p = graph.get_waypoint(k).get_location();
        const auto t = interpolate_time(traits, plan_waypoints[i-1], wp, p);
        arrival_timing[k] = t;
        departure_timing[k] = rmf_traffic_ros2::convert(t);
        plan_index[k] = i;
      }

      current_wp = *wp.graph_index();
      arrival_timing[*current_wp] = wp.time();
    }

    // This might get overriden several times since a single path waypoint might
    // show up several times in a plan. We want to use the last time that is
    // associated with this path waypoint.
    departure_timing[*current_wp] = rmf_traffic_ros2::convert(wp.time());
    plan_index[*current_wp] = i;
  }

  itinerary.set(plan->get_itinerary());

  std::vector<CommandHandle::Checkpoint> checkpoints;
  checkpoints.reserve(departure_timing.size());
  for (std::size_t i=0; i < departure_timing.size(); ++i)
  {
    auto depart_cb = [
        w = weak_from_this(),
        version = current_version,
        path_index = i+1](Eigen::Vector3d location)
    {
      // TODO(MXG): Fix the interpolate_time() function so that we can have
      // path_index = i instead of path_index = i+1 here.
      if (const auto data = w.lock())
        data->update_location(version, path_index, location);
    };

    checkpoints.push_back({i, departure_timing[i], std::move(depart_cb)});
  }

  command->receive_checkpoints(current_version, checkpoints, [](){ });

  return itinerary.version();
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::update_location(
    const std::size_t version,
    const std::size_t path_index,
    const Eigen::Vector3d location)
{
  const auto now = rmf_traffic_ros2::convert(node->now());

  worker.schedule(
        [w = weak_from_this(),
         version,
         path_index,
         location,
         now](const auto&)
  {
    const auto data = w.lock();
    if (!data)
      return;

    if (version != data->current_version)
      return;

    if (!data->plan)
      return;

    assert(path_index < data->arrival_timing.size());
    const auto expected_time = interpolate_time(
          now, data->traits, *data->plan,
          data->plan_index[path_index], location);

    if (expected_time)
    {
      const auto new_delay = now - *expected_time;
      const auto time_shift = new_delay - data->itinerary.delay();
      const auto threshold = std::chrono::seconds(1);
      if (time_shift < -threshold || threshold < time_shift)
        data->itinerary.delay(time_shift);

      return;
    }

    // TODO(MXG): If interpolate_time() is changed to use the departure
    RCLCPP_WARN(
      data->node->get_logger(),
      "Failed to compute timing estimate for [%s] owned by [%s] "
      "moving towards path index [%d]",
      data->itinerary.description().name().c_str(),
      data->itinerary.description().owner().c_str(),
      path_index);
  });
}

//==============================================================================
rmf_utils::optional<rmf_traffic::agv::Plan::Start>
TrafficLight::UpdateHandle::Implementation::Data::estimate_location() const
{
  // Estimate the current position of the robot based on its intended trajectory
  // and the delays that it has reported.

  const auto now = rmf_traffic_ros2::convert(node->now());

  const auto effective_time = now - itinerary.delay();
  const rclcpp::Time effective_time_rcl =
      rmf_traffic_ros2::convert(effective_time);

  if (departure_timing.back() < effective_time_rcl)
  {
    // The time that this robot is scheduled for has passed, so there's no way
    // to estimate its position.
    return rmf_utils::nullopt;
  }

  const auto& graph = planner->get_configuration().graph();

  rmf_traffic::agv::Plan::Start start(
        now, 0,
        calculate_orientation(
          path[0].position().block<2,1>(0,0),
          path[1].position().block<2,1>(0,0)));

  if (effective_time_rcl <= departure_timing.front())
  {
    // The robot has not made progress from its starting point, so we'll just
    // report its original starting conditions.
    return start;
  }

  bool traversing_lane = false;
  for (std::size_t i=1; i < departure_timing.size(); ++i)
  {
    if (effective_time_rcl < departure_timing[i])
    {
      start.waypoint(i);

      if (effective_time < arrival_timing[i])
      {
        // The vehicle is still moving towards this waypoint.
        const auto* lane = graph.lane_from(i-1, i);
        assert(lane);
        start.lane(lane->index());
        start.orientation(
              calculate_orientation(
                path[i-1].position().block<2,1>(0,0),
                path[i].position().block<2,1>(0,0)));

        traversing_lane = true;
      }

      break;
    }
  }

  for (const auto& route : plan->get_itinerary())
  {
    if (*route.trajectory().start_time() <= effective_time
        && effective_time <= *route.trajectory().finish_time())
    {
      const auto motion =
          rmf_traffic::Motion::compute_cubic_splines(route.trajectory());

      const auto p = motion->compute_position(effective_time);
      start.orientation(p[2]);

      if (traversing_lane)
        start.location(Eigen::Vector2d(p.block<2,1>(0,0)));
    }
  }

  return start;
}

//==============================================================================
class TrafficLight::UpdateHandle::Implementation::Negotiator
    : public rmf_traffic::schedule::Negotiator
{
public:

  Negotiator(std::shared_ptr<Data> data)
    : _data(std::move(data))
  {
    // Do nothing
  }

  void respond(
      const TableViewerPtr& table_viewer,
      const ResponderPtr& responder) final;

private:
  std::weak_ptr<Data> _data;
};

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Negotiator::respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder)
{
  const auto data = _data.lock();
  if (!data || !data->planner || !data->plan)
  {
    // If we no longer have access to the traffic light data or there is no
    // plan being followed, then we simply forfeit the negotiation.
    return responder->forfeit({});
  }

  rmf_traffic::schedule::StubbornNegotiator(data->itinerary)
      .respond(table_viewer, responder);
}

//==============================================================================s
TrafficLight::UpdateHandle::Implementation::Implementation(
    std::shared_ptr<CommandHandle> command_,
    rmf_traffic::schedule::Participant itinerary_,
    rmf_traffic::agv::VehicleTraits traits_,
    std::shared_ptr<rmf_traffic::schedule::Snappable> schedule_,
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<rclcpp::Node> node_)
  : data(std::make_shared<Data>(
           std::move(command_),
           std::move(itinerary_),
           std::move(traits_),
           std::move(schedule_),
           std::move(worker_),
           std::move(node_)))
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<TrafficLight::UpdateHandle>
TrafficLight::UpdateHandle::Implementation::make(
    std::shared_ptr<CommandHandle> command,
    rmf_traffic::schedule::Participant itinerary,
    rmf_traffic::agv::VehicleTraits traits,
    std::shared_ptr<rmf_traffic::schedule::Snappable> schedule,
    rxcpp::schedulers::worker worker,
    std::shared_ptr<rclcpp::Node> node,
    rmf_traffic_ros2::schedule::Negotiation* negotiation)
{
  std::shared_ptr<UpdateHandle> handle = std::make_shared<UpdateHandle>();
  handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
        std::move(command),
        std::move(itinerary),
        std::move(traits),
        std::move(schedule),
        std::move(worker),
        std::move(node));

  if (negotiation)
  {
    handle->_pimpl->negotiation_license = negotiation->register_negotiator(
          handle->_pimpl->data->itinerary.id(),
          std::make_unique<Negotiator>(handle->_pimpl->data));
  }

  return handle;
}

//==============================================================================
std::size_t TrafficLight::UpdateHandle::follow_new_path(
    const std::vector<Waypoint>& new_path)
{
  const std::size_t version = ++_pimpl->received_version;
  _pimpl->data->worker.schedule(
        [version, new_path, data = _pimpl->data](const auto&)
  {
    data->update_path(version, new_path);
  });

  return version;
}

} // namespace agv
} // namespace rmf_fleet_adapter
