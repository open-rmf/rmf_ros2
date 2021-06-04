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
#include <rmf_traffic/DetectConflict.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class TrafficLight::UpdateHandle::Implementation::Data
    : public std::enable_shared_from_this<Data>
{
public:

  std::shared_ptr<CommandHandle> command;

  rmf_traffic::schedule::Participant itinerary;

  rmf_traffic::blockade::Participant blockade;

  rmf_traffic::agv::VehicleTraits traits;

  std::shared_ptr<rmf_traffic::schedule::Snappable> schedule;

  rxcpp::schedulers::worker worker;

  std::shared_ptr<Node> node;

  std::optional<rclcpp::Time> last_immediate_stop;

  rmf_traffic::blockade::ReservedRange current_range =
      rmf_traffic::blockade::ReservedRange{0, 0};
  std::size_t current_path_version = 0;
  std::size_t current_plan_version = 0;

  std::shared_ptr<rmf_traffic::Profile> profile;

  std::vector<Waypoint> path;

  std::shared_ptr<rmf_traffic::agv::Planner> planner;

  std::map<std::size_t, rmf_traffic::Time> arrival_timing;
  std::map<std::size_t, rclcpp::Time> departure_timing;
  std::map<std::size_t, std::size_t> pending_waypoint_index;

  std::vector<rmf_traffic::Route> stashed_itinerary;
  std::vector<rmf_traffic::Route> active_itinerary;
  std::vector<rmf_traffic::Route> plan_itinerary;
  std::vector<rmf_traffic::agv::Plan::Waypoint> pending_waypoints;

  struct Location
  {
    std::string map;
    Eigen::Vector3d position;
  };

  // Remember which checkpoint we need to depart from next
  std::optional<std::size_t> last_departed_waypoint;
  std::optional<Location> last_known_location;
  bool awaiting_confirmation = false;
  bool rejected = false;
  bool approved = false;
  std::size_t next_departure_checkpoint = 0;

  std::size_t processing_version = 0;

  // The current state of charge of the battery
  float current_battery_soc = -1.0;

  // TODO(MXG): Make this configurable
  rmf_traffic::Duration ready_timing_threshold = std::chrono::seconds(20);

  std::shared_ptr<services::FindPath> find_path_service;
  rxcpp::subscription find_path_subscription;

  /// A timer that periodically checks if the next checkpoint is ready, when
  /// we're sitting and waiting for the departure time of our current checkpoint
  rclcpp::TimerBase::SharedPtr ready_check_timer;

  /// A timer that periodically checks if we need to delay our scheduled
  /// trajectory while we wait for the blockade manager to give us permission
  /// to depart from a checkpoint
  rclcpp::TimerBase::SharedPtr waiting_timer;

  rclcpp::Publisher<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_state_pub;
  rclcpp::TimerBase::SharedPtr fleet_state_timer;

  struct NegotiateManagers
  {
    rmf_rxcpp::subscription_guard subscription;
    rclcpp::TimerBase::SharedPtr timer;
  };
  using NegotiatePtr = std::shared_ptr<services::Negotiate>;
  using NegotiateServiceMap =
      std::unordered_map<NegotiatePtr, NegotiateManagers>;
  NegotiateServiceMap negotiate_services;

  /// Completely change the path that this participant is following
  void update_path(
      std::size_t version,
      const std::vector<Waypoint>& new_path);

  /// Calculate the preferred timing of the participant
  void plan_timing(
    rmf_traffic::agv::Plan::Start start,
    std::size_t version,
    std::vector<Waypoint> new_path,
    std::shared_ptr<rmf_traffic::agv::Planner> new_planner,
    std::function<void()> approval_cb);

  rmf_utils::optional<rmf_traffic::schedule::ItineraryVersion> update_timing(
      std::size_t version,
      std::vector<Waypoint> new_path,
      rmf_traffic::agv::Plan new_plan,
      std::shared_ptr<rmf_traffic::agv::Planner> new_planner);

  bool valid_plan(
      std::size_t version,
      const rmf_traffic::agv::Plan& new_plan);

  void update_location(
      std::size_t path_version,
      std::size_t plan_version,
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      Eigen::Vector3d location,
      std::size_t checkpoint_index);

  void update_stopped_location(
      std::size_t version,
      std::size_t target_checkpoint,
      Eigen::Vector3d location,
      Eigen::Vector3d expected_location);

  void new_range(
      const rmf_traffic::blockade::ReservationId reservation_id,
      const rmf_traffic::blockade::ReservedRange& new_range);

  void send_checkpoints(
      std::function<void()> approval_callback,
      CommandHandle::Reject rejection_callback);

  void watch_for_ready(
      std::size_t path_version,
      std::size_t checkpoint_id,
      bool assume_reached_checkpoint);

  bool check_if_ready(
      std::size_t path_version,
      std::size_t checkpoint_id);

  void check_waiting_delay(
      std::size_t path_version,
      std::size_t checkpoint_id);

  bool check_if_finished(
      std::size_t reached_checkpoint_id);

  void approve(
      std::size_t path_version,
      std::size_t plan_version);

  void reject(
      std::size_t path_version,
      std::size_t plan_version,
      std::size_t actual_last_departed,
      Eigen::Vector3d stopped_location);

  rmf_traffic::agv::Plan::Start current_location() const;

  const std::string& name() const
  {
    return itinerary.description().name();
  }

  void publish_fleet_state() const;

  Data(
      std::shared_ptr<CommandHandle> command_,
      rmf_traffic::blockade::Participant blockade_,
      rmf_traffic::schedule::Participant itinerary_,
      rmf_traffic::agv::VehicleTraits traits_,
      std::shared_ptr<rmf_traffic::schedule::Snappable> schedule_,
      rxcpp::schedulers::worker worker_,
      std::shared_ptr<Node> node_)
    : command(std::move(command_)),
      itinerary(std::move(itinerary_)),
      blockade(std::move(blockade_)),
      traits(std::move(traits_)),
      schedule(std::move(schedule_)),
      worker(std::move(worker_)),
      node(std::move(node_)),
      profile(std::make_shared<rmf_traffic::Profile>(
                itinerary.description().profile()))
  {
    fleet_state_pub = node->fleet_state();
    fleet_state_timer = node->create_wall_timer(
          std::chrono::seconds(1),
          [this]()
    {
      this->publish_fleet_state();
    });
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

  const auto now = rmf_traffic_ros2::convert(node->now());

  plan_itinerary.clear();
  pending_waypoints.clear();
  itinerary.clear();
  ready_check_timer = nullptr;
  waiting_timer = nullptr;
  last_departed_waypoint.reset();

  if (new_path.empty())
  {
    // Clear out this robot's itinerary because it is done moving
    RCLCPP_INFO(
      node->get_logger(),
      "Traffic light controlled robot [%s] owned by [%s] is being taken off "
      "the schedule because it is idle.",
      itinerary.description().name().c_str(),
      itinerary.description().owner().c_str());

    planner = nullptr;
    path.clear();
    departure_timing.clear();
    blockade.cancel();
    return;
  }
  else if (new_path.size() == 1)
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Traffic light controlled robot [%s] owned by [%s] was given only "
          "one waypoint. The traffic light controller requires the robot to "
          "start and end at separate locations which are both designated safe "
          "zones for the robot.",
          itinerary.description().name().c_str(),
          itinerary.description().owner().c_str());
    assert(false);
    return;
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
        "[%ld, %ld] that are too close together [%fm]",
        itinerary.description().name().c_str(),
        itinerary.description().owner().c_str(),
        i-1,
        i,
        dist);
      assert(false);
      return;
    }
  }

  std::vector<rmf_traffic::blockade::Writer::Checkpoint> checkpoints;

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

    checkpoints.push_back(
        {wp.position().block<2,1>(0,0), wp.map_name(), wp.yield()});
  }

  auto new_planner = std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Plan::Configuration(graph, traits),
        rmf_traffic::agv::Plan::Options(nullptr));

  last_known_location = Location{
    new_planner->get_configuration().graph().get_waypoint(0).get_map_name(),
    new_path.front().position()
  };

  rmf_traffic::agv::Plan::Start start{now, 0, new_path.front().position()[2]};

  auto approval_cb =
      [w = weak_from_this(),
       checkpoints = std::move(checkpoints)]()
  {
    if (const auto data = w.lock())
    {
      data->current_range = rmf_traffic::blockade::ReservedRange{0, 0};
      data->blockade.set(checkpoints);
    }
  };

  plan_timing(
        std::move(start), version, new_path,
        std::move(new_planner), std::move(approval_cb));
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::plan_timing(
    rmf_traffic::agv::Plan::Start start,
    const std::size_t version,
    std::vector<Waypoint> new_path,
    std::shared_ptr<rmf_traffic::agv::Planner> new_planner,
    std::function<void()> approval_cb)
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
         new_planner,
         approval_cb = std::move(approval_cb)](
        const services::FindPath::Result& result)
  {
    const auto data = w.lock();
    if (!data)
      return;

    if (version != data->processing_version)
      return;

    if (!result)
    {
      RCLCPP_ERROR(
        data->node->get_logger(),
        "Failed to find any itinerary for submitted path #%ld of robot [%s] "
        "owned by [%s]. This is a critical bug and should be reported to the "
        "rmf_core developers.",
        version,
        data->itinerary.description().name().c_str(),
        data->itinerary.description().owner().c_str());

      return;
    }

    data->current_path_version = version;
    approval_cb();

    assert(!result->get_waypoints().empty());

    data->update_timing(
      version,
      std::move(new_path),
      std::move(*result),
      std::move(new_planner));
  });
}

namespace {

//==============================================================================
std::optional<rmf_traffic::Time> linear_interpolate_time(
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
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  const Eigen::Vector3d& location)
{
  const Eigen::Vector2d p = location.block<2,1>(0,0);

  assert(waypoints.size() > 1);

  const std::size_t N = waypoints.size();
  for (std::size_t i=1; i < waypoints.size(); ++i)
  {
    const auto index = N - i;
    const auto& wp0 = waypoints.at(index-1);
    const auto& wp1 = waypoints.at(index);

    const auto gi_0 = wp0.graph_index();
    const auto gi_1 = wp1.graph_index();

    if (gi_1.has_value() && (!gi_0.has_value() || (*gi_0 != *gi_1)))
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

      if (traversal < 0.0)
      {
        // If the vehicle is behind the lane then we should project the lane
        // backwards to meet it
        const double s = traversal/traits.linear().get_nominal_velocity();
        return wp0.time() + rmf_traffic::time::from_seconds(s);
      }

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
void update_itineraries(
    rmf_traffic::schedule::Participant& scheduled_itinerary,
    std::vector<rmf_traffic::Route>& stashed_itinerary,
    std::vector<rmf_traffic::Route>& active_itinerary,
    const std::vector<rmf_traffic::Route>& plan_itinerary)
{
  // TODO(MXG): Revisit the way we keep the schedule up to date, since we can
  // probably do better.
  for (const auto& r : active_itinerary)
    stashed_itinerary.push_back(r);
  active_itinerary.clear();

  const auto cumulative_delay = scheduled_itinerary.delay();
  for (auto& r : stashed_itinerary)
  {
    assert(!r.trajectory().empty());
    if (!r.trajectory().empty())
      r.trajectory().front().adjust_times(cumulative_delay);
  }

  std::vector<rmf_traffic::Route> full_itinerary;
  full_itinerary.reserve(stashed_itinerary.size() + active_itinerary.size());

  full_itinerary.insert(
        full_itinerary.end(),
        stashed_itinerary.begin(),
        stashed_itinerary.end());

  full_itinerary.insert(
        full_itinerary.end(),
        plan_itinerary.begin(),
        plan_itinerary.end());

  scheduled_itinerary.set(std::move(full_itinerary));
}

//==============================================================================
void update_active_itinerary(
    std::vector<rmf_traffic::Route>& active_itinerary,
    const std::vector<rmf_traffic::Route>& plan_itinerary,
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& pending_waypoints,
    const std::vector<rmf_traffic::agv::Plan::Waypoint>::const_iterator& end_plan_it)
{
  for (auto it = pending_waypoints.begin(); it != end_plan_it; ++it)
  {
    const auto& wp = *it;

    const std::size_t initial_i = active_itinerary.empty()?
          0 : active_itinerary.size()-1;

    for (auto i=initial_i; i <= wp.itinerary_index(); ++i)
    {
      const auto& route = plan_itinerary.at(i);
      const auto& planned = route.trajectory();

      if (active_itinerary.size() <= i)
        active_itinerary.push_back({route.map(), {}});

      auto& active = active_itinerary.at(i).trajectory();

      const auto begin_it = [&]()
      {
        if (active.empty())
          return planned.begin();

        return ++planned.find(active.back().time());
      }();

      const auto* last_wp = [&]() -> const rmf_traffic::Trajectory::Waypoint*
      {
        if (i == wp.itinerary_index())
          return &planned[wp.trajectory_index()];

        return &planned.back();
      }();

      for (auto it = begin_it; &(*it) != last_wp; ++it)
        active.insert(*it);
    }
  }
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
  if (version != current_path_version)
    return rmf_utils::nullopt;

  // TODO(MXG): Is this really the best place to set the path member variable?
  // Maybe it should be set earlier in the chain.
  path = std::move(path_);
  assert(!path.empty());

  plan_itinerary = plan_.get_itinerary();
  planner = std::move(planner_);

  pending_waypoints = plan_.get_waypoints();
  approved = false;

  ++current_plan_version;

  assert(!pending_waypoints.empty());
  const rclcpp::Time initial_t =
      rmf_traffic_ros2::convert(pending_waypoints.front().time());

  const auto& graph = planner->get_configuration().graph();

  std::optional<std::size_t> first_wp;
  std::optional<std::size_t> current_wp;
  std::optional<rmf_traffic::Time> immediately_stop_until;
  std::optional<Eigen::Vector3d> expected_start;

  for (std::size_t i=0; i < pending_waypoints.size(); ++i)
  {
    const auto& wp = pending_waypoints[i];

    if (!wp.graph_index().has_value())
    {
      immediately_stop_until = wp.time();
      expected_start = wp.position();
      continue;
    }

    if (!current_wp)
    {
      next_departure_checkpoint = *wp.graph_index();
      first_wp = *wp.graph_index();
      current_wp = *wp.graph_index();
    }

    if (*wp.graph_index() != *current_wp)
    {
      const std::size_t next_wp = *wp.graph_index();
      for (std::size_t k=*current_wp+1; k < next_wp; ++k)
      {
        // For any intermediate waypoints that will be passed over without
        // stopping, we should try to calculate the time that the robot is
        // expected to pass over it.
        const auto p = graph.get_waypoint(k).get_location();
        const auto t = interpolate_time(traits, pending_waypoints[i-1], wp, p);
        arrival_timing[k] = t;
        departure_timing[k] = rmf_traffic_ros2::convert(t);
        pending_waypoint_index[k] = i;
      }

      current_wp = *wp.graph_index();
      arrival_timing[*current_wp] = wp.time();
    }

    // This might get overriden several times since a single path waypoint might
    // show up several times in a plan. We want to use the last time that is
    // associated with this path waypoint.
    departure_timing[*current_wp] = rmf_traffic_ros2::convert(wp.time());
    pending_waypoint_index[*current_wp] = i;
  }

  const auto now = node->now();

  bool resend_checkpoints = (first_wp.value() < current_range.end) || rejected;

  // Now that we have accounted for any lingering rejection, we can reset this
  // flag.
  rejected = false;

  if (!resend_checkpoints)
  {
    for (const auto& [checkpoint, ready_time] : departure_timing)
    {
      if (current_range.end <= checkpoint)
        break;

      // If we have significantly pushed back our departure times, then we
      // should resend the checkpoints with a shorter range.
      if (now + 5*ready_timing_threshold <= ready_time)
      {
        resend_checkpoints = true;
        current_range.end = checkpoint;
        break;
      }
    }
  }

  awaiting_confirmation = true;

  if (immediately_stop_until.has_value())
  {
    // An immediate stop will invalidate these earlier trajectories
    stashed_itinerary.clear();
    active_itinerary.clear();

    // It would be very strange if this happened, but let's try to be robust
    // to the possibility that the robot has diverged from its original path
    // and is returning to the zeroth waypoint.
    const std::size_t resume_target =
        first_wp.value() == 0 ? 1 : first_wp.value();

    std::vector<rmf_traffic::agv::Plan::Waypoint> resume_waypoints;
    auto end_resume_it = pending_waypoints.begin();
    for (; end_resume_it != pending_waypoints.end(); ++end_resume_it)
    {
      if (end_resume_it->graph_index().has_value())
      {
        resume_waypoints.push_back(*end_resume_it);
        break;
      }

      resume_waypoints.push_back(*end_resume_it);
    }

    update_active_itinerary(
          active_itinerary, plan_itinerary, pending_waypoints, end_resume_it);

    pending_waypoints.erase(pending_waypoints.begin(), end_resume_it);

    assert(!pending_waypoints.empty());
    assert(pending_waypoints.front().graph_index().has_value());

    auto stopped_at =
        [w = weak_from_this(),
         path_version = current_path_version,
         plan_version = current_plan_version,
         target = next_departure_checkpoint,
         expected_location = expected_start.value()](
        Eigen::Vector3d location)
    {
      if (const auto data = w.lock())
      {
        data->approve(path_version, plan_version);
        data->update_stopped_location(
              path_version, target, location, expected_location);
      }
    };

    auto departed =
        [w = weak_from_this(),
         path_version = current_path_version,
         plan_version = current_plan_version,
         checkpoint_index = resume_target-1,
         waypoints = std::move(resume_waypoints)](
        Eigen::Vector3d location)
    {
      if (const auto data = w.lock())
      {
        data->approve(path_version, plan_version);
        data->update_location(
              path_version, plan_version,
              waypoints, location, checkpoint_index);
      }
    };

    assert(expected_start.has_value());

    last_immediate_stop =
        rmf_traffic_ros2::convert(immediately_stop_until.value());

    command->immediately_stop_until(
          current_path_version,
          last_immediate_stop.value(),
          std::move(stopped_at), std::move(departed));
  }
  else if (last_immediate_stop.has_value())
  {
    if (now < *last_immediate_stop)
      command->resume(current_path_version);
  }

  if (!immediately_stop_until.has_value())
    last_immediate_stop.reset();

  update_itineraries(
        itinerary, stashed_itinerary, active_itinerary, plan_itinerary);

  if (resend_checkpoints)
  {
    auto approval_cb =
        [w = weak_from_this(),
         path_version = current_path_version,
         plan_version = current_plan_version]()
    {
      if (const auto data = w.lock())
        data->approve(path_version, plan_version);
    };

    auto reject_cb =
        [w = weak_from_this(),
         path_version = current_path_version,
         plan_version = current_plan_version](
        const std::size_t last_departed,
        Eigen::Vector3d stopped_location)
    {
      if (const auto data = w.lock())
      {
        data->reject(
              path_version, plan_version,
              last_departed, stopped_location);
      }
    };

    send_checkpoints(std::move(approval_cb), std::move(reject_cb));
  }
  else
  {
    // If we are not resending checkpoints and we are not requesting an
    // immediate stop, then we do not need to wait for any confirmation.
    if (!immediately_stop_until.has_value())
      awaiting_confirmation = false;

    watch_for_ready(version, next_departure_checkpoint, false);
  }

  return itinerary.version();
}

//==============================================================================
bool TrafficLight::UpdateHandle::Implementation::Data::valid_plan(
    std::size_t version,
    const rmf_traffic::agv::Plan& new_plan)
{
  if (version != current_path_version)
    return false;

  const auto now = rmf_traffic_ros2::convert(node->now());

  for (const auto& wp : new_plan.get_waypoints())
  {
    if (!wp.graph_index().has_value())
      continue;

    if (wp.graph_index().value() <= last_departed_waypoint)
    {
      // If we have already departed from this waypoint but we're supposed to
      // wait at it, then this plan is invalid.
      if (now < wp.time())
        return false;
    }
    else
    {
      break;
    }
  }

  return true;
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::update_location(
    const std::size_t path_version,
    const std::size_t plan_version,
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    const Eigen::Vector3d location,
    const std::size_t checkpoint_index)
{
  const auto now = rmf_traffic_ros2::convert(node->now());

  worker.schedule(
        [w = weak_from_this(),
         path_version,
         waypoints,
         location,
         now,
         checkpoint_index](const auto&)
  {
    const auto data = w.lock();
    if (!data)
      return;

    if (path_version != data->current_path_version)
      return;

    if (data->last_departed_waypoint.has_value())
    {
      data->last_departed_waypoint =
          std::max(checkpoint_index, data->last_departed_waypoint.value());
    }
    else
    {
      data->last_departed_waypoint = checkpoint_index;
    }

    data->last_known_location = Location{
        data->planner->get_configuration().graph()
          .get_waypoint(checkpoint_index).get_map_name(),
        location
    };

    data->blockade.reached(checkpoint_index);

    if (data->awaiting_confirmation)
    {
      if (checkpoint_index < data->current_range.end)
      {
        data->approve(data->current_path_version, data->current_plan_version);
      }
      else
      {
        data->reject(
          data->current_path_version,
          data->current_plan_version,
          checkpoint_index,
          location);
      }
    }

    assert(checkpoint_index < data->arrival_timing.size());

    const auto expected_time = interpolate_time(
          now, data->traits, waypoints, location);

    if (expected_time)
    {
      const auto new_delay = now - *expected_time;
      const auto time_shift = new_delay - data->itinerary.delay();
      const auto threshold = std::chrono::seconds(1);
      if (time_shift < -threshold || threshold < time_shift)
      {
        data->itinerary.delay(time_shift);
      }

      // Check whether the next waypoint is ready based on the current timing
      // predictions.
      data->check_if_ready(path_version, checkpoint_index+1);

      return;
    }

    RCLCPP_WARN(
      data->node->get_logger(),
      "Failed to compute timing estimate for [%s] owned by [%s] "
      "moving away from checkpoint index [%ld].",
      data->itinerary.description().name().c_str(),
      data->itinerary.description().owner().c_str(),
      checkpoint_index);
  });
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::update_stopped_location(
    std::size_t version,
    std::size_t target_checkpoint,
    Eigen::Vector3d location,
    Eigen::Vector3d expected_location)
{
  const auto now = rmf_traffic_ros2::convert(node->now());

  worker.schedule(
        [w = weak_from_this(),
         version,
         target_checkpoint,
         location,
         expected_location,
         now](
        const auto&)
  {
    const auto data = w.lock();
    if (!data)
      return;

    if (version != data->current_path_version)
      return;

    if ( (location - expected_location).norm() > 0.5)
    {
      // The stopped location is far from where we intended the participant to
      // stop, so we should recompute the plan.
      rmf_traffic::agv::Plan::Start start(
          now, target_checkpoint, location[2], location.block<2,1>(0,0));

      if (target_checkpoint > 0)
      {
        const auto* lane = data->planner->get_configuration().graph().lane_from(
              target_checkpoint-1, target_checkpoint);
        assert(lane);
        start.lane(lane->index());
      }

      data->plan_timing(
            std::move(start), version, data->path, data->planner, [](){});
    }
  });
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::new_range(
    const rmf_traffic::blockade::ReservationId reservation_id,
    const rmf_traffic::blockade::ReservedRange& new_range)
{
  // TODO(MXG): All this logic should be embedded in the Participant class which
  // should also accept a callback for receiving new ranges.
  if (reservation_id != blockade.reservation_id())
    return;

  if (new_range == current_range)
    return;

  // If we have already reached the last point, then we no longer need to do
  // anything with these ranges.
  if (blockade.last_reached() == path.size()-1)
    return;

  // If we are awaiting confirmation, we will ignore any new ranges.
  // - If we get an approval, then we will be releasing checkpoints, which will
  // trigger a new range.
  // - If we get a rejection, then we will be finding a new plan, which will
  // generate new checkpoints.
  if (awaiting_confirmation)
    return;

  const auto last_ready = blockade.last_ready();

  // If we don't have any ready points yet, then a range that ends past 0 should
  // not be believed. We must have readied some points earlier but released them
  // before the moderator published this range.
  if (!last_ready.has_value() && new_range.end > 0)
    return;

  if (last_ready.has_value())
  {
    // If our last ready value is less than one below the end of the range, then
    // we must have released some readied points before the moderator published
    // this range.
    if (last_ready.value()+1 < new_range.end)
      return;
  }

  current_range = new_range;

  auto reject =
      [w = weak_from_this(),
       path_version = current_path_version,
       plan_version = current_plan_version](
      const std::size_t last_departed,
      Eigen::Vector3d stopped_location)
  {
    if (const auto data = w.lock())
    {
      RCLCPP_WARN(
        data->node->get_logger(),
        "[rmf_fleet_adapter::TrafficLight::UpdateHandle] A checkpoint "
        "rejection was issued in a situation where a rejection should not be "
        "possible. This indicates an integration error with the traffic light "
        "fleet adapter. Fleet name: [%s], Robot name: [%s], last departed "
        "checkpoint: %lu, stopped location: <%.2f, %.2f, %.2f>",
        data->itinerary.description().owner().c_str(),
        data->itinerary.description().name().c_str(),
        last_departed,
        stopped_location.x(),
        stopped_location.y(),
        stopped_location.z());

      data->reject(path_version, plan_version, last_departed, stopped_location);
    }
  };

  send_checkpoints(
        [](){}, // No approval needed
        std::move(reject));
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::send_checkpoints(
    std::function<void()> approval_callback,
    CommandHandle::Reject rejection_callback)
{
  using Checkpoint = TrafficLight::CommandHandle::Checkpoint;

  const auto in_range_inclusive = [&](
      const std::size_t plan_index,
      const std::size_t end_path_index) -> bool
  {
    if (pending_waypoints.size() <= plan_index)
      return false;

    return pending_waypoints.at(plan_index)
        .graph_index().value() <= end_path_index;
  };

  const auto in_range_exclusive = [&](
      const std::size_t plan_index,
      const std::size_t end_path_index) -> bool
  {
    if (pending_waypoints.size() <= plan_index)
      return false;

    return pending_waypoints.at(plan_index)
        .graph_index().value() < end_path_index;
  };

  std::vector<Checkpoint> checkpoints;
  std::size_t next = 0;
  std::size_t current_checkpoint_index = next_departure_checkpoint;

  // If a plan was changed, resulting in a smaller range, we want to maintain
  // the larger of these two values as the next_departure_checkpoint
  next_departure_checkpoint =
      std::max(current_range.end, next_departure_checkpoint);

  while (in_range_inclusive(next, current_checkpoint_index))
    ++next;
  --next;

  while (in_range_exclusive(next, current_range.end))
  {
    const std::size_t next_checkpoint_index = current_checkpoint_index + 1;

    std::size_t end_plan_segment_index = next;
    while (in_range_inclusive(end_plan_segment_index, next_checkpoint_index))
      ++end_plan_segment_index;

    assert(end_plan_segment_index > next);
    if (end_plan_segment_index < pending_waypoints.size())
    {
      const std::size_t check_index =
          pending_waypoints.at(end_plan_segment_index-1).graph_index().value();

      if (check_index <= current_checkpoint_index)
      {
        // We need to make sure that the last plan waypoint we pass along
        // belongs to the next path checkpoint, otherwise the interpolator won't
        // have enough information to infer what the current delay is.
        ++end_plan_segment_index;
      }
    }

    std::vector<rmf_traffic::agv::Plan::Waypoint> departed_waypoints(
      pending_waypoints.begin() + next,
      pending_waypoints.begin() + end_plan_segment_index);

    assert(!departed_waypoints.empty());

    const std::size_t last_checkpoint_index =
        std::min(
          departed_waypoints.back().graph_index().value(),
          current_range.end);

    for (std::size_t c = current_checkpoint_index;
         c < last_checkpoint_index; ++c)
    {
      auto departed =
          [w = weak_from_this(),
           approval_callback = approval_callback,
           path_version = current_path_version,
           plan_version = current_plan_version,
           checkpoint_index = c,
           waypoints = departed_waypoints](Eigen::Vector3d location)
      {
        approval_callback();

        if (const auto data = w.lock())
        {
          data->update_location(
                path_version, plan_version,
                waypoints, location, checkpoint_index);
        }
      };

      checkpoints.emplace_back(
            CommandHandle::Checkpoint{
              c,
              departure_timing.at(current_checkpoint_index),
              std::move(departed)
            });
    }

    next = end_plan_segment_index-1;

    if (next < pending_waypoints.size())
    {
      current_checkpoint_index =
          pending_waypoints.at(next).graph_index().value();
    }
  }

  if (checkpoints.empty())
  {
    auto on_standby =
        [w = weak_from_this(),
         approval_callback = approval_callback,
         path_version = current_path_version,
         standby_checkpoint = current_range.end]()
    {
      approval_callback();

      if (const auto data = w.lock())
      {
        data->worker.schedule(
              [w = data->weak_from_this(),
               path_version,
               standby_checkpoint](const auto&)
        {
          if (const auto data = w.lock())
            data->watch_for_ready(path_version, standby_checkpoint, true);
        });
      }
    };

    return;
  }

  const auto end_plan_it = [&]()
      -> std::vector<rmf_traffic::agv::Plan::Waypoint>::const_iterator
  {
    auto it = ++pending_waypoints.begin();
    while (it != pending_waypoints.end())
    {
      const auto check = it++;
      if (check->graph_index().value() >= current_range.end)
        return check-1;
    }

    return it;
  }();

  update_active_itinerary(
        active_itinerary, plan_itinerary, pending_waypoints, end_plan_it);

  pending_waypoints.erase(pending_waypoints.begin(), end_plan_it);

  // Cancel this timer that is watching to see if we need to delay our schedule
  // while waiting to depart
  waiting_timer = nullptr;

  auto on_standby =
      [w = weak_from_this(),
       approval_callback = approval_callback,
       path_version = current_path_version,
       standby_checkpoint = current_range.end]()
  {
    approval_callback();

    if (const auto data = w.lock())
    {
     data->worker.schedule(
           [w = data->weak_from_this(),
            path_version,
            standby_checkpoint](const auto&)
     {
       if (const auto data = w.lock())
         data->watch_for_ready(path_version, standby_checkpoint, true);
     });
    }
  };

  if (current_range.begin == current_range.end)
  {
    // We should just clear out the checkpoints since the only thing that
    // matters in a single-point range is the on_standby callback.
    checkpoints.clear();
  }

  command->receive_checkpoints(
        current_path_version,
        std::move(checkpoints),
        current_range.end,
        std::move(on_standby),
        std::move(rejection_callback));
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::watch_for_ready(
    const std::size_t path_version,
    const std::size_t checkpoint_id,
    const bool assume_reached_checkpoint)
{
  if (path_version != current_path_version)
    return;

  if (assume_reached_checkpoint)
  {
    blockade.reached(checkpoint_id);
  }

  if (check_if_finished(checkpoint_id))
    return;

  waiting_timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [w = weak_from_this(), path_version, checkpoint_id]()
  {
    if (const auto data = w.lock())
      data->check_waiting_delay(path_version, checkpoint_id);
  });

  if (check_if_ready(path_version, checkpoint_id))
    return;

  ready_check_timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [w = weak_from_this(), path_version, checkpoint_id]()
  {
    if (const auto data = w.lock())
    {
      if (data->check_if_ready(path_version, checkpoint_id))
        data->ready_check_timer = nullptr;
    }
  });
}

//==============================================================================
bool TrafficLight::UpdateHandle::Implementation::Data::check_if_ready(
    std::size_t version,
    std::size_t checkpoint_id)
{
  // Counter-intuitively, we return true for these sanity check cases because we
  // want the caller to quit right away.
  if (version != current_path_version)
    return true;

  if (pending_waypoints.empty())
    return true;

  const auto depart_it = departure_timing.find(checkpoint_id);
  if (depart_it == departure_timing.end())
    return true;

  // If we are waiting for an approval or a rejection, then we should not
  // ready-up any more waypoints.
  if (awaiting_confirmation)
  {
    if (checkpoint_id <= current_range.end)
    {
      approve(current_path_version, current_plan_version);
    }
    else
    {
      reject(current_path_version, current_plan_version,
             checkpoint_id-1, last_known_location.value().position);
    }

    return false;
  }

  const auto ready_time = depart_it->second;
  const auto now = node->now() - itinerary.delay();

  if (now + ready_timing_threshold <= ready_time)
    return false;

  for (auto it = depart_it; it != departure_timing.end(); ++it)
  {
    // TODO(MXG): It would probably be better to reverse iterate here
    if (it->second <= now + ready_timing_threshold)
      blockade.ready(it->first);
  }

  return true;
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::check_waiting_delay(
    const std::size_t version,
    const std::size_t checkpoint_id)
{
  if (version != current_path_version)
    return;

  const auto depart_it = departure_timing.find(checkpoint_id);
  if (depart_it == departure_timing.end())
    return;

  const auto now = node->now();
  const auto new_delay = now - depart_it->second;
  const auto time_shift = (new_delay - itinerary.delay())
      .to_chrono<std::chrono::nanoseconds>();

  // We add some extra margin in here, because we don't know exactly when
  // we will get permission to continue.
  if (time_shift > -std::chrono::seconds(1))
  {
    const auto chosen_shift = time_shift + std::chrono::seconds(2);
    itinerary.delay(chosen_shift);
  }
}

//==============================================================================
bool TrafficLight::UpdateHandle::Implementation::Data::check_if_finished(
    const std::size_t reached_checkpoint_id)
{
  if (path.size() <= reached_checkpoint_id+1)
  {
    waiting_timer = nullptr;
    ready_check_timer = nullptr;
    itinerary.clear();
    return true;
  }

  return false;
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::approve(
    const std::size_t path_version,
    const std::size_t plan_version)
{
  worker.schedule(
        [w = weak_from_this(),
         path_version,
         plan_version](const auto&)
  {
    const auto data = w.lock();
    if (!data)
      return;

    if (path_version != data->current_path_version)
      return;

    if (plan_version != data->current_plan_version)
      return;

    if (data->approved)
      return;

    data->approved = true;
    data->awaiting_confirmation = false;

    data->blockade.release(data->current_range.end);
  });
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::reject(
    std::size_t path_version,
    std::size_t plan_version,
    std::size_t actual_last_departed,
    Eigen::Vector3d stopped_location)
{
  worker.schedule(
        [w = weak_from_this(),
         path_version,
         plan_version,
         actual_last_departed,
         stopped_location](const auto&)
  {
    const auto data = w.lock();
    if (!data)
      return;

    if (path_version != data->current_path_version)
      return;

    if (data->rejected)
      return;

    // Even if the plan version is wrong, it may still be useful to update this
    // information.
    data->last_departed_waypoint = actual_last_departed;
    data->last_known_location = Location{
        data->planner->get_configuration().graph()
          .get_waypoint(actual_last_departed).get_map_name(),
        stopped_location
    };

    if (plan_version != data->current_plan_version)
      return;

    if (data->check_if_finished(actual_last_departed))
      return;

    // The current plan was rejected, so we'll need to work out a new one
    data->rejected = true;
    data->awaiting_confirmation = false;
    data->plan_timing(
          data->current_location(), path_version, data->path,
          data->planner, [](){});
  });
}

//==============================================================================
rmf_traffic::agv::Plan::Start
TrafficLight::UpdateHandle::Implementation::Data::current_location() const
{
  const auto now = rmf_traffic_ros2::convert(node->now());

  if (last_departed_waypoint.has_value() && last_known_location.has_value())
  {
    const auto* lane = planner->get_configuration().graph().lane_from(
          *last_departed_waypoint, *last_departed_waypoint+1);
    assert(lane);

    const Eigen::Vector3d p = last_known_location->position;

    return rmf_traffic::agv::Plan::Start(
          now,
          *last_departed_waypoint+1,
          p[2],
          p.block<2,1>(0,0),
          lane->index());
  }

  // If either of the above are missing, we have to assume that the robot has
  // not proceeded past its starting waypoint.
  return rmf_traffic::agv::Plan::Start(
        now, 0, path.front().position()[2]);
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data
::publish_fleet_state() const
{
  if (!last_known_location.has_value())
    return;

  auto robot_mode = [&]()
  {
    if (current_range.begin == current_range.end)
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
  }();

  const auto& map = last_known_location->map;
  const auto p = last_known_location->position;
  auto location = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::Location>()
      .t(node->now())
      .x(p.x())
      .y(p.y())
      .yaw(p[2])
      .level_name(map)
      .index(0);

  const auto& fleet_name = itinerary.description().owner();
  auto robot_state = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::RobotState>()
      .name(name())
      .model(fleet_name)
      // TODO(MXG): Have a way to fill this in
      .task_id("")
      // TODO(MXG): We could keep track of the seq value and increment it once
      // with each publication. This is not currently an important feature
      // outside of the fleet driver, so for now we just set it to zero.
      .seq(0)
      .mode(std::move(robot_mode))
      // We multiply by 100 to convert from the [0.0, 1.0] range to percentage
      .battery_percent(current_battery_soc*100.0)
      .location(std::move(location))
      .path({});

  fleet_state_pub->publish(
        rmf_fleet_msgs::build<rmf_fleet_msgs::msg::FleetState>()
        .name(fleet_name)
        .robots({std::move(robot_state)}));
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
  if (!data || !data->planner || data->pending_waypoints.empty())
  {
    // If we no longer have access to the traffic light data or there is no
    // plan being followed, then we simply forfeit the negotiation.
    return responder->forfeit({});
  }

  const std::size_t last_reached = data->blockade.last_reached();
  if (last_reached >= data->path.size())
  {
    // If we have reached the end of the path, we can just submit an empty
    // proposal.
    return responder->submit({});
  }

  auto start = data->current_location();

  rmf_traffic::agv::Plan::Goal goal(
        data->planner->get_configuration().graph().num_waypoints()-1);

  auto approval_cb =
      [w = data->weak_from_this(),
       version = data->current_path_version](
      const rmf_traffic::agv::Plan& plan)
      -> rmf_utils::optional<rmf_traffic::schedule::ItineraryVersion>
  {
    const auto data = w.lock();
    if (!data)
      return std::nullopt;

    if (!data->valid_plan(version, plan))
      return std::nullopt;

    if (!data->planner)
    {
      // The robot has already finished the path that was being negotiatied
      return std::nullopt;
    }

    return data->update_timing(version, data->path, plan, data->planner);
  };

  // TODO(MXG): The management of negotiation services should probably get
  // wrapped in its own class to be shared between this module and the GoToPlace
  // phase implementation.
  services::ProgressEvaluator evaluator;
  if (table_viewer->parent_id())
  {
    const auto& s = table_viewer->sequence();
    assert(s.size() >= 2);
    evaluator.compliant_leeway_base *= s[s.size()-2].version + 1;
  }

  auto negotiate = services::Negotiate::path(
        data->planner, {std::move(start)}, std::move(goal),
        table_viewer, responder, std::move(approval_cb), std::move(evaluator));

  auto negotiate_sub =
      rmf_rxcpp::make_job<services::Negotiate::Result>(negotiate)
      .observe_on(rxcpp::identity_same_worker(data->worker))
      .subscribe(
        [w = data->weak_from_this()](const auto& result)
  {
    if (const auto data = w.lock())
    {
      result.respond();
      data->negotiate_services.erase(result.service);
    }
    else
    {
      result.service->responder()->forfeit({});
    }
  });

  using namespace std::chrono_literals;
  const auto wait_duration = 2s + table_viewer->sequence().back().version * 10s;
  auto negotiate_timer = data->node->create_wall_timer(
        wait_duration,
        [s = negotiate->weak_from_this()]()
  {
    if (const auto service = s.lock())
      service->interrupt();
  });

  data->negotiate_services[negotiate] =
      Data::NegotiateManagers{
        std::move(negotiate_sub),
        std::move(negotiate_timer)
      };
}

//==============================================================================
rmf_traffic::blockade::Participant make_blockade(
    rmf_traffic_ros2::blockade::Writer& writer,
    const rmf_traffic::schedule::Participant& itinerary,
    TrafficLight::UpdateHandle::Implementation* impl)
{
  const double radius = itinerary.description().profile()
      .vicinity()->get_characteristic_length();

  auto new_range_cb =
      [impl](
      const rmf_traffic::blockade::ReservationId reservation,
      const rmf_traffic::blockade::ReservedRange& range)
  {
    impl->new_range(reservation, range);
  };

  return writer.make_participant(
        itinerary.id(), radius, std::move(new_range_cb));
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::new_range(
    const rmf_traffic::blockade::ReservationId reservation_id,
    const rmf_traffic::blockade::ReservedRange& new_range)
{
  assert(data);
  data->new_range(reservation_id, new_range);
}

//==============================================================================
TrafficLight::UpdateHandle::Implementation::Implementation(
    std::shared_ptr<CommandHandle> command_,
    rmf_traffic::schedule::Participant itinerary_,
    std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer,
    rmf_traffic::agv::VehicleTraits traits_,
    std::shared_ptr<rmf_traffic::schedule::Snappable> schedule_,
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<Node> node_)
  : data(std::make_shared<Data>(
           std::move(command_),
           make_blockade(*blockade_writer, itinerary_, this),
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
    std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer,
    rmf_traffic::agv::VehicleTraits traits,
    std::shared_ptr<rmf_traffic::schedule::Snappable> schedule,
    rxcpp::schedulers::worker worker,
    std::shared_ptr<Node> node,
    rmf_traffic_ros2::schedule::Negotiation* negotiation)
{
  std::shared_ptr<UpdateHandle> handle = std::make_shared<UpdateHandle>();
  handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
        std::move(command),
        std::move(itinerary),
        std::move(blockade_writer),
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
  if (new_path.size() < 2)
  {
    throw std::runtime_error(
      "[TrafficLight::follow_new_path] Invalid number of waypoints given ["
      + std::to_string(new_path.size()) + "]. Must be at least 2.");
  }

  const std::size_t version = ++_pimpl->received_version;
  _pimpl->data->worker.schedule(
        [version, new_path, data = _pimpl->data](const auto&)
  {
    data->update_path(version, new_path);
  });

  return version;
}

//==============================================================================
auto TrafficLight::UpdateHandle::update_idle_location(
  std::string map,
  Eigen::Vector3d position) -> UpdateHandle&
{
  const std::size_t version = ++_pimpl->received_version;
  _pimpl->data->worker.schedule(
    [version, map = std::move(map), position, data = _pimpl->data](const auto&)
  {
    if (rmf_utils::modular(version).less_than(data->processing_version))
      return;

    data->last_known_location =
      Implementation::Data::Location{
        map,
        position
      };

    if (!data->path.empty())
      data->update_path(version, {});
  });

  return *this;
}

//==============================================================================
auto TrafficLight::UpdateHandle::update_battery_soc(double battery_soc)
-> UpdateHandle&
{
  _pimpl->data->worker.schedule(
        [battery_soc, data = _pimpl->data](const auto&)
  {
    data->current_battery_soc = battery_soc;
  });

  return *this;
}

//==============================================================================
auto TrafficLight::UpdateHandle::fleet_state_publish_period(
    std::optional<rmf_traffic::Duration> value) -> UpdateHandle&
{
  if (value.has_value())
  {
    _pimpl->data->fleet_state_timer =
        _pimpl->data->node->create_wall_timer(
          value.value(),
          [self = _pimpl->data.get()]()
    {
      self->publish_fleet_state();
    });
  }
  else
  {
    _pimpl->data->fleet_state_timer = nullptr;
  }

  return *this;
}

//==============================================================================
class TrafficLight::CommandHandle::Blocker::Implementation
{
public:
  rmf_traffic::schedule::ParticipantId id;
  rmf_traffic::schedule::ParticipantDescription description;
};

//==============================================================================
rmf_traffic::schedule::ParticipantId
TrafficLight::CommandHandle::Blocker::participant_id() const
{
  return _pimpl->id;
}

//==============================================================================
const rmf_traffic::schedule::ParticipantDescription&
TrafficLight::CommandHandle::Blocker::description() const
{
  return _pimpl->description;
}

//==============================================================================
TrafficLight::CommandHandle::Blocker::Blocker()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
