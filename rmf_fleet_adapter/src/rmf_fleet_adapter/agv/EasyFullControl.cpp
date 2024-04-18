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

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/Transformation.hpp>
#include "internal_EasyFullControl.hpp"
#include "internal_RobotUpdateHandle.hpp"
#include "internal_FleetUpdateHandle.hpp"

// Public rmf_task API headers
#include <rmf_task/Event.hpp>
#include <rmf_task/events/SimpleEventState.hpp>
#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ParkRobotFactory.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_utils/math.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_fleet_adapter/tasks/ParkRobotIndefinitely.hpp>

#include <thread>
#include <yaml-cpp/yaml.h>
#include <iostream>

//==============================================================================
namespace rmf_fleet_adapter {
namespace agv {

using ConsiderRequest = EasyFullControl::ConsiderRequest;

//==============================================================================
class EasyFullControl::RobotState::Implementation
{
public:
  std::string map_name;
  Eigen::Vector3d position;
  double battery_soc;
};

//==============================================================================
EasyFullControl::RobotState::RobotState(
  std::string map_name,
  Eigen::Vector3d position,
  double battery_soc)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{
      std::move(map_name),
      position,
      battery_soc
    }))
{
  // Do nothing
}

//==============================================================================
const std::string& EasyFullControl::RobotState::map() const
{
  return _pimpl->map_name;
}

//==============================================================================
void EasyFullControl::RobotState::set_map(std::string value)
{
  _pimpl->map_name = std::move(value);
}

//==============================================================================
Eigen::Vector3d EasyFullControl::RobotState::position() const
{
  return _pimpl->position;
}

//==============================================================================
void EasyFullControl::RobotState::set_position(Eigen::Vector3d value)
{
  _pimpl->position = value;
}

//==============================================================================
double EasyFullControl::RobotState::battery_state_of_charge() const
{
  return _pimpl->battery_soc;
}

//==============================================================================
void EasyFullControl::RobotState::set_battery_state_of_charge(double value)
{
  _pimpl->battery_soc = value;
}

//==============================================================================
class EasyFullControl::RobotConfiguration::Implementation
{
public:
  std::vector<std::string> compatible_chargers;
  std::optional<bool> responsive_wait;
  std::optional<double> max_merge_waypoint_distance;
  std::optional<double> max_merge_lane_distance;
  std::optional<double> min_lane_length;
};

//==============================================================================
EasyFullControl::RobotConfiguration::RobotConfiguration(
  std::vector<std::string> compatible_chargers,
  std::optional<bool> responsive_wait,
  std::optional<double> max_merge_waypoint_distance,
  std::optional<double> max_merge_lane_distance,
  std::optional<double> min_lane_length)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{
      std::move(compatible_chargers),
      responsive_wait,
      max_merge_waypoint_distance,
      max_merge_lane_distance,
      min_lane_length
    }))
{
  // Do nothing
}

//==============================================================================
const std::vector<std::string>&
EasyFullControl::RobotConfiguration::compatible_chargers() const
{
  return _pimpl->compatible_chargers;
}

//==============================================================================
void EasyFullControl::RobotConfiguration::set_compatible_chargers(
  std::vector<std::string> chargers)
{
  _pimpl->compatible_chargers = std::move(chargers);
}

//==============================================================================
std::optional<bool> EasyFullControl::RobotConfiguration::responsive_wait() const
{
  return _pimpl->responsive_wait;
}

//==============================================================================
void EasyFullControl::RobotConfiguration::set_responsive_wait(
  std::optional<bool> enable)
{
  _pimpl->responsive_wait = enable;
}

//==============================================================================
std::optional<double> EasyFullControl::RobotConfiguration::
max_merge_waypoint_distance() const
{
  return _pimpl->max_merge_waypoint_distance;
}

//==============================================================================
void EasyFullControl::RobotConfiguration::set_max_merge_waypoint_distance(
  std::optional<double> distance)
{
  _pimpl->max_merge_waypoint_distance = distance;
}

//==============================================================================
std::optional<double> EasyFullControl::RobotConfiguration::
max_merge_lane_distance() const
{
  return _pimpl->max_merge_lane_distance;
}

//==============================================================================
void EasyFullControl::RobotConfiguration::set_max_merge_lane_distance(
  std::optional<double> distance)
{
  _pimpl->max_merge_lane_distance = distance;
}

//==============================================================================
std::optional<double> EasyFullControl::RobotConfiguration::min_lane_length()
const
{
  return _pimpl->min_lane_length;
}

//==============================================================================
void EasyFullControl::RobotConfiguration::set_min_lane_length(
  std::optional<double> distance)
{
  _pimpl->min_lane_length = distance;
}

//==============================================================================
class EasyFullControl::RobotCallbacks::Implementation
{
public:
  NavigationRequest navigate;
  StopRequest stop;
  ActionExecutor action_executor;
  LocalizationRequest localize;
};

//==============================================================================
EasyFullControl::RobotCallbacks::RobotCallbacks(
  NavigationRequest navigate,
  StopRequest stop,
  ActionExecutor action_executor)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{
      std::move(navigate),
      std::move(stop),
      std::move(action_executor),
      nullptr
    }))
{
  // Do nothing
}

//==============================================================================
auto EasyFullControl::RobotCallbacks::navigate() const -> NavigationRequest
{
  return _pimpl->navigate;
}

//==============================================================================
auto EasyFullControl::RobotCallbacks::stop() const -> StopRequest
{
  return _pimpl->stop;
}

//==============================================================================
auto EasyFullControl::RobotCallbacks::action_executor() const -> ActionExecutor
{
  return _pimpl->action_executor;
}

//==============================================================================
auto EasyFullControl::RobotCallbacks::with_localization(
  LocalizationRequest localization) -> RobotCallbacks&
{
  _pimpl->localize = std::move(localization);
  return *this;
}

//==============================================================================
auto EasyFullControl::RobotCallbacks::localize() const -> LocalizationRequest
{
  return _pimpl->localize;
}

//==============================================================================
class EasyFullControl::CommandExecution::Implementation::Data
{
public:
  std::vector<std::size_t> waypoints;
  std::vector<std::size_t> lanes;
  std::optional<Eigen::Vector2d> target_location;
  std::optional<double> final_orientation;
  rmf_traffic::Duration planned_wait_time;
  std::optional<ScheduleOverride> schedule_override;
  std::shared_ptr<NavParams> nav_params;
  std::function<void(rmf_traffic::Duration)> arrival_estimator;

  void release_stubbornness()
  {
    if (schedule_override.has_value())
    {
      schedule_override->release_stubbornness();
    }
  }

  void update_location(
    const std::shared_ptr<RobotContext>& context,
    const std::string& map,
    Eigen::Vector3d location)
  {
    if (schedule_override.has_value())
    {
      return schedule_override->overridden_update(
        context,
        map,
        location);
    }

    auto planner = context->planner();
    if (!planner)
    {
      RCLCPP_ERROR(
        context->node()->get_logger(),
        "Planner unavailable for robot [%s], cannot update its location",
        context->requester_id().c_str());
      return;
    }

    const auto& graph = planner->get_configuration().graph();
    const rmf_traffic::agv::LaneClosure* closures =
      context->get_lane_closures();
    std::optional<std::pair<std::size_t, double>> on_waypoint;
    auto p = Eigen::Vector2d(location[0], location[1]);
    const double yaw = location[2];
    for (std::size_t wp : waypoints)
    {
      if (wp >= graph.num_waypoints())
      {
        RCLCPP_ERROR(
          context->node()->get_logger(),
          "Robot [%s] has a command with a waypoint [%lu] that is outside "
          "the range of the graph [%lu]. We will not do a location update.",
          context->requester_id().c_str(),
          wp,
          graph.num_waypoints());
        // Should we also issue a replan command?
        return;
      }

      const auto p_wp = graph.get_waypoint(wp).get_location();
      auto dist = (p - p_wp).norm();
      if (dist <= nav_params->max_merge_waypoint_distance)
      {
        if (!on_waypoint.has_value() || dist < on_waypoint->second)
        {
          on_waypoint = std::make_pair(wp, dist);
        }
      }
    }

    rmf_traffic::agv::Plan::StartSet starts;
    const auto now = rmf_traffic_ros2::convert(context->node()->now());
    if (on_waypoint.has_value())
    {
      const auto wp = on_waypoint->first;
      starts.push_back(rmf_traffic::agv::Plan::Start(now, wp, yaw, p));
      for (std::size_t lane_id : graph.lanes_from(wp))
      {
        if (lane_id >= graph.num_lanes())
        {
          RCLCPP_ERROR(
            context->node()->get_logger(),
            "Nav graph for robot [%s] has an invalid lane ID [%lu] leaving "
            "vertex [%lu], lane ID range is [%lu]. We will not do a location "
            "update.",
            context->requester_id().c_str(),
            lane_id,
            wp,
            graph.num_lanes());
          // Should we also issue a replan command?
          return;
        }

        if (closures && closures->is_closed(lane_id))
        {
          // Don't use a lane that's closed
          continue;
        }

        auto wp_exit = graph.get_lane(lane_id).exit().waypoint_index();
        starts.push_back(
          rmf_traffic::agv::Plan::Start(now, wp_exit, yaw, p, lane_id));
      }
    }
    else
    {
      std::optional<std::pair<std::size_t, double>> on_lane;
      for (auto lane_id : lanes)
      {
        if (lane_id >= graph.num_lanes())
        {
          RCLCPP_ERROR(
            context->node()->get_logger(),
            "Robot [%s] has a command with a lane [%lu] that is outside the "
            "range of the graph [%lu]. We will not do a location update.",
            context->requester_id().c_str(),
            lane_id,
            graph.num_lanes());
          // Should we also issue a replan command?
          return;
        }

        if (closures && closures->is_closed(lane_id))
        {
          continue;
        }

        const auto& lane = graph.get_lane(lane_id);
        const auto p0 =
          graph.get_waypoint(lane.entry().waypoint_index()).get_location();
        const auto p1 =
          graph.get_waypoint(lane.exit().waypoint_index()).get_location();
        const auto lane_length = (p1 - p0).norm();
        double dist_to_lane = 0.0;
        if (lane_length < nav_params->min_lane_length)
        {
          dist_to_lane = std::min(
            (p - p0).norm(),
            (p - p1).norm());
        }
        else
        {
          const auto lane_u = (p1 - p0)/lane_length;
          const auto proj = (p - p0).dot(lane_u);
          if (proj < 0.0 || lane_length < proj)
          {
            continue;
          }
          dist_to_lane = (p - p0 - proj * lane_u).norm();
        }

        if (dist_to_lane <= nav_params->max_merge_lane_distance)
        {
          if (!on_lane.has_value() || dist_to_lane < on_lane->second)
          {
            on_lane = std::make_pair(lane_id, dist_to_lane);
          }
        }
      }

      if (on_lane.has_value())
      {
        const auto lane_id = on_lane->first;
        const auto& lane = graph.get_lane(lane_id);
        const auto wp0 = lane.entry().waypoint_index();
        const auto wp1 = lane.exit().waypoint_index();
        starts.push_back(
          rmf_traffic::agv::Plan::Start(now, wp1, yaw, p, lane_id));

        if (const auto* reverse_lane = graph.lane_from(wp1, wp0))
        {
          starts.push_back(rmf_traffic::agv::Plan::Start(
              now, wp0, yaw, p, reverse_lane->index()));
        }
      }
    }

    if (starts.empty())
    {
      starts = nav_params->compute_plan_starts(graph, map, location, now);
    }

    if (!starts.empty())
    {
      if (context->debug_positions)
      {
        std::stringstream ss;
        ss << __FILE__ << "|" << __LINE__ << ": " << starts.size()
           << " starts:" << print_starts(starts, graph);
        std::cout << ss.str() << std::endl;
      }
      context->set_location(starts);
    }
    else
    {
      if (context->debug_positions)
      {
        std::stringstream ss;
        ss << __FILE__ << "|" << __LINE__ << ": setting robot to LOST | "
           << map << " <" << location.block<2, 1>(0, 0).transpose()
           << "> orientation " << location[2] * 180.0 / M_PI << "\n";
        ss << waypoints.size() << " waypoints:";
        for (std::size_t wp : waypoints)
        {
          ss << "\n -- " << print_waypoint(wp, graph);
        }
        ss << lanes.size() << " lanes:";
        for (std::size_t lane : lanes)
        {
          ss << "\n -- " << print_lane(lane, graph);
        }

        std::cout << ss.str() << std::endl;
      }
      context->set_lost(Location { now, map, location });
    }

    if (target_location.has_value())
    {
      const auto p_final = *target_location;
      const auto distance = (p_final - p).norm();
      double rotation = 0.0;
      if (final_orientation.has_value())
      {
        rotation =
          std::fabs(rmf_utils::wrap_to_pi(location[2] - *final_orientation));
        const auto reversible =
          planner->get_configuration().vehicle_traits()
          .get_differential()->is_reversible();
        if (reversible)
        {
          const double alternate_orientation = rmf_utils::wrap_to_pi(
            *final_orientation + M_PI);

          const double alternate_rotation = std::fabs(
            rmf_utils::wrap_to_pi(location[2] - alternate_orientation));
          rotation = std::min(rotation, alternate_rotation);
        }
      }

      const auto& traits = planner->get_configuration().vehicle_traits();
      const auto v = std::max(traits.linear().get_nominal_velocity(), 0.001);
      const auto w =
        std::max(traits.rotational().get_nominal_velocity(), 0.001);
      const auto t = distance / v + rotation / w;
      arrival_estimator(
        rmf_traffic::time::from_seconds(t) + planned_wait_time);
    }
  }
};

void EasyFullControl::CommandExecution::Implementation::finish()
{
  if (auto context = w_context.lock())
  {
    context->worker().schedule(
      [
        context = context,
        data = this->data,
        identifier = this->identifier,
        finisher = this->finisher
      ](const auto&)
      {
        if (!ActivityIdentifier::Implementation::get(*identifier).update_fn)
        {
          // This activity has already finished
          return;
        }

        // Prevent this activity from doing any further updates
        ActivityIdentifier::Implementation::get(
          *identifier).update_fn = nullptr;
        if (data && data->schedule_override.has_value())
        {
          data->release_stubbornness();
          RCLCPP_INFO(
            context->node()->get_logger(),
            "Requesting replan for [%s] after finishing a schedule override",
            context->requester_id().c_str());
          context->request_replan();
        }
        else
        {
          // Trigger the next step in the sequence
          finisher();
        }
      });
  }
}

auto EasyFullControl::CommandExecution::Implementation::override_schedule(
  std::string map,
  std::vector<Eigen::Vector3d> path,
  rmf_traffic::Duration hold) -> Stubbornness
{
  auto stubborn = std::make_shared<StubbornOverride>();
  if (const auto context = w_context.lock())
  {
    context->worker().schedule(
      [
        context,
        stubborn,
        data = this->data,
        identifier = this->identifier,
        map = std::move(map),
        path = std::move(path),
        hold
      ](const auto&)
      {
        if (!ActivityIdentifier::Implementation::get(*identifier).update_fn)
        {
          // Don't do anything because this command is finished
          return;
        }

        data->release_stubbornness();
        data->schedule_override = ScheduleOverride::make(
          context, map, path, hold, stubborn);
      });
  }

  return Stubbornness::Implementation::make(stubborn);
}

auto EasyFullControl::CommandExecution::Implementation::make(
  const std::shared_ptr<RobotContext>& context,
  Data data_,
  std::function<void(CommandExecution)> begin) -> CommandExecution
{
  auto data = std::make_shared<Data>(data_);
  auto update_fn = [w_context = context->weak_from_this(), data](
    const std::string& map,
    Eigen::Vector3d location)
    {
      if (auto context = w_context.lock())
      {
        data->update_location(context, map, location);
      }
    };
  auto identifier = ActivityIdentifier::Implementation::make(update_fn);

  CommandExecution cmd;
  cmd._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{context, data, begin, nullptr, identifier});
  return cmd;
}

auto EasyFullControl::CommandExecution::Implementation::make_hold(
  const std::shared_ptr<RobotContext>& context,
  rmf_traffic::Time expected_time,
  rmf_traffic::PlanId plan_id,
  std::function<void()> finisher) -> CommandExecution
{
  auto update_fn = [
    w_context = context->weak_from_this(),
    expected_time,
    plan_id
    ](
    const std::string& map,
    Eigen::Vector3d location)
    {
      const auto context = w_context.lock();
      if (!context)
        return;

      const auto delay = context->now() - expected_time;
      context->itinerary().cumulative_delay(plan_id, delay);
      if (const auto nav_params = context->nav_params())
      {
        if (context->debug_positions)
        {
          std::cout << "Searching for location from " << __FILE__ << "|" <<
            __LINE__ << std::endl;
        }
        nav_params->search_for_location(map, location, *context);
      }
    };
  auto identifier = ActivityIdentifier::Implementation::make(update_fn);

  CommandExecution cmd;
  cmd._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{context, nullptr, nullptr, finisher, identifier});
  return cmd;
}

//==============================================================================
void EasyFullControl::CommandExecution::finished()
{
  _pimpl->finish();
}

//==============================================================================
bool EasyFullControl::CommandExecution::okay() const
{
  if (!_pimpl->identifier)
  {
    return false;
  }

  if (!ActivityIdentifier::Implementation::get(*_pimpl->identifier).update_fn)
  {
    return false;
  }

  return true;
}

//==============================================================================
auto EasyFullControl::CommandExecution::override_schedule(
  std::string map,
  std::vector<Eigen::Vector3d> path,
  rmf_traffic::Duration hold) -> Stubbornness
{
  return _pimpl->override_schedule(std::move(map), std::move(path), hold);
}

//==============================================================================
auto EasyFullControl::CommandExecution::identifier() const
-> ConstActivityIdentifierPtr
{
  return _pimpl->identifier;
}

//==============================================================================
EasyFullControl::CommandExecution::CommandExecution()
{
  // Do nothing
}

//==============================================================================
EasyFullControl::EasyFullControl()
{
  // Do nothing
}

//==============================================================================
const std::string& EasyFullControl::Destination::map() const
{
  return _pimpl->map;
}

//==============================================================================
Eigen::Vector3d EasyFullControl::Destination::position() const
{
  return _pimpl->position;
}

//==============================================================================
Eigen::Vector2d EasyFullControl::Destination::xy() const
{
  return _pimpl->position.block<2, 1>(0, 0);
}

//==============================================================================
double EasyFullControl::Destination::yaw() const
{
  return _pimpl->position[2];
}

//==============================================================================
std::optional<std::size_t> EasyFullControl::Destination::graph_index() const
{
  return _pimpl->graph_index;
}

//==============================================================================
std::string EasyFullControl::Destination::name() const
{
  return _pimpl->name;
}

//==============================================================================
std::optional<double> EasyFullControl::Destination::speed_limit() const
{
  return _pimpl->speed_limit;
}

//==============================================================================
std::optional<std::string> EasyFullControl::Destination::dock() const
{
  return _pimpl->dock;
}

//==============================================================================
rmf_traffic::agv::Graph::LiftPropertiesPtr
EasyFullControl::Destination::inside_lift() const
{
  return _pimpl->lift;
}

//==============================================================================
EasyFullControl::Destination::Destination()
{
  // Do nothing
}

//==============================================================================
struct ProgressTracker : std::enable_shared_from_this<ProgressTracker>
{
  /// The queue of commands to execute while following this path, in reverse
  /// order so that the next command can always be popped off the back.
  std::vector<EasyFullControl::CommandExecution> reverse_queue;
  EasyFullControl::ActivityIdentifierPtr current_identifier;
  TriggerOnce finished;

  void next()
  {
    if (reverse_queue.empty())
    {
      current_identifier = nullptr;
      finished.trigger();
      return;
    }

    auto current_activity = reverse_queue.back();
    reverse_queue.pop_back();
    current_identifier = EasyFullControl::CommandExecution::Implementation
      ::get(current_activity).identifier;
    auto& current_activity_impl =
      EasyFullControl::CommandExecution::Implementation::get(current_activity);
    current_activity_impl.finisher = [w_progress = weak_from_this()]()
      {
        if (const auto progress = w_progress.lock())
        {
          progress->next();
        }
      };


    const auto begin = current_activity_impl.begin;
    if (begin)
    {
      begin(std::move(current_activity));
    }
  }

  static std::shared_ptr<ProgressTracker> make(
    std::vector<EasyFullControl::CommandExecution> queue,
    std::function<void()> finished)
  {
    std::reverse(queue.begin(), queue.end());
    auto tracker = std::make_shared<ProgressTracker>();
    tracker->reverse_queue = std::move(queue);
    tracker->finished = TriggerOnce(std::move(finished));
    return tracker;
  }
};

//==============================================================================
/// Implements a state machine to send waypoints from follow_new_path() one
/// at a time to the robot via its API. Also updates state of robot via a timer.
class EasyCommandHandle : public RobotCommandHandle,
  public std::enable_shared_from_this<EasyCommandHandle>
{
public:
  using Planner = rmf_traffic::agv::Planner;
  using Graph = rmf_traffic::agv::Graph;
  using VehicleTraits = rmf_traffic::agv::VehicleTraits;
  using ActionExecution = RobotUpdateHandle::ActionExecution;
  using ActionExecutor = RobotUpdateHandle::ActionExecutor;
  using NavigationRequest = EasyFullControl::NavigationRequest;
  using StopRequest = EasyFullControl::StopRequest;
  using Status = rmf_task::Event::Status;
  using ActivityIdentifierPtr = EasyFullControl::ActivityIdentifierPtr;

  // State machine values.
  enum class InternalRobotState : uint8_t
  {
    IDLE = 0,
    MOVING = 1
  };

  EasyCommandHandle(
    std::shared_ptr<NavParams> nav_params,
    NavigationRequest handle_nav_request,
    StopRequest handle_stop);

  // Implement base class methods.
  void stop() final;

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) final;

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final;

  Eigen::Vector3d to_robot_coordinates(
    const std::string& map,
    Eigen::Vector3d position) const
  {
    auto robot_position = nav_params->to_robot_coordinates(map, position);
    if (robot_position.has_value())
    {
      return *robot_position;
    }

    if (const auto context = w_context.lock())
    {
      RCLCPP_WARN(
        context->node()->get_logger(),
        "[EasyFullControl] Unable to find robot transform for map [%s] for "
        "robot [%s]. We will not apply a transform.",
        map.c_str(),
        context->requester_id().c_str());
    }
    return position;
  }

  std::weak_ptr<RobotContext> w_context;
  std::shared_ptr<NavParams> nav_params;
  std::shared_ptr<ProgressTracker> current_progress;

  // Callbacks from user
  NavigationRequest handle_nav_request;
  StopRequest handle_stop;
};

//==============================================================================
EasyCommandHandle::EasyCommandHandle(
  std::shared_ptr<NavParams> nav_params_,
  NavigationRequest handle_nav_request_,
  StopRequest handle_stop_)
: nav_params(std::move(nav_params_)),
  handle_nav_request(std::move(handle_nav_request_)),
  handle_stop(std::move(handle_stop_))
{
  // Do nothing
}

//==============================================================================
void EasyCommandHandle::stop()
{
  if (!this->current_progress)
  {
    return;
  }

  const auto activity_identifier = this->current_progress->current_identifier;
  if (!activity_identifier)
  {
    return;
  }

  /// Prevent any further specialized updates.
  EasyFullControl::ActivityIdentifier::Implementation
  ::get(*activity_identifier).update_fn = nullptr;

  this->current_progress = nullptr;
  this->handle_stop(activity_identifier);
}

//==============================================================================
void EasyCommandHandle::follow_new_path(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& cmd_waypoints,
  ArrivalEstimator next_arrival_estimator_,
  RequestCompleted path_finished_callback_)
{
  const auto context = w_context.lock();
  if (!context)
  {
    return;
  }

  RCLCPP_DEBUG(
    context->node()->get_logger(),
    "follow_new_path for robot [%s] with PlanId [%ld]",
    context->requester_id().c_str(),
    context->itinerary().current_plan_id());

  if (cmd_waypoints.empty() ||
    next_arrival_estimator_ == nullptr ||
    path_finished_callback_ == nullptr)
  {
    RCLCPP_WARN(
      context->node()->get_logger(),
      "Received a new path for robot [%s] with invalid parameters. "
      " Ignoring...",
      context->requester_id().c_str());
    return;
  }

  const auto& planner = context->planner();
  if (!planner)
  {
    RCLCPP_ERROR(
      context->node()->get_logger(),
      "Planner missing for [%s], cannot follow new path commands",
      context->requester_id().c_str());
    return;
  }
  const auto& graph = planner->get_configuration().graph();
  std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints = cmd_waypoints;

  if (waypoints.size() < 2)
  {
    // This command doesn't actually want us to go anywhere so we will consider
    // it completed right away. But in case the robot is doing something else
    // right now, we will command it to stop.
    stop();
    path_finished_callback_();
    return;
  }

  std::optional<std::string> opt_initial_map;
  for (const auto& wp : waypoints)
  {
    if (wp.graph_index().has_value())
    {
      const std::size_t i = *wp.graph_index();
      opt_initial_map = graph.get_waypoint(i).get_map_name();
      break;
    }
  }

  if (!opt_initial_map.has_value())
  {
    for (const auto& l : context->location())
    {
      opt_initial_map = graph.get_waypoint(l.waypoint()).get_map_name();
      break;
    }
  }

  if (!opt_initial_map.has_value())
  {
    RCLCPP_ERROR(
      context->node()->get_logger(),
      "Could not find an initial map in follow_new_path command for robot [%s]."
      "This means the robot is lost and may need an operator to intervene.",
      context->requester_id().c_str());
    // Stop to prevent cascading problems.
    stop();
    return;
  }
  std::string initial_map = *opt_initial_map;

  std::vector<EasyFullControl::CommandExecution> queue;
  const auto& current_location = context->location();

  bool found_connection = false;
  std::size_t i0 = 0;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    const auto& wp = waypoints[i];
    if (wp.graph_index().has_value())
    {
      for (const auto& l : current_location)
      {
        if (nav_params->in_same_stack(*wp.graph_index(),
          l.waypoint()) && !l.lane().has_value())
        {
          if (l.location().has_value())
          {
            if (i > 0)
            {
              found_connection = true;
              i0 = i - 1;
            }
            else
            {
              const double dist =
                (*l.location() - wp.position().block<2, 1>(0, 0)).norm();
              if (dist <= nav_params->max_merge_lane_distance)
              {
                found_connection = true;
                i0 = 0;
              }
            }
          }
          else
          {
            found_connection = true;
            i0 = i;
          }
        }

        if (l.lane().has_value())
        {
          Eigen::Vector2d p_l;
          if (l.location().has_value())
          {
            p_l = *l.location();
          }
          else
          {
            p_l = graph.get_waypoint(l.waypoint()).get_location();
          }
          const double dist = (wp.position().block<2, 1>(0, 0) - p_l).norm();
          const double merge_dist = graph.get_waypoint(*wp.graph_index())
            .merge_radius().value_or(nav_params->max_merge_lane_distance);
          if (dist <= merge_dist)
          {
            found_connection = true;
            i0 = i;
          }
        }
      }
    }
    else
    {
      for (const auto& l : current_location)
      {
        Eigen::Vector2d p_l;
        if (l.location().has_value())
        {
          p_l = *l.location();
        }
        else
        {
          p_l = graph.get_waypoint(l.waypoint()).get_location();
        }
        const double dist = (wp.position().block<2, 1>(0, 0) - p_l).norm();
        if (dist <= nav_params->max_merge_lane_distance)
        {
          found_connection = true;
          i0 = i;
        }
      }
    }

    if (i > 0)
    {
      for (const auto lane : wp.approach_lanes())
      {
        for (const auto& l : current_location)
        {
          if (l.lane().has_value())
          {
            if (lane == *l.lane())
            {
              found_connection = true;
              i0 = i - 1;
            }
          }
        }
      }
    }
  }

  if (!found_connection)
  {
    // The robot has drifted away from the starting point since the plan started
    // so we'll ask for a new plan.
    std::stringstream ss;
    ss << "Current location" << print_starts(current_location, graph);
    ss << "\nCommanded path:";
    const auto t0 = waypoints.front().time();
    for (const auto& wp : waypoints)
    {
      ss << "\n -- " << print_plan_waypoint(wp, graph, t0);
    }

    RCLCPP_INFO(
      context->node()->get_logger(),
      "Requesting replan for [%s] because it is too far from its commanded "
      "path. Details:\n%s",
      context->requester_id().c_str(),
      ss.str().c_str());

    // Stop the robot to prevent it from diverging too far while a replan
    // happens.
    stop();
    context->request_replan();
    return;
  }

  if (i0 >= waypoints.size() - 1)
  {
    // Always issue at least one command to approach the final waypoint.
    i0 = waypoints.size() - 2;
  }

  std::size_t i1 = i0 + 1;
  while (i1 < waypoints.size())
  {
    std::vector<std::size_t> cmd_wps;
    std::vector<std::size_t> cmd_lanes;
    const auto& wp0 = waypoints[i0];
    const auto& wp1 = waypoints[i1];
    if (wp0.graph_index().has_value())
    {
      cmd_wps.push_back(*wp0.graph_index());
    }

    for (auto lane_id : wp1.approach_lanes())
    {
      cmd_lanes.push_back(lane_id);
      const auto& lane = graph.get_lane(lane_id);
      const std::size_t entry_wp = lane.entry().waypoint_index();
      const std::size_t exit_wp = lane.exit().waypoint_index();
      for (auto wp : {entry_wp, exit_wp})
      {
        if (std::find(cmd_wps.begin(), cmd_wps.end(), wp) == cmd_wps.end())
        {
          cmd_wps.push_back(wp);
        }
      }
    }

    std::string map = [&]()
      {
        if (wp1.graph_index().has_value())
        {
          return graph.get_waypoint(*wp1.graph_index()).get_map_name();
        }

        return initial_map;
      }();
    if (initial_map != map)
    {
      initial_map = map;
    }

    std::optional<double> speed_limit;
    if (!wp1.approach_lanes().empty())
    {
      const auto arrival_lane = wp1.approach_lanes().back();
      speed_limit = graph.get_lane(arrival_lane).properties().speed_limit();
    }

    Eigen::Vector3d target_position = wp1.position();
    std::size_t target_index = i1;
    rmf_traffic::Duration planned_wait_time = rmf_traffic::Duration(0);
    if (nav_params->skip_rotation_commands)
    {
      std::size_t i2 = i1 + 1;
      while (i2 < waypoints.size())
      {
        const auto& wp2 = waypoints[i2];

        const bool midlane_wait =
          !wp1.graph_index().has_value() && !wp2.graph_index().has_value();
        const bool same_stack =
          wp1.graph_index().has_value() && wp2.graph_index().has_value()
          && nav_params->in_same_stack(*wp1.graph_index(), *wp2.graph_index());

        if (same_stack || midlane_wait)
        {
          target_index = i2;
          target_position = wp2.position();
          if (std::abs(wp1.position()[2] -
            wp2.position()[2])*180.0 / M_PI < 1e-2)
          {
            // The plan had a wait between these points.
            planned_wait_time += wp2.time() - wp1.time();
          }
          ++i2;
          continue;
        }
        break;
      }
    }

    rmf_traffic::agv::Graph::LiftPropertiesPtr in_lift;
    if (wp1.graph_index().has_value())
    {
      in_lift = graph.get_waypoint(*wp1.graph_index()).in_lift();
    }
    else
    {
      for (const auto& lift : graph.all_known_lifts())
      {
        if (lift->is_in_lift(target_position.block<2, 1>(0, 0)))
        {
          in_lift = lift;
          break;
        }
      }
    }

    const auto command_position = to_robot_coordinates(map, target_position);
    auto destination = EasyFullControl::Destination::Implementation::make(
      std::move(map),
      command_position,
      wp1.graph_index(),
      nav_params->get_vertex_name(graph, wp1.graph_index()),
      speed_limit,
      in_lift);

    const auto target_p = waypoints.at(target_index).position();
    queue.push_back(
      EasyFullControl::CommandExecution::Implementation::make(
        context,
        EasyFullControl::CommandExecution::Implementation::Data{
          cmd_wps,
          cmd_lanes,
          target_position.block<2, 1>(0, 0),
          target_position[2],
          planned_wait_time,
          std::nullopt,
          nav_params,
          [next_arrival_estimator_, target_index,
          target_p](rmf_traffic::Duration dt)
          {
            next_arrival_estimator_(target_index, dt);
          }
        },
        [
          handle_nav_request = this->handle_nav_request,
          destination = std::move(destination)
        ](EasyFullControl::CommandExecution execution)
        {
          handle_nav_request(destination, execution);
        }
    ));

    i0 = target_index;
    i1 = i0 + 1;
  }

  this->current_progress = ProgressTracker::make(
    queue,
    path_finished_callback_);
  this->current_progress->next();
}

//==============================================================================
namespace {
class DockFinder : public rmf_traffic::agv::Graph::Lane::Executor
{
public:
  DockFinder(std::string dock_name)
  : looking_for(std::move(dock_name))
  {
    // Do nothing
  }

  void execute(const DoorOpen&) override {}
  void execute(const DoorClose&) override {}
  void execute(const LiftSessionBegin&) override {}
  void execute(const LiftDoorOpen&) override {}
  void execute(const LiftSessionEnd&) override {}
  void execute(const LiftMove&) override {}
  void execute(const Wait&) override {}
  void execute(const Dock& dock) override
  {
    if (looking_for == dock.dock_name())
    {
      found = true;
    }
  }

  std::string looking_for;
  bool found = false;
};
} // anonymous namespace

//==============================================================================
void EasyCommandHandle::dock(
  const std::string& dock_name_,
  RequestCompleted docking_finished_callback_)
{
  const auto context = w_context.lock();
  if (!context)
  {
    return;
  }

  RCLCPP_DEBUG(
    context->node()->get_logger(),
    "Received a request to dock robot [%s] at [%s]...",
    context->requester_id().c_str(),
    dock_name_.c_str());

  const auto plan_id = context->itinerary().current_plan_id();
  auto planner = context->planner();
  if (!planner)
  {
    RCLCPP_ERROR(
      context->node()->get_logger(),
      "Planner unavailable for robot [%s], cannot execute docking command [%s]",
      context->requester_id().c_str(),
      dock_name_.c_str());
    return;
  }

  const auto& graph = planner->get_configuration().graph();
  DockFinder finder(dock_name_);
  std::optional<std::size_t> found_lane;
  for (std::size_t i = 0; i < graph.num_lanes(); ++i)
  {
    const auto& lane = graph.get_lane(i);
    if (const auto event = lane.entry().event())
    {
      event->execute(finder);
    }

    if (const auto event = lane.exit().event())
    {
      event->execute(finder);
    }

    if (finder.found)
    {
      found_lane = i;
      break;
    }
  }

  if (!found_lane.has_value())
  {
    RCLCPP_WARN(
      context->node()->get_logger(),
      "Unable to find a dock named [%s] in the graph for robot [%s]. We "
      "will skip this command as finished to avoid permanently blocking.",
      dock_name_.c_str(),
      context->requester_id().c_str());
    docking_finished_callback_();
    return;
  }

  const auto& lane = graph.get_lane(*found_lane);
  const std::size_t i0 = lane.entry().waypoint_index();
  const std::size_t i1 = lane.exit().waypoint_index();
  const auto& wp0 = graph.get_waypoint(i0);
  const auto& wp1 = graph.get_waypoint(i1);
  const Eigen::Vector2d p0 = wp0.get_location();
  const Eigen::Vector2d p1 = wp1.get_location();
  const double dist = (p1 - p0).norm();
  const auto& traits = planner->get_configuration().vehicle_traits();
  const double v = std::max(traits.linear().get_nominal_velocity(), 0.001);
  const auto dt = rmf_traffic::time::from_seconds(dist / v);
  const auto& itin = context->itinerary();
  const auto now = context->now();
  const auto initial_delay = itin.cumulative_delay(itin.current_plan_id())
    .value_or(rmf_traffic::Duration(0));
  const rmf_traffic::Time expected_arrival = now + dt - initial_delay;

  auto data = EasyFullControl::CommandExecution::Implementation::Data{
    {i0, i1},
    {*found_lane},
    p1,
    std::nullopt,
    rmf_traffic::Duration(0),
    std::nullopt,
    nav_params,
    [w_context = context->weak_from_this(), expected_arrival, plan_id](
      rmf_traffic::Duration dt)
    {
      const auto context = w_context.lock();
      if (!context)
      {
        return;
      }

      context->worker().schedule([
          w = context->weak_from_this(),
          expected_arrival,
          plan_id,
          dt
        ](const auto&)
        {
          const auto context = w.lock();
          if (!context)
            return;

          const rmf_traffic::Time now = context->now();
          const auto updated_arrival = now + dt;
          const auto delay = updated_arrival - expected_arrival;
          context->itinerary().cumulative_delay(
            plan_id, delay, std::chrono::seconds(1));
        });
    }
  };

  const double dy = p1.y() - p0.y();
  const double dx = p1.x() - p0.x();
  double angle = 0.0;
  if (dx*dx + dy*dy > 1e-6)
  {
    angle = std::atan2(dy, dx);
  }

  const Eigen::Vector3d position(p1.x(), p1.y(), angle);
  const auto command_position = to_robot_coordinates(
    wp1.get_map_name(), position);

  auto destination = EasyFullControl::Destination::Implementation::make(
    wp1.get_map_name(),
    command_position,
    i1,
    nav_params->get_vertex_name(graph, i1),
    lane.properties().speed_limit(),
    wp1.in_lift(),
    dock_name_);

  auto cmd = EasyFullControl::CommandExecution::Implementation::make(
    context,
    data,
    [
      handle_nav_request = this->handle_nav_request,
      destination = std::move(destination)
    ](
      EasyFullControl::CommandExecution execution)
    {
      handle_nav_request(destination, execution);
    });

  this->current_progress = ProgressTracker::make(
    {std::move(cmd)},
    std::move(docking_finished_callback_));
  this->current_progress->next();
}

//==============================================================================
ConsiderRequest consider_all()
{
  return [](const nlohmann::json&, FleetUpdateHandle::Confirmation& confirm)
    {
      confirm.accept();
    };
}

//==============================================================================
class EasyFullControl::EasyRobotUpdateHandle::Implementation
{
public:

  struct Updater
  {
    std::shared_ptr<RobotUpdateHandle> handle;
    std::shared_ptr<NavParams> nav_params;

    Updater(std::shared_ptr<NavParams> params_)
    : handle(nullptr),
      nav_params(std::move(params_))
    {
      // Do nothing
    }

    Eigen::Vector3d to_rmf_coordinates(
      const std::string& map,
      Eigen::Vector3d position,
      const RobotContext& context) const
    {
      const auto p = nav_params->to_rmf_coordinates(map, position);
      if (p.has_value())
      {
        return *p;
      }

      RCLCPP_WARN(
        context.node()->get_logger(),
        "[EasyFullControl] Unable to find robot transform for map [%s] for "
        "robot [%s]. We will not apply a transform.",
        map.c_str(),
        context.requester_id().c_str());
      return position;
    }
  };

  std::shared_ptr<Updater> updater;
  rxcpp::schedulers::worker worker;

  static Implementation& get(EasyRobotUpdateHandle& handle)
  {
    return *handle._pimpl;
  }

  Implementation(
    std::shared_ptr<NavParams> params_,
    rxcpp::schedulers::worker worker_)
  : updater(std::make_shared<Updater>(params_)),
    worker(worker_)
  {
    // Do nothing
  }

  static std::shared_ptr<EasyRobotUpdateHandle> make(
    std::shared_ptr<NavParams> params_,
    rxcpp::schedulers::worker worker_)
  {
    auto handle = std::shared_ptr<EasyRobotUpdateHandle>(
      new EasyRobotUpdateHandle);
    handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
      std::move(params_), std::move(worker_));
    return handle;
  }
};

//==============================================================================
void EasyFullControl::EasyRobotUpdateHandle::update(
  RobotState state,
  ConstActivityIdentifierPtr current_activity)
{
  _pimpl->worker.schedule(
    [
      state = std::move(state),
      current_activity = std::move(current_activity),
      updater = _pimpl->updater
    ](const auto&)
    {
      if (!updater->handle)
      {
        return;
      }

      auto context = RobotUpdateHandle::Implementation
      ::get(*updater->handle).get_context();

      context->current_battery_soc(state.battery_state_of_charge());

      const auto position = updater->to_rmf_coordinates(
        state.map(), state.position(), *context);

      if (current_activity)
      {
        const auto update_fn =
        ActivityIdentifier::Implementation::get(*current_activity).update_fn;
        if (update_fn)
        {
          update_fn(
            state.map(), position);
          return;
        }
      }

      if (context->debug_positions)
      {
        std::cout << "Searching for location from " << __FILE__ << "|" << __LINE__ << std::endl;
      }
      updater->nav_params->search_for_location(state.map(), position, *context);
    });
}

//==============================================================================
double EasyFullControl::EasyRobotUpdateHandle::max_merge_waypoint_distance()
const
{
  return _pimpl->updater->nav_params->max_merge_waypoint_distance;
}

//==============================================================================
void EasyFullControl::EasyRobotUpdateHandle::set_max_merge_waypoint_distance(
  double distance)
{
  _pimpl->updater->nav_params->max_merge_waypoint_distance = distance;
}

//==============================================================================
double EasyFullControl::EasyRobotUpdateHandle::max_merge_lane_distance() const
{
  return _pimpl->updater->nav_params->max_merge_lane_distance;
}

//==============================================================================
void EasyFullControl::EasyRobotUpdateHandle::set_max_merge_lane_distance(
  double distance)
{
  _pimpl->updater->nav_params->max_merge_lane_distance = distance;
}

//==============================================================================
double EasyFullControl::EasyRobotUpdateHandle::min_lane_length() const
{
  return _pimpl->updater->nav_params->min_lane_length;
}

//==============================================================================
void EasyFullControl::EasyRobotUpdateHandle::set_min_lane_length(
  double length)
{
  _pimpl->updater->nav_params->min_lane_length = length;
}

//==============================================================================
std::shared_ptr<RobotUpdateHandle>
EasyFullControl::EasyRobotUpdateHandle::more()
{
  if (_pimpl->updater)
  {
    return _pimpl->updater->handle;
  }
  return nullptr;
}

//==============================================================================
std::shared_ptr<const RobotUpdateHandle>
EasyFullControl::EasyRobotUpdateHandle::more() const
{
  if (_pimpl->updater)
  {
    return _pimpl->updater->handle;
  }

  return nullptr;
}

//==============================================================================
EasyFullControl::EasyRobotUpdateHandle::EasyRobotUpdateHandle()
{
  // Do nothing
}

//==============================================================================
class EasyFullControl::FleetConfiguration::Implementation
{
public:
  std::string fleet_name;
  std::optional<std::unordered_map<std::string,
    Transformation>> transformations_to_robot_coordinates;
  std::unordered_map<std::string,
    RobotConfiguration> known_robot_configurations;
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits;
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;
  rmf_battery::agv::ConstBatterySystemPtr battery_system;
  rmf_battery::ConstMotionPowerSinkPtr motion_sink;
  rmf_battery::ConstDevicePowerSinkPtr ambient_sink;
  rmf_battery::ConstDevicePowerSinkPtr tool_sink;
  double recharge_threshold;
  double recharge_soc;
  bool account_for_battery_drain;
  std::optional<rmf_traffic::Duration> retreat_to_charger_interval;
  std::unordered_map<std::string, ConsiderRequest> task_consideration;
  std::unordered_map<std::string, ConsiderRequest> action_consideration;
  rmf_task::ConstRequestFactoryPtr finishing_request;
  bool skip_rotation_commands;
  std::optional<std::string> server_uri;
  rmf_traffic::Duration max_delay;
  rmf_traffic::Duration update_interval;
  bool default_responsive_wait;
  double default_max_merge_waypoint_distance;
  double default_max_merge_lane_distance;
  double default_min_lane_length;
  std::unordered_map<std::string, std::string> lift_emergency_levels;
};

//==============================================================================
EasyFullControl::FleetConfiguration::FleetConfiguration(
  const std::string& fleet_name,
  std::optional<std::unordered_map<std::string, Transformation>>
  transformations_to_robot_coordinates,
  std::unordered_map<std::string, RobotConfiguration>
  known_robot_configurations,
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits,
  std::shared_ptr<const rmf_traffic::agv::Graph> graph,
  rmf_battery::agv::ConstBatterySystemPtr battery_system,
  rmf_battery::ConstMotionPowerSinkPtr motion_sink,
  rmf_battery::ConstDevicePowerSinkPtr ambient_sink,
  rmf_battery::ConstDevicePowerSinkPtr tool_sink,
  double recharge_threshold,
  double recharge_soc,
  bool account_for_battery_drain,
  std::unordered_map<std::string, ConsiderRequest> task_consideration,
  std::unordered_map<std::string, ConsiderRequest> action_consideration,
  rmf_task::ConstRequestFactoryPtr finishing_request,
  bool skip_rotation_commands,
  std::optional<std::string> server_uri,
  rmf_traffic::Duration max_delay,
  rmf_traffic::Duration update_interval,
  bool default_responsive_wait,
  double default_max_merge_waypoint_distance,
  double default_max_merge_lane_distance,
  double default_min_lane_length)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(fleet_name),
        std::move(transformations_to_robot_coordinates),
        std::move(known_robot_configurations),
        std::move(traits),
        std::move(graph),
        std::move(battery_system),
        std::move(motion_sink),
        std::move(ambient_sink),
        std::move(tool_sink),
        std::move(recharge_threshold),
        std::move(recharge_soc),
        std::move(account_for_battery_drain),
        std::chrono::seconds(10),
        std::move(task_consideration),
        std::move(action_consideration),
        std::move(finishing_request),
        skip_rotation_commands,
        std::move(server_uri),
        std::move(max_delay),
        std::move(update_interval),
        default_responsive_wait,
        std::move(default_max_merge_waypoint_distance),
        std::move(default_max_merge_lane_distance),
        std::move(default_min_lane_length),
        {}
      }))
{
  // Do nothing
}

//==============================================================================
std::optional<EasyFullControl::FleetConfiguration>
EasyFullControl::FleetConfiguration::from_config_files(
  const std::string& config_file,
  const std::string& nav_graph_path,
  std::optional<std::string> server_uri)
{
  // Load fleet config file
  const auto fleet_config = YAML::LoadFile(config_file);
  // Check that config file is valid and contains all necessary nodes
  const YAML::Node rmf_fleet = fleet_config["rmf_fleet"];
  if (!rmf_fleet)
  {
    std::cout
      << "rmf_fleet dictionary is not provided in the configuration file ["
      << config_file << "] so we cannot parse a fleet configuration."
      << std::endl;
    return std::nullopt;
  }

  // Fleet name
  if (!rmf_fleet["name"])
  {
    std::cout << "Fleet name is not provided" << std::endl;
    return std::nullopt;
  }
  const std::string fleet_name = rmf_fleet["name"].as<std::string>();

  // Profile
  if (!rmf_fleet["profile"] || !rmf_fleet["profile"]["footprint"] ||
    !rmf_fleet["profile"]["vicinity"])
  {
    std::cout << "Fleet profile is not provided" << std::endl;
    return std::nullopt;
  }
  const YAML::Node profile = rmf_fleet["profile"];
  const double footprint_rad = profile["footprint"].as<double>();
  const double vicinity_rad = profile["vicinity"].as<double>();

  // Traits
  if (!rmf_fleet["limits"] || !rmf_fleet["limits"]["linear"] ||
    !rmf_fleet["limits"]["angular"])
  {
    std::cout << "Fleet traits are not provided" << std::endl;
    return std::nullopt;
  }
  const YAML::Node limits = rmf_fleet["limits"];
  const YAML::Node linear = limits["linear"];
  const double v_nom = linear[0].as<double>();
  const double a_nom = linear[1].as<double>();
  const YAML::Node angular = limits["angular"];
  const double w_nom = angular[0].as<double>();
  const double b_nom = angular[1].as<double>();

  // Reversibility, defaults to false
  bool reversible = false;
  if (rmf_fleet["reversible"])
  {
    reversible = rmf_fleet["reversible"].as<bool>();
  }

  auto traits = std::make_shared<VehicleTraits>(VehicleTraits{
        {v_nom, a_nom},
        {w_nom, b_nom},
        rmf_traffic::Profile{
          rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(
            footprint_rad),
          rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(
            vicinity_rad)
        }
      });
  traits->get_differential()->set_reversible(reversible);

  // Graph
  const auto graph = parse_graph(nav_graph_path, *traits);

  // Set up parameters required for task planner
  // Battery system
  if (!rmf_fleet["battery_system"] || !rmf_fleet["battery_system"]["voltage"] ||
    !rmf_fleet["battery_system"]["capacity"] ||
    !rmf_fleet["battery_system"]["charging_current"])
  {
    std::cout << "Fleet battery system is not provided" << std::endl;
    return std::nullopt;
  }
  const YAML::Node battery = rmf_fleet["battery_system"];
  const double voltage = battery["voltage"].as<double>();
  const double capacity = battery["capacity"].as<double>();
  const double charging_current = battery["charging_current"].as<double>();

  const auto battery_system_optional = rmf_battery::agv::BatterySystem::make(
    voltage, capacity, charging_current);
  if (!battery_system_optional.has_value())
  {
    std::cout << "Invalid battery parameters" << std::endl;
    return std::nullopt;
  }
  const auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
    *battery_system_optional);

  // Mechanical system
  if (!rmf_fleet["mechanical_system"])
  {
    std::cout << "Fleet [mechanical_system] information is not provided"
              << std::endl;
    return std::nullopt;
  }

  if (
    !rmf_fleet["mechanical_system"]["mass"] ||
    !rmf_fleet["mechanical_system"]["moment_of_inertia"] ||
    !rmf_fleet["mechanical_system"]["friction_coefficient"])
  {
    std::cout << "Fleet [mechanical_system] information is not valid"
              << std::endl;
    return std::nullopt;
  }

  const YAML::Node mechanical = rmf_fleet["mechanical_system"];
  const double mass = mechanical["mass"].as<double>();
  const double moment_of_inertia = mechanical["moment_of_inertia"].as<double>();
  const double friction = mechanical["friction_coefficient"].as<double>();

  auto mechanical_system_optional = rmf_battery::agv::MechanicalSystem::make(
    mass, moment_of_inertia, friction);
  if (!mechanical_system_optional.has_value())
  {
    std::cout << "Invalid mechanical parameters" << std::endl;
    return std::nullopt;
  }
  rmf_battery::agv::MechanicalSystem& mechanical_system =
    *mechanical_system_optional;

  const auto motion_sink =
    std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
    *battery_system, mechanical_system);

  // Ambient power system
  if (!rmf_fleet["ambient_system"] || !rmf_fleet["ambient_system"]["power"])
  {
    std::cout << "Fleet ambient system is not provided" << std::endl;
    return std::nullopt;
  }
  const YAML::Node ambient_system = rmf_fleet["ambient_system"];
  const double ambient_power_drain = ambient_system["power"].as<double>();
  auto ambient_power_system = rmf_battery::agv::PowerSystem::make(
    ambient_power_drain);
  if (!ambient_power_system)
  {
    std::cout << "Invalid values supplied for ambient power system"
              << std::endl;
    return std::nullopt;
  }
  const auto ambient_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *ambient_power_system);

  // Tool power system
  if (!rmf_fleet["tool_system"] || !rmf_fleet["tool_system"]["power"])
  {
    std::cout << "Fleet tool system is not provided" << std::endl;
    return std::nullopt;
  }
  const YAML::Node tool_system = rmf_fleet["tool_system"];
  const double tool_power_drain = ambient_system["power"].as<double>();
  auto tool_power_system = rmf_battery::agv::PowerSystem::make(
    tool_power_drain);
  if (!tool_power_system)
  {
    std::cout << "Invalid values supplied for tool power system" << std::endl;
    return std::nullopt;
  }
  const auto tool_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  // Drain battery
  bool account_for_battery_drain = true;
  if (!rmf_fleet["account_for_battery_drain"])
  {
    std::cout << "[account_for_battery_drain] value is not provided, "
              << "default to True" << std::endl;
  }
  else
  {
    account_for_battery_drain =
      rmf_fleet["account_for_battery_drain"].as<bool>();
  }
  // Recharge threshold
  double recharge_threshold = 0.2;
  if (!rmf_fleet["recharge_threshold"])
  {
    std::cout
      << "Recharge threshold is not provided, default to 0.2" << std::endl;
  }
  else
  {
    recharge_threshold = rmf_fleet["recharge_threshold"].as<double>();
  }
  // Recharge state of charge
  double recharge_soc = 0.2;
  if (!rmf_fleet["recharge_soc"])
  {
    std::cout << "Recharge state of charge is not provided, default to 1.0"
              << std::endl;
  }
  else
  {
    recharge_soc = rmf_fleet["recharge_soc"].as<double>();
  }
  // Retreat to charger
  std::optional<rmf_traffic::Duration> retreat_to_charger_interval =
    rmf_traffic::time::from_seconds(10);
  const auto retreat_to_charger_yaml = rmf_fleet["retreat_to_charger_interval"];
  if (!retreat_to_charger_yaml)
  {
    std::cout << "[retreat_to_charger_interval] value is not provided, "
              << "default to 10 seconds" << std::endl;
  }
  else if (retreat_to_charger_yaml.IsScalar())
  {
    const double retreat_to_charger_interval_sec =
      retreat_to_charger_yaml.as<double>();
    if (retreat_to_charger_interval_sec <= 0.0)
    {
      std::cout
        << "[retreat_to_charger_interval] has invalid value ["
        << retreat_to_charger_interval_sec << "]. Turning off retreat to "
        << "behavior." << std::endl;
      retreat_to_charger_interval = std::nullopt;
    }
    else
    {
      retreat_to_charger_interval =
        rmf_traffic::time::from_seconds(retreat_to_charger_interval_sec);
    }
  }
  else if (retreat_to_charger_yaml.IsNull())
  {
    retreat_to_charger_interval = std::nullopt;
  }
  else
  {
    const auto mark = retreat_to_charger_yaml.Mark();
    std::cout << "[retreat_to_charger_interval] Unsupported value type "
              << "provided: line " << mark.line << ", column " << mark.column
              << std::endl;
    return std::nullopt;
  }

  // Task capabilities
  std::unordered_map<std::string, ConsiderRequest> task_consideration;
  const YAML::Node task_capabilities = rmf_fleet["task_capabilities"];
  if (!task_capabilities)
  {
    std::cout
      << "[task_capabilities] dictionary was not provided in config file ["
      << config_file << "]" << std::endl;
  }
  else
  {
    for (const auto& capability : task_capabilities)
    {
      std::string task = capability.first.as<std::string>();
      if (task == "loop")
      {
        // Always change loop to patrol to support legacy terminology
        task = "patrol";
      }

      if (capability.second.as<bool>())
      {
        task_consideration[task] = consider_all();
      }
    }

    if (task_consideration.empty())
    {
      std::cout
        << "No known task capabilities found in config file ["
        << config_file << "]" << std::endl;
    }
  }

  // Action considerations
  std::unordered_map<std::string, ConsiderRequest> action_consideration;
  const auto& actions_yaml = rmf_fleet["actions"];
  if (actions_yaml)
  {
    const auto actions = actions_yaml.as<std::vector<std::string>>();
    for (const std::string& action : actions)
    {
      action_consideration[action] = consider_all();
    }
  }

  // Finishing tasks
  std::string finishing_request_string;
  const auto& finishing_request_yaml = rmf_fleet["finishing_request"];
  if (!finishing_request_yaml)
  {
    std::cout
      << "Finishing request is not provided. The valid finishing requests "
      "are [charge, park, nothing]. The task planner will default to [nothing]."
      << std::endl;
  }
  else
  {
    finishing_request_string = finishing_request_yaml.as<std::string>();
  }
  std::cout << "Finishing request: " << finishing_request_string << std::endl;
  rmf_task::ConstRequestFactoryPtr finishing_request;
  if (finishing_request_string == "charge")
  {
    auto charge_factory =
      std::make_shared<rmf_task::requests::ChargeBatteryFactory>();
    charge_factory->set_indefinite(true);
    finishing_request = charge_factory;
    std::cout
      << "Fleet is configured to perform ChargeBattery as finishing request"
      << std::endl;
  }
  else if (finishing_request_string == "park")
  {
    finishing_request =
      std::make_shared<rmf_fleet_adapter::tasks::ParkRobotIndefinitely>(
      "idle", nullptr);
    std::cout
      << "Fleet is configured to perform ParkRobot as finishing request"
      << std::endl;
  }
  else if (finishing_request_string == "nothing")
  {
    std::cout << "Fleet is not configured to perform any finishing request"
              << std::endl;
  }
  else
  {
    std::cout
      << "Provided finishing request " << finishing_request_string
      << "is unsupported. The valid finishing requests are"
      "[charge, park, nothing]. The task planner will default to [nothing].";
  }

  // Ignore rotations within path commands
  bool skip_rotation_commands = true;
  if (rmf_fleet["skip_rotation_commands"])
  {
    skip_rotation_commands = rmf_fleet["skip_rotation_commands"].as<bool>();
  }

  // Set the fleet state topic publish period
  double fleet_state_frequency = 2.0;
  if (!rmf_fleet["publish_fleet_state"])
  {
    std::cout
      << "Fleet state publish frequency is not provided, default to 2.0"
      << std::endl;
  }
  else
  {
    fleet_state_frequency = rmf_fleet["publish_fleet_state"].as<double>();
  }
  const double update_interval = 1.0/fleet_state_frequency;

  // Set the maximum delay
  double max_delay = 10.0;
  if (!rmf_fleet["max_delay"])
  {
    std::cout << "Maximum delay is not provided, default to 10.0" << std::endl;
  }
  else
  {
    max_delay = rmf_fleet["max_delay"].as<double>();
  }

  std::optional<std::unordered_map<std::string, Transformation>> tf_dict;
  const auto transforms = rmf_fleet["transforms"];
  if (transforms)
  {
    if (!transforms.IsMap())
    {
      std::cerr
        << "[transforms] entry should be a dictionary in config file ["
        << config_file << "]" << std::endl;
    }
    else
    {
      tf_dict = std::unordered_map<std::string, Transformation>();
      for (const auto& node : transforms)
      {
        const auto map = node.first.as<std::string>();
        const auto& transform_yaml = node.second;
        const auto translation_yaml = transform_yaml["translation"];
        Eigen::Vector2d translation = Eigen::Vector2d(0.0, 0.0);
        if (translation_yaml)
        {
          const auto xy_vec = translation_yaml.as<std::vector<double>>();
          if (xy_vec.size() != 2)
          {
            std::cerr
              << "[traslation] element has invalid number of elements ["
              << xy_vec.size() << "] (it should be exactly 2) in config file ["
              << config_file << "]" << std::endl;
          }

          std::size_t N = std::min(xy_vec.size(), (std::size_t)2);
          for (std::size_t i = 0; i < N; ++i)
          {
            translation[i] = xy_vec[i];
          }
        }

        const auto rotation_yaml = transform_yaml["rotation"];
        double rotation = 0.0;
        if (rotation_yaml)
        {
          rotation = rotation_yaml.as<double>();
        }

        const auto scale_yaml = transform_yaml["scale"];
        double scale = 1.0;
        if (scale_yaml)
        {
          scale = scale_yaml.as<double>();
        }

        tf_dict->insert({map, Transformation(rotation, scale, translation)});
      }
    }
  }

  bool default_responsive_wait = false;
  const YAML::Node& default_responsive_wait_yaml = rmf_fleet["responsive_wait"];
  if (default_responsive_wait_yaml)
  {
    default_responsive_wait = default_responsive_wait_yaml.as<bool>();
  }

  double default_max_merge_waypoint_distance = 1e-3;
  const YAML::Node& default_max_merge_waypoint_distance_yaml =
    rmf_fleet["max_merge_waypoint_distance"];
  if (default_max_merge_waypoint_distance_yaml)
  {
    default_max_merge_waypoint_distance =
      default_max_merge_waypoint_distance_yaml.as<double>();
  }

  double default_max_merge_lane_distance = 0.3;
  const YAML::Node& default_max_merge_lane_distance_yaml =
    rmf_fleet["max_merge_lane_distance"];
  if (default_max_merge_lane_distance_yaml)
  {
    default_max_merge_lane_distance =
      default_max_merge_lane_distance_yaml.as<double>();
  }

  double default_min_lane_length = 1e-8;
  const YAML::Node& default_min_lane_length_yaml = rmf_fleet["min_lane_length"];
  if (default_min_lane_length_yaml)
  {
    default_min_lane_length =
      default_min_lane_length_yaml.as<double>();
  }

  std::unordered_map<std::string,
    RobotConfiguration> known_robot_configurations;
  const YAML::Node& robots = rmf_fleet["robots"];
  if (robots)
  {
    if (!robots.IsMap())
    {
      std::cerr
        << "[robots] element is not a map in config file [" << config_file
        << "] so we cannot parse any known robot configurations." << std::endl;
      return std::nullopt;
    }
    else
    {
      for (const auto& robot : robots)
      {
        const auto robot_name = robot.first.as<std::string>();
        const YAML::Node& robot_config_yaml = robot.second;

        if (!robot_config_yaml.IsMap() && !robot_config_yaml.IsNull())
        {
          std::cerr
            << "Entry for [" << robot_name << "] in [robots] dictionary is not "
            << "a dictionary, so it cannot be parsed." << std::endl;
          return std::nullopt;
        }

        if (robot_config_yaml.IsMap())
        {
          std::vector<std::string> chargers;
          const YAML::Node& charger_yaml = robot_config_yaml["charger"];
          if (charger_yaml)
          {
            chargers.push_back(charger_yaml.as<std::string>());
          }

          const YAML::Node& responsive_wait_yaml =
            robot_config_yaml["responsive_wait"];
          std::optional<bool> responsive_wait = std::nullopt;
          if (responsive_wait_yaml)
          {
            responsive_wait = responsive_wait_yaml.as<bool>();
          }

          const YAML::Node& max_merge_waypoint_distance_yaml =
            robot_config_yaml["max_merge_waypoint_distance"];
          std::optional<double> max_merge_waypoint_distance = std::nullopt;
          if (max_merge_waypoint_distance_yaml)
          {
            max_merge_waypoint_distance =
              max_merge_waypoint_distance_yaml.as<double>();
          }

          const YAML::Node& max_merge_lane_distance_yaml =
            robot_config_yaml["max_merge_lane_distance"];
          std::optional<double> max_merge_lane_distance = std::nullopt;
          if (max_merge_lane_distance_yaml)
          {
            max_merge_lane_distance = max_merge_lane_distance_yaml.as<double>();
          }

          const YAML::Node& min_lane_length_yaml =
            robot_config_yaml["min_lane_length"];
          std::optional<double> min_lane_length = std::nullopt;
          if (min_lane_length_yaml)
          {
            min_lane_length = min_lane_length_yaml.as<double>();
          }

          auto config = RobotConfiguration(
            std::move(chargers),
            responsive_wait,
            max_merge_waypoint_distance,
            max_merge_lane_distance,
            min_lane_length);
          known_robot_configurations.insert_or_assign(robot_name, config);
        }
        else
        {
          auto config = RobotConfiguration({});
          known_robot_configurations.insert_or_assign(robot_name, config);
        }
      }
    }
  }

  std::unordered_map<std::string, std::string> lift_emergency_levels;
  const YAML::Node& lift_emergency_levels_yaml =
    rmf_fleet["lift_emergency_levels"];
  if (lift_emergency_levels_yaml)
  {
    if (!lift_emergency_levels_yaml.IsMap())
    {
      std::cerr
        << "[lift_emergency_levels] element is not a map in the config file ["
        << config_file << "] so we cannot parse what level each lift will go "
        << "to in an emergency." << std::endl;
      return std::nullopt;
    }
    else
    {
      for (const auto& lift : lift_emergency_levels_yaml)
      {
        auto lift_name = lift.first.as<std::string>();
        auto level_name = lift.second.as<std::string>();
        lift_emergency_levels[std::move(lift_name)] = std::move(level_name);
      }
    }
  }

  auto config = FleetConfiguration(
    fleet_name,
    std::move(tf_dict),
    std::move(known_robot_configurations),
    std::move(traits),
    std::make_shared<rmf_traffic::agv::Graph>(std::move(graph)),
    battery_system,
    motion_sink,
    ambient_sink,
    tool_sink,
    recharge_threshold,
    recharge_soc,
    account_for_battery_drain,
    task_consideration,
    action_consideration,
    finishing_request,
    skip_rotation_commands,
    server_uri,
    rmf_traffic::time::from_seconds(max_delay),
    rmf_traffic::time::from_seconds(update_interval),
    default_responsive_wait,
    default_max_merge_waypoint_distance,
    default_max_merge_lane_distance,
    default_min_lane_length);
  config.change_lift_emergency_levels() = lift_emergency_levels;
  config.set_retreat_to_charger_interval(retreat_to_charger_interval);
  return config;
}

//==============================================================================
const std::string& EasyFullControl::FleetConfiguration::fleet_name() const
{
  return _pimpl->fleet_name;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_fleet_name(std::string value)
{
  _pimpl->fleet_name = std::move(value);
}

//==============================================================================
const std::optional<TransformDictionary>&
EasyFullControl::FleetConfiguration::transformations_to_robot_coordinates()
const
{
  return _pimpl->transformations_to_robot_coordinates;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::add_robot_coordinate_transformation(
  std::string map,
  Transformation transformation)
{
  if (!_pimpl->transformations_to_robot_coordinates.has_value())
  {
    _pimpl->transformations_to_robot_coordinates = TransformDictionary();
  }

  _pimpl->transformations_to_robot_coordinates
  ->insert_or_assign(std::move(map), transformation);
}

//==============================================================================
auto EasyFullControl::FleetConfiguration::known_robot_configurations() const
-> const std::unordered_map<std::string, RobotConfiguration>&
{
  return _pimpl->known_robot_configurations;
}

//==============================================================================
std::vector<std::string>
EasyFullControl::FleetConfiguration::known_robots() const
{
  std::vector<std::string> names;
  for (const auto& [name, _] : _pimpl->known_robot_configurations)
  {
    names.push_back(name);
  }
  return names;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::add_known_robot_configuration(
  std::string robot_name,
  RobotConfiguration configuration)
{
  _pimpl->known_robot_configurations.insert_or_assign(
    std::move(robot_name), std::move(configuration));
}

//==============================================================================
auto EasyFullControl::FleetConfiguration::get_known_robot_configuration(
  const std::string& robot_name) const -> std::optional<RobotConfiguration>
{
  const auto r_it = _pimpl->known_robot_configurations.find(robot_name);
  if (r_it == _pimpl->known_robot_configurations.end())
    return std::nullopt;

  return r_it->second;
}

//==============================================================================
auto EasyFullControl::FleetConfiguration::vehicle_traits() const
-> const std::shared_ptr<const VehicleTraits>&
{
  return _pimpl->traits;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_vehicle_traits(
  std::shared_ptr<const VehicleTraits> value)
{
  _pimpl->traits = std::move(value);
}

//==============================================================================
auto EasyFullControl::FleetConfiguration::graph() const
-> const std::shared_ptr<const Graph>&
{
  return _pimpl->graph;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_graph(
  std::shared_ptr<const Graph> value)
{
  _pimpl->graph = std::move(value);
}

//==============================================================================
rmf_battery::agv::ConstBatterySystemPtr
EasyFullControl::FleetConfiguration::battery_system() const
{
  return _pimpl->battery_system;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_battery_system(
  rmf_battery::agv::ConstBatterySystemPtr value)
{
  _pimpl->battery_system = std::move(value);
}

//==============================================================================
rmf_battery::ConstMotionPowerSinkPtr
EasyFullControl::FleetConfiguration::motion_sink() const
{
  return _pimpl->motion_sink;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_motion_sink(
  rmf_battery::ConstMotionPowerSinkPtr value)
{
  _pimpl->motion_sink = std::move(value);
}

//==============================================================================
rmf_battery::ConstDevicePowerSinkPtr
EasyFullControl::FleetConfiguration::ambient_sink() const
{
  return _pimpl->ambient_sink;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_ambient_sink(
  rmf_battery::ConstDevicePowerSinkPtr value)
{
  _pimpl->ambient_sink = std::move(value);
}

//==============================================================================
rmf_battery::ConstDevicePowerSinkPtr
EasyFullControl::FleetConfiguration::tool_sink() const
{
  return _pimpl->tool_sink;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_tool_sink(
  rmf_battery::ConstDevicePowerSinkPtr value)
{
  _pimpl->tool_sink = std::move(value);
}

//==============================================================================
double EasyFullControl::FleetConfiguration::recharge_threshold() const
{
  return _pimpl->recharge_threshold;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_recharge_threshold(double value)
{
  _pimpl->recharge_threshold = value;
}

//==============================================================================
double EasyFullControl::FleetConfiguration::recharge_soc() const
{
  return _pimpl->recharge_soc;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_recharge_soc(double value)
{
  _pimpl->recharge_soc = value;
}

//==============================================================================
bool EasyFullControl::FleetConfiguration::account_for_battery_drain() const
{
  return _pimpl->account_for_battery_drain;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_account_for_battery_drain(
  bool value)
{
  _pimpl->account_for_battery_drain = value;
}

//==============================================================================
std::optional<rmf_traffic::Duration>
EasyFullControl::FleetConfiguration::retreat_to_charger_interval() const
{
  return _pimpl->retreat_to_charger_interval;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_retreat_to_charger_interval(
  std::optional<rmf_traffic::Duration> value)
{
  _pimpl->retreat_to_charger_interval = value;
}

//==============================================================================
const std::unordered_map<std::string, ConsiderRequest>&
EasyFullControl::FleetConfiguration::task_consideration() const
{
  return _pimpl->task_consideration;
}

//==============================================================================
std::unordered_map<std::string, ConsiderRequest>&
EasyFullControl::FleetConfiguration::task_consideration()
{
  return _pimpl->task_consideration;
}

//==============================================================================
const std::unordered_map<std::string, ConsiderRequest>&
EasyFullControl::FleetConfiguration::action_consideration() const
{
  return _pimpl->action_consideration;
}

//==============================================================================
std::unordered_map<std::string, ConsiderRequest>&
EasyFullControl::FleetConfiguration::action_consideration()
{
  return _pimpl->action_consideration;
}

//==============================================================================
rmf_task::ConstRequestFactoryPtr
EasyFullControl::FleetConfiguration::finishing_request() const
{
  return _pimpl->finishing_request;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_finishing_request(
  rmf_task::ConstRequestFactoryPtr value)
{
  _pimpl->finishing_request = std::move(value);
}

//==============================================================================
bool EasyFullControl::FleetConfiguration::skip_rotation_commands() const
{
  return _pimpl->skip_rotation_commands;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_skip_rotation_commands(bool value)
{
  _pimpl->skip_rotation_commands = value;
}

//==============================================================================
std::optional<std::string> EasyFullControl::FleetConfiguration::server_uri()
const
{
  return _pimpl->server_uri;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_server_uri(
  std::optional<std::string> value)
{
  _pimpl->server_uri = std::move(value);
}

//==============================================================================
rmf_traffic::Duration EasyFullControl::FleetConfiguration::max_delay() const
{
  return _pimpl->max_delay;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_max_delay(
  rmf_traffic::Duration value)
{
  _pimpl->max_delay = value;
}

//==============================================================================
rmf_traffic::Duration EasyFullControl::FleetConfiguration::update_interval()
const
{
  return _pimpl->update_interval;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_update_interval(
  rmf_traffic::Duration value)
{
  _pimpl->update_interval = value;
}

//==============================================================================
bool EasyFullControl::FleetConfiguration::default_responsive_wait() const
{
  return _pimpl->default_responsive_wait;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_default_responsive_wait(
  bool enable)
{
  _pimpl->default_responsive_wait = enable;
}

//==============================================================================
double EasyFullControl::FleetConfiguration::default_max_merge_waypoint_distance()
const
{
  return _pimpl->default_max_merge_waypoint_distance;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::
set_default_max_merge_waypoint_distance(double distance)
{
  _pimpl->default_max_merge_waypoint_distance = distance;
}

//==============================================================================
double EasyFullControl::FleetConfiguration::default_max_merge_lane_distance()
const
{
  return _pimpl->default_max_merge_lane_distance;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_default_max_merge_lane_distance(
  double distance)
{
  _pimpl->default_max_merge_lane_distance = distance;
}

//==============================================================================
double EasyFullControl::FleetConfiguration::default_min_lane_length() const
{
  return _pimpl->default_min_lane_length;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_default_min_lane_length(
  double distance)
{
  _pimpl->default_min_lane_length = distance;
}

//==============================================================================
void EasyFullControl::FleetConfiguration::set_lift_emergency_level(
  std::string lift_name,
  std::string emergency_level_name)
{
  _pimpl->lift_emergency_levels[std::move(lift_name)] =
    std::move(emergency_level_name);
}

//==============================================================================
std::unordered_map<std::string, std::string>&
EasyFullControl::FleetConfiguration::change_lift_emergency_levels()
{
  return _pimpl->lift_emergency_levels;
}

//==============================================================================
const std::unordered_map<std::string, std::string>&
EasyFullControl::FleetConfiguration::lift_emergency_levels() const
{
  return _pimpl->lift_emergency_levels;
}

//==============================================================================
using EasyCommandHandlePtr = std::shared_ptr<EasyCommandHandle>;

//==============================================================================
std::shared_ptr<FleetUpdateHandle> EasyFullControl::more()
{
  return _pimpl->fleet_handle;
}

//==============================================================================
std::shared_ptr<const FleetUpdateHandle> EasyFullControl::more() const
{
  return _pimpl->fleet_handle;
}

//==============================================================================
auto EasyFullControl::add_robot(
  std::string robot_name,
  RobotState initial_state,
  RobotConfiguration configuration,
  RobotCallbacks callbacks) -> std::shared_ptr<EasyRobotUpdateHandle>
{
  const auto node = _pimpl->node();

  auto worker =
    FleetUpdateHandle::Implementation::get(*_pimpl->fleet_handle).worker;
  auto robot_nav_params = std::make_shared<NavParams>(_pimpl->nav_params);
  auto easy_updater = EasyRobotUpdateHandle::Implementation::make(
    robot_nav_params, worker);

  LocalizationRequest localization = nullptr;
  if (callbacks.localize())
  {
    localization = [
      inner = callbacks.localize(),
      nav_params = robot_nav_params
      ](Destination estimate, CommandExecution execution)
      {
        auto robot_position = nav_params->to_robot_coordinates(
          estimate.map(),
          estimate.position());
        if (robot_position.has_value())
        {
          auto transformed_estimate = estimate;
          Destination::Implementation::get(transformed_estimate)
          .position = *robot_position;

          inner(transformed_estimate, execution);
        }
      };
  }

  const auto& fleet_impl =
    FleetUpdateHandle::Implementation::get(*_pimpl->fleet_handle);
  const auto& planner = *fleet_impl.planner;
  const auto& graph = planner->get_configuration().graph();
  const auto& traits = planner->get_configuration().vehicle_traits();
  const auto& fleet_name = _pimpl->fleet_handle->fleet_name();

  RCLCPP_INFO(
    node->get_logger(),
    "Adding robot [%s] to fleet [%s].",
    robot_name.c_str(),
    fleet_name.c_str());

  auto insertion = _pimpl->cmd_handles.insert({robot_name, nullptr});
  if (!insertion.second)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Robot [%s] was previously added to fleet [%s]. Ignoring request...",
      robot_name.c_str(),
      fleet_name.c_str());
    return nullptr;
  }

  if (configuration.compatible_chargers().size() > 1)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Robot [%s] is configured to have %lu chargers, but we will only make "
      "use of the first one. Making use of multiple chargers will be added in "
      "a future version of RMF.",
      robot_name.c_str(),
      configuration.compatible_chargers().size());
  }

  std::optional<std::size_t> charger_index;
  if (!configuration.compatible_chargers().empty())
  {
    const auto& charger_name = configuration.compatible_chargers().front();
    const auto* charger_wp = graph.find_waypoint(charger_name);
    if (!charger_wp)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Cannot find a waypoint named [%s] in the navigation graph of fleet "
        "[%s] needed for the charging point of robot [%s]. We will not add the "
        "robot to the fleet.",
        charger_name.c_str(),
        fleet_name.c_str(),
        robot_name.c_str());

      _pimpl->cmd_handles.erase(robot_name);
      return nullptr;
    }

    charger_index = charger_wp->index();
  }

  rmf_traffic::Time now = std::chrono::steady_clock::time_point(
    std::chrono::nanoseconds(node->now().nanoseconds()));

  const auto position_opt = robot_nav_params->to_rmf_coordinates(
    initial_state.map(), initial_state.position());
  if (!position_opt.has_value())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "[EasyFullControl] Unable to find a robot transform for map [%s] while "
      "adding robot [%s]. We cannot initialize this robot.",
      initial_state.map().c_str(),
      robot_name.c_str());

    _pimpl->cmd_handles.erase(robot_name);
    return nullptr;
  }

  if (configuration.max_merge_waypoint_distance().has_value())
  {
    robot_nav_params->max_merge_waypoint_distance =
      *configuration.max_merge_waypoint_distance();
  }
  if (configuration.max_merge_lane_distance().has_value())
  {
    robot_nav_params->max_merge_lane_distance =
      *configuration.max_merge_lane_distance();
  }

  if (configuration.min_lane_length().has_value())
  {
    robot_nav_params->min_lane_length = *configuration.min_lane_length();
  }

  robot_nav_params->find_stacked_vertices(graph);
  const Eigen::Vector3d position = *position_opt;
  auto starts = robot_nav_params->compute_plan_starts(
    graph, initial_state.map(), position, now);

  if (starts.empty())
  {
    const auto& p = position;
    RCLCPP_ERROR(
      node->get_logger(),
      "Unable to compute a location on the navigation graph for robot [%s] "
      "being added to fleet [%s] using map [%s] and position [%.3f, %.3f, %.3f] "
      "specified in the initial_state argument. This can happen if the map in "
      "initial_state does not match any of the map names in the navigation "
      "graph supplied or if the position reported in the initial_state is far "
      "way from the navigation graph. This robot will not be added to the "
      "fleet.",
      robot_name.c_str(),
      fleet_name.c_str(),
      initial_state.map().c_str(),
      p[0], p[1], p[2]);

    _pimpl->cmd_handles.erase(robot_name);
    return nullptr;
  }

  const auto handle_nav_request = callbacks.navigate();
  const auto handle_stop = callbacks.stop();
  if (handle_nav_request == nullptr || handle_stop == nullptr)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "One or more required callbacks given to [EasyFullControl::add_robot] "
      "were null. The robot [%s] will not be added to fleet [%s].",
      robot_name.c_str(),
      _pimpl->fleet_handle->fleet_name().c_str());

    _pimpl->cmd_handles.erase(robot_name);
    return nullptr;
  }

  const auto cmd_handle = std::make_shared<EasyCommandHandle>(
    robot_nav_params,
    std::move(handle_nav_request),
    std::move(handle_stop));
  insertion.first->second = cmd_handle;

  bool enable_responsive_wait = _pimpl->default_responsive_wait;
  if (configuration.responsive_wait().has_value())
  {
    enable_responsive_wait = *configuration.responsive_wait();
  }

  _pimpl->fleet_handle->add_robot(
    insertion.first->second,
    robot_name,
    traits.profile(),
    starts,
    [
      cmd_handle,
      easy_updater,
      node,
      robot_name = robot_name,
      fleet_name = fleet_name,
      charger_index,
      action_executor = callbacks.action_executor(),
      localization = std::move(localization),
      nav_params = robot_nav_params,
      enable_responsive_wait
    ](const RobotUpdateHandlePtr& updater)
    {
      auto context = RobotUpdateHandle::Implementation::get(*updater)
      .get_context();

      context->worker().schedule(
        [
          cmd_handle,
          easy_updater,
          node,
          handle = updater,
          robot_name,
          fleet_name,
          charger_index,
          action_executor,
          localization,
          context,
          nav_params,
          enable_responsive_wait
        ](const auto&)
        {
          cmd_handle->w_context = context;
          context->set_nav_params(nav_params);
          EasyRobotUpdateHandle::Implementation::get(*easy_updater)
          .updater->handle = handle;
          handle->set_action_executor(action_executor);
          context->set_localization(localization);
          if (charger_index.has_value())
          {
            handle->set_charger_waypoint(*charger_index);
          }
          handle->enable_responsive_wait(enable_responsive_wait);

          RCLCPP_INFO(
            node->get_logger(),
            "Successfully added robot [%s] to the fleet [%s].",
            robot_name.c_str(),
            fleet_name.c_str());
        });
    });

  return easy_updater;
}

} // namespace agv
} // namespace rmf_fleet_adapter
