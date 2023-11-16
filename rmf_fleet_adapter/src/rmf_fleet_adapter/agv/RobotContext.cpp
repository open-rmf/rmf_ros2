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

#include "internal_RobotUpdateHandle.hpp"

#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_traffic/schedule/StubbornNegotiator.hpp>

#include <rmf_fleet_msgs/msg/robot_mode.hpp>

#include <rmf_utils/math.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
void NavParams::search_for_location(
  const std::string& map,
  Eigen::Vector3d position,
  RobotContext& context)
{
  auto planner = context.planner();
  if (!planner)
  {
    RCLCPP_ERROR(
      context.node()->get_logger(),
      "Planner unavailable for robot [%s], cannot update its location",
      context.requester_id().c_str());
    return;
  }
  const auto& graph = planner->get_configuration().graph();
  const auto now = context.now();
  auto starts = compute_plan_starts(graph, map, position, now);
  if (!starts.empty())
  {
    if (context.debug_positions)
    {
      std::stringstream ss;
      ss << __FILE__ << "|" << __LINE__ << ": " << starts.size()
        << " starts:" << print_starts(starts, graph);
      std::cout << ss.str() << std::endl;
    }
    context.set_location(std::move(starts));
  }
  else
  {
    if (context.debug_positions)
    {
      std::cout << __FILE__ << "|" << __LINE__ << ": setting robot to LOST | "
        << map << " <" << position.block<2, 1>(0, 0).transpose()
        << "> orientation " << position[2] * 180.0 / M_PI << std::endl;
    }
    context.set_lost(Location { now, map, position });
  }
}

//==============================================================================
void NavParams::find_stacked_vertices(const rmf_traffic::agv::Graph& graph)
{
  for (std::size_t i = 0; i < graph.num_waypoints() - 1; ++i)
  {
    const auto& wp_i = graph.get_waypoint(i);
    const Eigen::Vector2d p_i = wp_i.get_location();
    const std::string& map_i = wp_i.get_map_name();
    for (std::size_t j = i+1; j < graph.num_waypoints(); ++j)
    {
      const auto& wp_j = graph.get_waypoint(j);
      const Eigen::Vector2d p_j = wp_j.get_location();
      const std::string& map_j = wp_j.get_map_name();
      if (map_i != map_j)
      {
        continue;
      }

      const double dist = (p_i - p_j).norm();
      if (dist > max_merge_waypoint_distance)
      {
        continue;
      }

      // stack these waypoints
      auto stack_i = stacked_vertices[i];
      auto stack_j = stacked_vertices[j];
      if (!stack_i && !stack_j)
      {
        // create a new stack
        stack_i = std::make_shared<std::unordered_set<std::size_t>>();
        stack_j = stack_i;
      }
      else if (stack_i && stack_j)
      {
        if (stack_i != stack_j)
        {
          for (const std::size_t other : *stack_j)
          {
            stack_i->insert(other);
            stacked_vertices[other] = stack_i;
          }

        }
      }
      else if (!stack_i)
      {
        stack_i = stack_j;
      }

      assert(stack_i);
      stack_i->insert(i);
      stack_i->insert(j);
      stacked_vertices[i] = stack_i;
      stacked_vertices[j] = stack_j;
    }
  }
}

//==============================================================================
rmf_traffic::agv::Plan::StartSet NavParams::process_locations(
  const rmf_traffic::agv::Graph& graph,
  rmf_traffic::agv::Plan::StartSet locations) const
{
  return _lift_boundary_filter(graph, _descend_stacks(graph, locations));
}

//==============================================================================
rmf_traffic::agv::Plan::StartSet NavParams::_descend_stacks(
  const rmf_traffic::agv::Graph& graph,
  rmf_traffic::agv::Plan::StartSet locations) const
{
  for (rmf_traffic::agv::Plan::Start& location : locations)
  {
    std::optional<std::size_t> waypoint_opt;
    if (location.lane().has_value())
    {
      const rmf_traffic::agv::Graph::Lane& lane =
        graph.get_lane(*location.lane());
      waypoint_opt = lane.entry().waypoint_index();
    }
    else
    {
      waypoint_opt = location.waypoint();
    }

    if (!waypoint_opt.has_value())
      continue;

    const std::size_t original_waypoint = waypoint_opt.value();
    std::size_t waypoint = original_waypoint;
    const auto s_it = stacked_vertices.find(waypoint);
    VertexStack stack;
    if (s_it != stacked_vertices.end())
      stack = s_it->second;
    if (!stack)
      continue;

    std::unordered_set<std::size_t> visited;
    bool can_descend = true;
    bool has_loop = false;
    while (can_descend)
    {
      can_descend = false;
      if (!visited.insert(waypoint).second)
      {
        // These stacked vertices have a loop so there's no way to find a bottom
        // for it. We need to just exit here.
        has_loop = true;
        break;
      }

      for (std::size_t v : *stack)
      {
        if (graph.lane_from(v, waypoint))
        {
          // std::cout << "descending " << v << " -> " << waypoint << std::endl;
          waypoint = v;
          can_descend = true;
          break;
        }
      }
    }

    if (has_loop)
    {
      continue;
    }

    // Transfer the location estimate over to the waypoint that's at the bottom
    // of the vertex stack.
    // std::cout << "Descended vertex stack to " << waypoint << std::endl;
    if (waypoint != original_waypoint)
    {
      location.lane(std::nullopt);
      location.waypoint(waypoint);
    }
  }

  return locations;
}

//==============================================================================
rmf_traffic::agv::Plan::StartSet NavParams::_lift_boundary_filter(
  const rmf_traffic::agv::Graph& graph,
  rmf_traffic::agv::Plan::StartSet locations) const
{
  const auto r_it = std::remove_if(
    locations.begin(),
    locations.end(),
    [&graph](const rmf_traffic::agv::Plan::Start& location)
    {
      if (location.lane().has_value())
      {
        // If the intention is to move along a lane, then it is okay to keep
        // this. If the lane is entering or exiting the lift, then it should
        // have the necessary events.
        // TODO(@mxgrey): Should we check and make sure that the event is
        // actually present?
        return false;
      }

      if (!location.location().has_value())
      {
        // If the robot is perfectly on some waypoint then there is no need to
        // filter.
        return false;
      }

      const Eigen::Vector2d p = location.location().value();

      const auto robot_inside_lift = [&]()
        -> rmf_traffic::agv::Graph::LiftPropertiesPtr
        {
          for (const auto& lift : graph.known_lifts())
          {
            // We assume lifts never overlap so we will return the first
            // positive hit.
            if (lift->is_in_lift(p))
              return lift;
          }

          return nullptr;
        }();

      const auto& wp = graph.get_waypoint(location.waypoint());
      const auto lift = wp.in_lift();
      // If the robot's lift status and the waypoint's lift status don't match
      // then we should filter this waypoint out.
      return wp.in_lift() != robot_inside_lift;
    });

  locations.erase(r_it, locations.end());
  return locations;
}

//==============================================================================
bool NavParams::in_same_stack(
  std::size_t wp0,
  std::size_t wp1) const
{
  if (wp0 == wp1)
  {
    return true;
  }

  const auto s_it = stacked_vertices.find(wp0);
  if (s_it == stacked_vertices.end())
    return false;

  const auto stack = s_it->second;
  if (!stack)
    return false;

  return stack->count(wp1);
}

//==============================================================================
std::shared_ptr<RobotCommandHandle> RobotContext::command()
{
  return _command_handle.lock();
}

//==============================================================================
std::shared_ptr<RobotUpdateHandle> RobotContext::make_updater()
{
  return RobotUpdateHandle::Implementation::make(shared_from_this());
}

//==============================================================================
Eigen::Vector3d RobotContext::position() const
{
  if (!_location.empty())
  {
    const auto& l = _location.front();
    if (l.location().has_value())
    {
      const Eigen::Vector2d& p = *l.location();
      return {p[0], p[1], l.orientation()};
    }

    const Eigen::Vector2d& p =
      navigation_graph().get_waypoint(l.waypoint()).get_location();
    return {p[0], p[1], l.orientation()};
  }
  else if (_lost.has_value() && _lost->location.has_value())
  {
    return _lost->location->position;
  }

  throw std::runtime_error(
    "No location information is available for [" + requester_id() + "]");
}

//==============================================================================
const std::string& RobotContext::map() const
{
  if (!_location.empty())
  {
    return navigation_graph()
      .get_waypoint(_location.front().waypoint()).get_map_name();
  }
  else if (_lost.has_value() && _lost->location.has_value())
  {
    return _lost->location->map;
  }

  throw std::runtime_error(
    "No location information is available for [" + requester_id() + "]");
}

//==============================================================================
rmf_traffic::Time RobotContext::now() const
{
  return rmf_traffic_ros2::convert(_node->now());
}

//==============================================================================
std::function<rmf_traffic::Time()> RobotContext::clock() const
{
  return [self = shared_from_this()]() { return self->now(); };
}

//==============================================================================
const rmf_traffic::agv::Plan::StartSet& RobotContext::location() const
{
  return _location;
}

//==============================================================================
void RobotContext::set_location(rmf_traffic::agv::Plan::StartSet location_)
{
  for (auto& location : location_)
  {
    location.orientation(rmf_utils::wrap_to_pi(location.orientation()));
  }

  _location = std::move(location_);
  filter_closed_lanes();

  // const auto& graph = navigation_graph();
  // for (const auto& l : _location)
  // {
  //   std::cout << " -- ";
  //   if (l.lane().has_value())
  //   {
  //     std::cout << "lane[" << *l.lane() << "] ";
  //     Printer printer;
  //     const auto& lane = graph.get_lane(*l.lane());
  //     if (lane.entry().event())
  //     {
  //       std::cout << " [entry ";
  //       lane.entry().event()->execute(printer);
  //       std::cout << "] ";
  //     }
  //     const auto& i_wp0 = lane.entry().waypoint_index();
  //     const auto& wp0 = graph.get_waypoint(i_wp0);
  //     const auto& i_wp1 = lane.exit().waypoint_index();
  //     const auto& wp1 = graph.get_waypoint(i_wp1);
  //     std::cout << i_wp0 << " [" << wp0.get_map_name() << ":" << wp0.get_location().transpose() << "] ";
  //     if (const auto lift = wp0.in_lift())
  //       std::cout << "{" << lift->name() << "} ";
  //     std::cout << "-> " << i_wp1 << " [" << wp1.get_map_name() << ":" << wp1.get_location().transpose() << "] ";
  //     if (const auto lift = wp1.in_lift())
  //       std::cout << "{" << lift->name() << "} ";

  //     if (lane.exit().event())
  //     {
  //       std::cout << "[exit ";
  //       lane.exit().event()->execute(printer);
  //       std::cout << "]";
  //     }
  //     std::cout << " | ";
  //   }
  //   std::cout << l.waypoint();
  //   const auto& wp = graph.get_waypoint(l.waypoint());
  //   if (const auto lift = wp.in_lift())
  //   {
  //     std::cout << " {" << lift->name() << "}";
  //   }
  //   if (l.location().has_value())
  //   {
  //     std::cout << " | " << l.location()->transpose();
  //   }
  //   std::cout << std::endl;
  // }

  if (_location.empty())
  {
    if (debug_positions)
    {
      std::cout << __FILE__ << "|" << __LINE__ << ": setting robot to LOST" << std::endl;
    }
    set_lost(std::nullopt);
    return;
  }
  else if (_lost)
  {
    nlohmann::json resolve;
    resolve["robot"] = name();
    resolve["group"] = group();
    resolve["msg"] = "The robot [" + requester_id() + "] has found a "
      "connection to the navigation graph.";
    _lost->ticket->resolve(resolve);
    _lost = std::nullopt;
    // If the robot has switched from lost to found, we should go ahead and
    // replan.
    RCLCPP_INFO(
      node()->get_logger(),
      "Requesting a replan for [%s] because it has been found after being lost",
      requester_id().c_str());
    request_replan();
  }

  _most_recent_valid_location = _location;
}

//==============================================================================
const std::optional<Lost>& RobotContext::lost() const
{
  return _lost;
}

//==============================================================================
void RobotContext::set_lost(std::optional<Location> location)
{
  _location.clear();
  if (!_lost.has_value())
  {
    nlohmann::json detail;
    detail["robot"] = name();
    detail["group"] = group();
    detail["msg"] = "The robot [" + requester_id() + "] is too far from the "
      "navigation graph and may need an operator to help bring it back.";

    auto ticket = _reporting.create_issue(
      rmf_task::Log::Tier::Error, "lost", detail);
    _lost = Lost { location, std::move(ticket) };
  }
  else
  {
    _lost->location = location;
  }
}

//==============================================================================
void RobotContext::filter_closed_lanes()
{
  const rmf_traffic::agv::LaneClosure* closures = get_lane_closures();
  if (closures)
  {
    for (std::size_t i = 0; i < _location.size(); )
    {
      if (_location[i].lane().has_value())
      {
        if (closures->is_closed(*_location[i].lane()))
        {
          _location.erase(_location.begin() + i);
          continue;
        }
      }
      ++i;
    }
  }
}

//==============================================================================
const rmf_traffic::agv::LaneClosure* RobotContext::get_lane_closures() const
{
  if (_emergency)
  {
    if (const auto planner = *_emergency_planner)
    {
      return &planner->get_configuration().lane_closures();
    }
  }
  else
  {
    if (const auto planner = *_planner)
    {
      return &planner->get_configuration().lane_closures();
    }
  }

  return nullptr;
}

//==============================================================================
rmf_traffic::schedule::Participant& RobotContext::itinerary()
{
  return _itinerary;
}

//==============================================================================
const rmf_traffic::schedule::Participant& RobotContext::itinerary() const
{
  return _itinerary;
}

//==============================================================================
auto RobotContext::schedule() const -> const std::shared_ptr<const Mirror>&
{
  return _schedule;
}

//==============================================================================
const rmf_traffic::schedule::ParticipantDescription&
RobotContext::description() const
{
  return _itinerary.description();
}

//==============================================================================
const std::shared_ptr<const rmf_traffic::Profile>& RobotContext::profile() const
{
  return _profile;
}

//==============================================================================
const std::string& RobotContext::name() const
{
  return _itinerary.description().name();
}

//==============================================================================
const std::string& RobotContext::group() const
{
  return _itinerary.description().owner();
}

//==============================================================================
const std::string& RobotContext::requester_id() const
{
  return _requester_id;
}

//==============================================================================
const rmf_traffic::agv::Graph& RobotContext::navigation_graph() const
{
  return (*_planner)->get_configuration().graph();
}

//==============================================================================
const std::shared_ptr<const rmf_traffic::agv::Planner>&
RobotContext::planner() const
{
  return *_planner;
}

//==============================================================================
const std::shared_ptr<const rmf_traffic::agv::Planner>&
RobotContext::emergency_planner() const
{
  return *_emergency_planner;
}

//==============================================================================
std::shared_ptr<NavParams> RobotContext::nav_params() const
{
  return _nav_params;
}

//==============================================================================
void RobotContext::set_nav_params(std::shared_ptr<NavParams> value)
{
  _nav_params = std::move(value);
}

//==============================================================================
class RobotContext::NegotiatorLicense
{
public:

  NegotiatorLicense(
    std::shared_ptr<RobotContext> context,
    rmf_traffic::schedule::Negotiator* negotiator)
  : _context(context),
    _negotiator(negotiator)
  {
    // Do nothing
  }

  ~NegotiatorLicense()
  {
    const auto context = _context.lock();
    if (!context)
      return;

    if (context && context->_negotiator == _negotiator)
      context->_negotiator = nullptr;
  }

private:
  std::weak_ptr<RobotContext> _context;
  rmf_traffic::schedule::Negotiator* _negotiator;
};

//==============================================================================
auto RobotContext::set_negotiator(
  rmf_traffic::schedule::Negotiator* negotiator)
-> std::shared_ptr<NegotiatorLicense>
{
  _negotiator = negotiator;

  return std::make_shared<NegotiatorLicense>(
    shared_from_this(), negotiator);
}

//==============================================================================
std::shared_ptr<void> RobotContext::be_stubborn()
{
  return _stubbornness;
}

//==============================================================================
bool RobotContext::is_stubborn() const
{
  return _stubbornness.use_count() > 1;
}

//==============================================================================
const rxcpp::observable<RobotContext::Empty>&
RobotContext::observe_replan_request() const
{
  return _replan_obs;
}

//==============================================================================
const rxcpp::observable<RobotContext::Empty>&
RobotContext::observe_charging_change() const
{
  return _charging_change_obs;
}

//==============================================================================
void RobotContext::request_replan()
{
  _replan_publisher.get_subscriber().on_next(Empty{});
}

//==============================================================================
const rxcpp::observable<RobotContext::GraphChange>&
RobotContext::observe_graph_change() const
{
  return _graph_change_obs;
}

//==============================================================================
void RobotContext::notify_graph_change(GraphChange changes)
{
  filter_closed_lanes();
  _graph_change_publisher.get_subscriber().on_next(std::move(changes));
}

//==============================================================================
const std::shared_ptr<rmf_fleet_adapter::agv::Node>& RobotContext::node()
{
  return _node;
}

//==============================================================================
std::shared_ptr<const Node> RobotContext::node() const
{
  return _node;
}

//==============================================================================
const rxcpp::schedulers::worker& RobotContext::worker() const
{
  return _worker;
}

//==============================================================================
rmf_utils::optional<rmf_traffic::Duration> RobotContext::maximum_delay() const
{
  return _maximum_delay;
}

//==============================================================================
RobotContext& RobotContext::maximum_delay(
  rmf_utils::optional<rmf_traffic::Duration> value)
{
  _maximum_delay = value;
  return *this;
}

//==============================================================================
const rmf_task::ConstActivatorPtr& RobotContext::task_activator() const
{
  return _task_activator;
}

//==============================================================================
const rmf_task::ConstParametersPtr& RobotContext::task_parameters() const
{
  return _task_parameters;
}

//==============================================================================
const rmf_task::State& RobotContext::current_task_end_state() const
{
  return _current_task_end_state;
}

//==============================================================================
RobotContext& RobotContext::current_task_end_state(
  const rmf_task::State& state)
{
  _current_task_end_state = state;
  return *this;
}

//==============================================================================
std::function<rmf_task::State()> RobotContext::make_get_state()
{
  return [self = shared_from_this()]()
    {
      if (self->_most_recent_valid_location.empty())
      {
        throw std::runtime_error(
                "Missing a _most_recent_valid_location for robot ["
                + self->requester_id() + "]. This is an internal RMF error, "
                "please report it to the RMF developers.");
      }

      rmf_task::State state;
      state.load_basic(
        self->_most_recent_valid_location.front(),
        self->_charging_wp,
        self->_current_battery_soc);

      state.insert<GetContext>(GetContext{self->shared_from_this()});
      return state;
    };
}

//==============================================================================
const std::string* RobotContext::current_task_id() const
{
  if (_current_task_id.has_value())
    return &(*_current_task_id);

  return nullptr;
}

//==============================================================================
RobotContext& RobotContext::current_task_id(std::optional<std::string> id)
{
  std::unique_lock<std::mutex> lock(*_current_task_id_mutex);
  _current_task_id = std::move(id);
  _initial_time_idle_outside_lift = std::nullopt;
  return *this;
}

//==============================================================================
std::string RobotContext::copy_current_task_id() const
{
  std::unique_lock<std::mutex> lock(*_current_task_id_mutex);
  if (_current_task_id.has_value())
    return _current_task_id.value();

  return {};
}

//==============================================================================
double RobotContext::current_battery_soc() const
{
  return _current_battery_soc;
}

//==============================================================================
RobotContext& RobotContext::current_battery_soc(const double battery_soc)
{
  if (battery_soc < 0.0 || battery_soc > 1.0)
  {
    RCLCPP_ERROR(
      _node->get_logger(),
      "Invalid battery state of charge given for [%s]: %0.3f",
      requester_id().c_str(),
      battery_soc);
    return *this;
  }

  _current_battery_soc = battery_soc;
  _battery_soc_publisher.get_subscriber().on_next(battery_soc);

  return *this;
}

//==============================================================================
std::size_t RobotContext::dedicated_charging_wp() const
{
  return _charging_wp;
}

//==============================================================================
bool RobotContext::waiting_for_charger() const
{
  return _waiting_for_charger;
}

//==============================================================================
const rxcpp::observable<double>& RobotContext::observe_battery_soc() const
{
  return _battery_soc_obs;
}

//==============================================================================
const std::shared_ptr<const rmf_task::TaskPlanner>&
RobotContext::task_planner() const
{
  return _task_planner;
}

//==============================================================================
auto RobotContext::task_planner(
  const std::shared_ptr<const rmf_task::TaskPlanner> task_planner)
-> RobotContext&
{
  _task_planner = task_planner;
  return *this;
}

//==============================================================================
void RobotContext::set_lift_entry_watchdog(
  RobotUpdateHandle::Unstable::Watchdog watchdog,
  rmf_traffic::Duration wait_duration)
{
  _lift_watchdog = std::move(watchdog);
  _lift_rewait_duration = wait_duration;
}

//==============================================================================
const RobotUpdateHandle::Unstable::Watchdog&
RobotContext::get_lift_watchdog() const
{
  return _lift_watchdog;
}

//==============================================================================
rmf_traffic::Duration RobotContext::get_lift_rewait_duration() const
{
  return _lift_rewait_duration;
}

//==============================================================================
void RobotContext::respond(
  const TableViewerPtr& table_viewer,
  const ResponderPtr& responder)
{
  if (_negotiator && !is_stubborn())
    return _negotiator->respond(table_viewer, responder);

  // If there is no negotiator assigned for this robot or the stubborn mode has
  // been requested, then use a StubbornNegotiator.
  //
  // TODO(MXG): Consider if this should be scheduled on a separate thread
  // instead of executed immediately. The StubbornNegotiator doesn't do any
  // planning, so it should be able to finish quickly, but that should be
  // verified with benchmarks.
  rmf_traffic::schedule::StubbornNegotiator(_itinerary).respond(
    table_viewer, responder);
}

//==============================================================================
void RobotContext::current_mode(uint32_t mode)
{
  _current_mode = mode;
}

//==============================================================================
uint32_t RobotContext::current_mode() const
{
  return _current_mode;
}

//==============================================================================
void RobotContext::override_status(std::optional<std::string> status)
{
  _override_status = status;
}

//==============================================================================
std::optional<std::string> RobotContext::override_status() const
{
  return _override_status;
}

//==============================================================================
void RobotContext::action_executor(
  RobotUpdateHandle::ActionExecutor action_executor)
{
  if (action_executor == nullptr)
  {
    RCLCPP_WARN(
      _node->get_logger(),
      "ActionExecutor set to nullptr for robot [%s]. If this robot needs to "
      "perform an action as part of a task, a critical task error will be "
      "thrown.",
      this->name().c_str());
  }
  _action_executor = action_executor;
}

//==============================================================================
RobotUpdateHandle::ActionExecutor RobotContext::action_executor() const
{
  return _action_executor;
}

//==============================================================================
std::shared_ptr<TaskManager> RobotContext::task_manager()
{
  return _task_manager.lock();
}

//==============================================================================
bool RobotContext::is_commissioned() const
{
  return _commissioned;
}

//==============================================================================
void RobotContext::decommission()
{
  _commissioned = false;
}

//==============================================================================
void RobotContext::recommission()
{
  _commissioned = true;
}

//==============================================================================
Reporting& RobotContext::reporting()
{
  return _reporting;
}

//==============================================================================
const Reporting& RobotContext::reporting() const
{
  return _reporting;
}

//==============================================================================
bool RobotContext::localize(
  EasyFullControl::Destination estimate,
  EasyFullControl::CommandExecution execution) const
{
  if (_localize)
  {
    _localize(std::move(estimate), std::move(execution));
    return true;
  }

  return false;
}

//==============================================================================
void RobotContext::set_localization(
  EasyFullControl::LocalizationRequest localization)
{
  _localize = std::move(localization);
}

//==============================================================================
const LiftDestination* RobotContext::current_lift_destination() const
{
  return _lift_destination.get();
}

//==============================================================================
std::shared_ptr<void> RobotContext::set_lift_destination(
  std::string lift_name,
  std::string destination_floor,
  bool requested_from_inside)
{
  _lift_destination = std::make_shared<LiftDestination>(
    LiftDestination{
      std::move(lift_name),
      std::move(destination_floor),
      requested_from_inside
    });
  _initial_time_idle_outside_lift = std::nullopt;

  return _lift_destination;
}

//==============================================================================
void RobotContext::release_lift()
{
  _lift_destination = nullptr;
}

//==============================================================================
const std::string& RobotContext::locked_mutex_group() const
{
  return _locked_mutex_group.name;
}

//==============================================================================
void RobotContext::request_mutex_group(std::string group)
{
  if (group.empty())
  {
    release_mutex_group();
    return;
  }

  if (group == _requesting_mutex_group.name)
  {
    return;
  }

  _requesting_mutex_group = MutexGroupData{std::move(group), _node->now()};
  _publish_mutex_group_request();
}

//==============================================================================
void RobotContext::release_mutex_group()
{
  _release_mutex_group(_requesting_mutex_group);
  _release_mutex_group(_locked_mutex_group);
}

//==============================================================================
RobotContext::RobotContext(
  std::shared_ptr<RobotCommandHandle> command_handle,
  std::vector<rmf_traffic::agv::Plan::Start> _initial_location,
  rmf_traffic::schedule::Participant itinerary,
  std::shared_ptr<const Mirror> schedule,
  SharedPlanner planner,
  SharedPlanner emergency_planner,
  rmf_task::ConstActivatorPtr activator,
  rmf_task::ConstParametersPtr parameters,
  std::shared_ptr<rmf_fleet_adapter::agv::Node> node,
  const rxcpp::schedulers::worker& worker,
  rmf_utils::optional<rmf_traffic::Duration> maximum_delay,
  rmf_task::State state,
  std::shared_ptr<const rmf_task::TaskPlanner> task_planner)
: _command_handle(std::move(command_handle)),
  _location(std::move(_initial_location)),
  _itinerary(std::move(itinerary)),
  _schedule(std::move(schedule)),
  _planner(std::move(planner)),
  _emergency_planner(std::move(emergency_planner)),
  _task_activator(std::move(activator)),
  _task_parameters(std::move(parameters)),
  _stubbornness(std::make_shared<int>(0)),
  _node(std::move(node)),
  _worker(worker),
  _maximum_delay(maximum_delay),
  _requester_id(
    _itinerary.description().owner() + "/" + _itinerary.description().name()),
  _charging_wp(state.dedicated_charging_waypoint().value()),
  _current_task_end_state(state),
  _current_task_id(std::nullopt),
  _task_planner(std::move(task_planner)),
  _reporting(_worker)
{
  _most_recent_valid_location = _location;
  _profile = std::make_shared<rmf_traffic::Profile>(
    _itinerary.description().profile());

  _replan_obs = _replan_publisher.get_observable();
  _graph_change_obs = _graph_change_publisher.get_observable();
  _charging_change_obs = _charging_change_publisher.get_observable();

  _battery_soc_obs = _battery_soc_publisher.get_observable();

  _current_mode = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
  _override_status = std::nullopt;

  _action_executor = nullptr;
}

//==============================================================================
void RobotContext::_set_task_manager(std::shared_ptr<TaskManager> mgr)
{
  _task_manager = std::move(mgr);
}

//==============================================================================
void RobotContext::_set_negotiation_license(std::shared_ptr<void> license)
{
  _negotiation_license = std::move(license);
}

//==============================================================================
void RobotContext::_set_emergency(bool value)
{
  _emergency = value;
  if (_emergency)
  {
    filter_closed_lanes();
  }
}

//==============================================================================
void RobotContext::_set_charging(std::size_t wp, bool waiting_for_charger)
{
  _charging_wp = wp;
  _waiting_for_charger = waiting_for_charger;
  _charging_change_publisher.get_subscriber().on_next(Empty{});
}

//==============================================================================
void RobotContext::_check_lift_state(
  const rmf_lift_msgs::msg::LiftState& state)
{
  if (_lift_destination.use_count() < 2)
  {
    if (_lift_destination && !_lift_destination->requested_from_inside)
    {
      // The lift destination reference count dropping to one while the lift
      // destination request is on the outside means the task that prompted the
      // lift usage was cancelled while the robot was outside of the lift.
      // Therefore we should release the usage of the lift.
      _lift_destination = nullptr;
      _initial_time_idle_outside_lift = std::nullopt;
    }
    else if (_lift_destination && !_current_task_id.has_value())
    {
      const Eigen::Vector2d p = position().block<2, 1>(0, 0);
      const auto& graph = navigation_graph();
      const auto& known_lifts = graph.known_lifts();
      const auto l_it = std::find_if(
        known_lifts.begin(),
        known_lifts.end(),
        [&](const auto& lift)
        {
          return lift->name() == _lift_destination->lift_name;
        });

      bool inside_lift = false;
      if (l_it != graph.known_lifts().end())
      {
        inside_lift = (*l_it)->is_in_lift(p);
      }

      if (inside_lift)
      {
        _initial_time_idle_outside_lift = std::nullopt;
      }
      else
      {
        const auto now = std::chrono::steady_clock::now();
        if (_initial_time_idle_outside_lift.has_value())
        {
          const auto lapse = now - *_initial_time_idle_outside_lift;
          if (lapse > std::chrono::seconds(2))
          {
            RCLCPP_INFO(
              _node->get_logger(),
              "Releasing lift [%s] for robot [%s] because it has remained idle "
              "outside of the lift.",
              _lift_destination->lift_name.c_str(),
              requester_id().c_str());
          }
          _lift_destination = nullptr;
          _initial_time_idle_outside_lift = std::nullopt;
        }
        else
        {
          _initial_time_idle_outside_lift = now;
        }
      }
    }
  }

  if (state.session_id == requester_id())
  {
    if (!_lift_destination || _lift_destination->lift_name != state.lift_name)
    {
      rmf_lift_msgs::msg::LiftRequest msg;
      msg.lift_name = state.lift_name;
      msg.request_type = rmf_lift_msgs::msg::LiftRequest::REQUEST_END_SESSION;
      msg.session_id = requester_id();
      _node->lift_request()->publish(msg);
    }
  }

  if (!_lift_destination)
  {
    return;
  }

  rmf_lift_msgs::msg::LiftRequest msg;
  msg.lift_name = _lift_destination->lift_name;
  msg.destination_floor = _lift_destination->destination_floor;
  msg.session_id = requester_id();
  msg.request_time = _node->now();
  msg.request_type = rmf_lift_msgs::msg::LiftRequest::REQUEST_AGV_MODE;
  msg.door_state = rmf_lift_msgs::msg::LiftRequest::DOOR_OPEN;

  _node->lift_request()->publish(msg);
}

//==============================================================================
void RobotContext::_check_mutex_groups(
  const rmf_fleet_msgs::msg::MutexGroupStates& states)
{
  // Make sure to release any mutex groups that this robot is not trying to
  // lock right now.
  for (const auto& assignment : states.assignments)
  {
    if (assignment.claimed != requester_id())
      return;

    if (assignment.group == _requesting_mutex_group.name)
    {
      if (_locked_mutex_group.name != assignment.group)
      {
        _node->mutex_group_request()->publish(
          rmf_fleet_msgs::build<rmf_fleet_msgs::msg::MutexGroupRequest>()
            .group(_locked_mutex_group.name)
            .claimer(requester_id())
            .claim_time(_locked_mutex_group.claim_time)
            .mode(rmf_fleet_msgs::msg::MutexGroupRequest::MODE_RELEASE));
      }

      _locked_mutex_group = _requesting_mutex_group;
    }
    else if (assignment.group != _locked_mutex_group.name)
    {
      // This assignment does not match either the requested nor the currently
      // locked mutex group. This is an error so let's release it.
      _node->mutex_group_request()->publish(
        rmf_fleet_msgs::build<rmf_fleet_msgs::msg::MutexGroupRequest>()
        .group(assignment.group)
        .claimer(requester_id())
        .claim_time(assignment.claim_time)
        .mode(rmf_fleet_msgs::msg::MutexGroupRequest::MODE_RELEASE));
    }
  }
}

//==============================================================================
void RobotContext::_release_mutex_group(MutexGroupData& data)
{
  if (data.name.empty())
  {
    return;
  }

  _node->mutex_group_request()->publish(
    rmf_fleet_msgs::build<rmf_fleet_msgs::msg::MutexGroupRequest>()
    .group(data.name)
    .claimer(requester_id())
    .claim_time(data.claim_time)
    .mode(rmf_fleet_msgs::msg::MutexGroupRequest::MODE_RELEASE));
  data.name = "";
}

//==============================================================================
void RobotContext::_publish_mutex_group_request()
{
  const auto now = std::chrono::steady_clock::now();
  if (_current_task_id.has_value())
  {
    _last_active_task_time = now;
  }
  else
  {
    if (_last_active_task_time + std::chrono::seconds(10) < now)
    {
      // The robot has been idle for 10 seconds. It should not be keeping a
      // mutex locked; a task probably ended wrongly.
      if (!_requesting_mutex_group.name.empty()
          || !_locked_mutex_group.name.empty())
      {
        auto warning = [&](const std::string& name)
          {
            RCLCPP_ERROR(
              _node->get_logger(),
              "Forcibly releasing mutex group [%s] requested by robot [%s] "
              "because the robot has been idle for an excessive amount of time.",
              name.c_str(),
              requester_id().c_str());
          };

        if (!_requesting_mutex_group.name.empty())
        {
          warning(_requesting_mutex_group.name);
        }

        if (!_locked_mutex_group.name.empty())
        {
          warning(_locked_mutex_group.name);
        }

        release_mutex_group();
      }
    }
  }

  auto publish = [&](const MutexGroupData& data)
    {
      _node->mutex_group_request()->publish(
        rmf_fleet_msgs::build<rmf_fleet_msgs::msg::MutexGroupRequest>()
        .group(data.name)
        .claimer(requester_id())
        .claim_time(data.claim_time)
        .mode(rmf_fleet_msgs::msg::MutexGroupRequest::MODE_LOCK));
    };

  if (!_requesting_mutex_group.name.empty())
  {
    publish(_requesting_mutex_group);
  }

  if (!_locked_mutex_group.name.empty())
  {
    publish(_locked_mutex_group);
  }
}

} // namespace agv
} // namespace rmf_fleet_adapter
