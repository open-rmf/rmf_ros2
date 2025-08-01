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
#include "../TaskManager.hpp"

#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_traffic/schedule/StubbornNegotiator.hpp>

#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_door_msgs/msg/door_mode.hpp>

#include <rmf_utils/math.hpp>
#include <string>
#include <unordered_set>

namespace rmf_fleet_adapter {

//==============================================================================
DynamicEventCallbacks::~DynamicEventCallbacks()
{
  const auto c = context.lock();
  if (!c)
    return;

  c->worker().schedule([c](const auto&)
    {
      // Publish a dynamic event status update that reports the dynamic event
      // as non-active.
      c->_publish_dynamic_event_status(
        rmf_task::Event::State::Status::Uninitialized, 0);
    });
}

namespace agv {

//==============================================================================
std::unordered_map<std::size_t, VertexStack> compute_stacked_vertices(
  const rmf_traffic::agv::Graph& graph,
  double max_merge_waypoint_distance)
{
  std::unordered_map<std::size_t, VertexStack> stacked_vertices;
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

  return stacked_vertices;
}

//==============================================================================
void NavParams::search_for_location(
  const std::string& map,
  Eigen::Vector3d position,
  RobotContext& context)
{
  auto planner = context.planner();
  const auto& graph = context.navigation_graph();
  if (!planner)
  {
    RCLCPP_ERROR(
      context.node()->get_logger(),
      "Planner unavailable for robot [%s], cannot update its location",
      context.requester_id().c_str());
    return;
  }
  const auto now = context.now();
  auto starts = compute_plan_starts(context, map, position, now);
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

rmf_traffic::agv::Plan::StartSet NavParams::compute_plan_starts(
  const RobotContext& context,
  const std::string& map_name,
  const Eigen::Vector3d position,
  const rmf_traffic::Time start_time) const
{
  const auto& graph = context.navigation_graph();
  auto unfiltered = unfiltered_compute_plan_starts(
    graph, map_name, position, start_time);

  if (!unfiltered.empty())
    return process_locations(std::move(unfiltered), context);

  return {};
}

//==============================================================================
void NavParams::find_stacked_vertices(const rmf_traffic::agv::Graph& graph)
{
  stacked_vertices = compute_stacked_vertices(
    graph, max_merge_waypoint_distance);
}

//==============================================================================
std::string NavParams::get_vertex_name(
  const rmf_traffic::agv::Graph& graph,
  const std::optional<std::size_t> v_opt) const
{
  if (!v_opt.has_value())
    return "";

  const std::size_t v = v_opt.value();
  if (const std::string* n = graph.get_waypoint(v).name())
  {
    return *n;
  }
  else
  {
    const auto& stack_it = stacked_vertices.find(v);
    if (stack_it != stacked_vertices.end() && stack_it->second)
    {
      for (const auto& v : *stack_it->second)
      {
        if (const auto* n = graph.get_waypoint(v).name())
        {
          return *n;
        }
      }
    }
  }

  return "";
}

//==============================================================================
rmf_traffic::agv::Plan::StartSet NavParams::process_locations(
  rmf_traffic::agv::Plan::StartSet locations,
  const RobotContext& context) const
{
  return _strict_lane_filter(
    _lift_boundary_filter(
      _descend_stacks(
        context.navigation_graph(),
        locations),
      context));
}

//==============================================================================
rmf_traffic::agv::Plan::StartSet NavParams::_descend_stacks(
  const rmf_traffic::agv::Graph& graph,
  rmf_traffic::agv::Plan::StartSet locations) const
{
  std::vector<std::size_t> remove;
  for (std::size_t i = 0; i < locations.size(); ++i)
  {
    rmf_traffic::agv::Plan::Start& location = locations[i];
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
    if (waypoint != original_waypoint)
    {
      bool can_merge = true;
      if (const auto r_merge = graph.get_waypoint(waypoint).merge_radius())
      {
        if (const auto p_opt = location.location())
        {
          const auto p = p_opt.value();
          const auto p_wp = graph.get_waypoint(waypoint).get_location();
          if ((p - p_wp).norm() > r_merge)
          {
            can_merge = false;
            remove.push_back(i);
          }
        }
      }

      if (can_merge)
      {
        location.lane(std::nullopt);
        location.waypoint(waypoint);
      }
    }
  }

  for (auto r_it = remove.rbegin(); r_it != remove.rend(); ++r_it)
  {
    locations.erase(locations.begin() + *r_it);
  }

  return locations;
}

//==============================================================================
rmf_traffic::agv::Plan::StartSet NavParams::_lift_boundary_filter(
  rmf_traffic::agv::Plan::StartSet locations,
  const RobotContext& context) const
{
  std::string correct_map;
  if (const auto level = context.current_boarded_lift_level())
  {
    correct_map = *level;
  }
  else if (const auto location = context.reported_location())
  {
    correct_map = location->map;
  }

  const auto& graph = context.navigation_graph();
  const auto valid_waypoint = [&graph, &correct_map](std::size_t wp_index)
    {
      if (correct_map.empty())
        return true;

      const auto& wp_map = graph.get_waypoint(wp_index).get_map_name();
      return wp_map == correct_map;
    };

  const auto valid_lane = [&graph, &correct_map, valid_waypoint](
    std::size_t lane_index)
    {
      const auto& wp = graph.get_lane(lane_index).entry().waypoint_index();
      return valid_waypoint(wp);
    };

  const auto r_it = std::remove_if(
    locations.begin(),
    locations.end(),
    [&graph, valid_waypoint,
    valid_lane](const rmf_traffic::agv::Plan::Start& location)
    {
      if (location.lane().has_value())
      {
        // If the intention is to move along a lane, then it is okay to keep
        // this, as long as the first waypoint is on the correct map.

        // TODO(@mxgrey): Should we check and make sure that the right events
        // are actually present?
        return !valid_lane(*location.lane());
      }

      if (!location.location().has_value())
      {
        // If the robot is perfectly on some waypoint then we should only check
        // if it's on the correct map.
        return !valid_waypoint(location.waypoint());
      }

      const Eigen::Vector2d p = location.location().value();

      const auto robot_inside_lift = [&]()
      -> rmf_traffic::agv::Graph::LiftPropertiesPtr
      {
        for (const auto& lift : graph.all_known_lifts())
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
rmf_traffic::agv::Plan::StartSet NavParams::_strict_lane_filter(
  rmf_traffic::agv::Plan::StartSet locations) const
{
  const auto r_it = std::remove_if(
    locations.begin(),
    locations.end(),
    [this](const rmf_traffic::agv::Plan::Start& location)
    {
      const auto lane = location.lane();
      if (lane.has_value())
      {
        return this->strict_lanes.count(*lane) > 0;
      }

      return false;
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

  if (_location.empty())
  {
    if (debug_positions)
    {
      std::cout << __FILE__ << "|" << __LINE__ << ": setting robot to LOST" <<
        std::endl;
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
rmf_traffic::ParticipantId RobotContext::participant_id() const
{
  return _itinerary.id();
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
bool RobotContext::robot_finishing_request() const
{
  return _robot_finishing_request;
}

//==============================================================================
void RobotContext::robot_finishing_request(bool robot_specific)
{
  _robot_finishing_request = robot_specific;
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
std::shared_ptr<void> RobotContext::be_charging()
{
  return _lock_charging;
}

//==============================================================================
bool RobotContext::is_charging() const
{
  return _lock_charging.use_count() > 1;
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
uint64_t RobotContext::last_reservation_request_id()
{
  return _last_reservation_request_id++;
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
void RobotContext::set_commission(RobotUpdateHandle::Commission value)
{
  {
    std::lock_guard<std::mutex> lock(*_commission_mutex);
    _commission = std::move(value);
  }

  if (const auto mgr = _task_manager.lock())
  {
    if (!_commission.is_performing_idle_behavior())
    {
      mgr->_cancel_idle_behavior({"decommissioned"});
    }
    else
    {
      // We trigger this in case the robot needs to begin its idle behavior.
      // If it shouldn't perform idle behavior for any reason (e.g. already
      // performing a task), then this will have no effect.
      mgr->_begin_next_task();
    }
  }
}

//==============================================================================
const RobotUpdateHandle::Commission& RobotContext::commission() const
{
  return _commission;
}

//==============================================================================
RobotUpdateHandle::Commission RobotContext::copy_commission() const
{
  std::lock_guard<std::mutex> lock(*_commission_mutex);
  RobotUpdateHandle::Commission copy = _commission;
  return copy;
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
bool RobotContext::has_lift_arrived(
  const std::string& lift_name,
  const std::string& destination_floor) const
{
  if (!_lift_destination)
    return false;

  if (!_lift_destination->matches(lift_name, destination_floor))
    return false;

  return _lift_arrived;
}

//==============================================================================
std::shared_ptr<void> RobotContext::set_lift_destination(
  std::string lift_name,
  std::string destination_floor,
  bool requested_from_inside)
{
  RCLCPP_INFO(
    _node->get_logger(),
    "Setting lift destination for [%s] to %s at level %s",
    requester_id().c_str(),
    lift_name.c_str(),
    destination_floor.c_str());

  _lift_arrived = has_lift_arrived(lift_name, destination_floor);

  _lift_destination = std::make_shared<LiftDestination>(
    LiftDestination{
      std::move(lift_name),
      std::move(destination_floor),
      requested_from_inside
    });

  _initial_time_idle_outside_lift = std::nullopt;

  _publish_lift_destination();
  return _lift_destination;
}

//==============================================================================
void RobotContext::release_lift()
{
  if (_lift_destination)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Releasing lift [%s] for [%s]",
      _lift_destination->lift_name.c_str(),
      requester_id().c_str());
  }
  _lift_destination = nullptr;
  _initial_time_idle_outside_lift = std::nullopt;
  _lift_stubbornness = nullptr;
  _lift_arrived = false;
  clear_final_lift_destination();
}

//==============================================================================
std::optional<RobotUpdateHandle::LiftDestination>
RobotContext::final_lift_destination() const
{
  std::unique_lock<std::mutex> lock(*_final_lift_destination_mutex);
  return _final_lift_destination;
}

//==============================================================================
void RobotContext::set_final_lift_destination(
  RobotUpdateHandle::LiftDestination destination)
{
  std::unique_lock<std::mutex> lock(*_final_lift_destination_mutex);
  _final_lift_destination = std::move(destination);
}

//==============================================================================
void RobotContext::clear_final_lift_destination()
{
  std::unique_lock<std::mutex> lock(*_final_lift_destination_mutex);
  _final_lift_destination = std::nullopt;
}

//==============================================================================
const std::optional<std::string>& RobotContext::holding_door() const
{
  return _holding_door;
}

//==============================================================================
const std::unordered_map<std::string, TimeMsg>&
RobotContext::locked_mutex_groups() const
{
  return _locked_mutex_groups;
}

//==============================================================================
const std::unordered_map<std::string, TimeMsg>&
RobotContext::requesting_mutex_groups() const
{
  return _requesting_mutex_groups;
}

//==============================================================================
const rxcpp::observable<std::string>& RobotContext::request_mutex_groups(
  std::unordered_set<std::string> groups,
  rmf_traffic::Time claim_time)
{
  const auto t = rmf_traffic_ros2::convert(claim_time);
  for (const auto& group : groups)
  {
    const auto [it, inserted] = _requesting_mutex_groups.insert({group, t});
    if (!inserted)
    {
      if (t.nanosec < it->second.nanosec)
        it->second = t;
    }
  }

  _publish_mutex_group_requests();
  return _mutex_group_lock_obs;
}

//==============================================================================
void RobotContext::retain_mutex_groups(
  const std::unordered_set<std::string>& retain)
{
  _retain_mutex_groups(retain, _requesting_mutex_groups);
  _retain_mutex_groups(retain, _locked_mutex_groups);
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
  _lock_charging(std::make_shared<int>(0)),
  _current_task_end_state(state),
  _current_task_id(std::nullopt),
  _task_planner(std::move(task_planner)),
  _reporting(_worker),
  _last_reservation_request_id(0),
  _use_parking_spot_reservations(false)
{
  _most_recent_valid_location = _location;
  _profile = std::make_shared<rmf_traffic::Profile>(
    _itinerary.description().profile());

  _replan_obs = _replan_publisher.get_observable();
  _graph_change_obs = _graph_change_publisher.get_observable();
  _charging_change_obs = _charging_change_publisher.get_observable();
  _battery_soc_obs = _battery_soc_publisher.get_observable();
  _mutex_group_lock_obs = _mutex_group_lock_subject.get_observable();

  _current_mode = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
  _override_status = std::nullopt;

  _action_executor = nullptr;
}

//==============================================================================
void RobotContext::schedule_itinerary(
  std::shared_ptr<rmf_traffic::PlanId> plan_id,
  rmf_traffic::schedule::Itinerary new_itinerary)
{
  bool scheduled = false;
  std::size_t attempts = 0;
  while (!scheduled)
  {
    if (++attempts > 5)
    {
      std::stringstream ss_sizes;
      for (const auto& r : new_itinerary)
      {
        ss_sizes << "[" << r.map() << ":" << r.trajectory().size() << "]";
      }

      RCLCPP_ERROR(
        node()->get_logger(),
        "Repeatedly failled attempts to update schedule with an itinerary "
        "containing [%lu] routes with sizes %s during LockMutexGroup "
        "action for robot [%s]. Last attempted value was [%lu]. We will "
        "continue without updating the traffic schedule. This could lead to "
        "traffic management problems. Please report this bug to the "
        "maintainers of RMF.",
        new_itinerary.size(),
        ss_sizes.str().c_str(),
        requester_id().c_str(),
        *plan_id);
      break;
    }

    scheduled = itinerary().set(*plan_id, new_itinerary);

    if (!scheduled)
    {
      for (const auto& r : new_itinerary)
      {
        if (r.trajectory().size() < 2)
        {
          std::stringstream ss_sizes;
          for (const auto& r : new_itinerary)
          {
            ss_sizes << "[" << r.map() << ":" << r.trajectory().size() << "]";
          }

          RCLCPP_ERROR(
            node()->get_logger(),
            "Attempting to schedule an itinerary for robot [%s] containing a "
            "route that has fewer than 2 waypoints. Routes sizes are %s. "
            "We will continue without updating the traffic schedule. This "
            "could lead to traffic management problems. Please report this "
            "bug to the maintainers of RMF.",
            requester_id().c_str(),
            ss_sizes.str().c_str());
          return;
        }
      }

      *plan_id = itinerary().assign_plan_id();
      if (attempts > 1)
      {
        RCLCPP_ERROR(
          node()->get_logger(),
          "Invalid plan_id [%lu] when current plan_id is [%lu] for robot [%s] "
          "while performing a LockMutexGroup. Please report this bug to an RMF "
          "developer.",
          *plan_id,
          itinerary().current_plan_id(),
          requester_id().c_str());
      }
    }
  }
}

//==============================================================================
void RobotContext::schedule_hold(
  std::shared_ptr<rmf_traffic::PlanId> plan_id,
  rmf_traffic::Time time,
  rmf_traffic::Duration wait,
  Eigen::Vector3d position,
  const std::string& map)
{
  rmf_traffic::Trajectory hold;
  const auto zero = Eigen::Vector3d::Zero();
  hold.insert(time, position, zero);
  hold.insert(time + wait, position, zero);
  schedule_itinerary(plan_id, {rmf_traffic::Route(map, std::move(hold))});
}

//==============================================================================
std::shared_ptr<const std::string>
RobotContext::current_boarded_lift_level() const
{
  return _current_boarded_lift_level.lock();
}

//==============================================================================
void RobotContext::_set_current_boarded_lift_level(
  std::shared_ptr<const std::string> lift_level)
{
  _current_boarded_lift_level = lift_level;
}

//==============================================================================
std::shared_ptr<const Location> RobotContext::reported_location() const
{
  return _reported_location;
}

//==============================================================================
void RobotContext::_set_reported_location(
  std::shared_ptr<const Location> value)
{
  _reported_location = std::move(value);
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
void RobotContext::_hold_door(std::string door_name)
{
  _holding_door = std::move(door_name);
}

//==============================================================================
void RobotContext::_release_door(const std::string& door_name)
{
  if (_holding_door.has_value() && *_holding_door == door_name)
    _holding_door = std::nullopt;
}

//==============================================================================
void RobotContext::_set_lift_arrived(
  const std::string& lift_name,
  const std::string& destination_floor)
{
  if (!_lift_destination)
    return;

  if (!_lift_destination->matches(lift_name, destination_floor))
    return;

  _lift_arrived = true;
}

//==============================================================================
void RobotContext::_check_lift_state(
  const rmf_lift_msgs::msg::LiftState& state)
{
  if (_lift_destination.use_count() < 2)
  {
    if (_lift_destination && !_lift_destination->requested_from_inside)
    {
      const auto now = std::chrono::steady_clock::now();
      if (_initial_time_idle_outside_lift.has_value())
      {
        const auto lapse = now - *_initial_time_idle_outside_lift;
        if (lapse > std::chrono::seconds(30))
        {
          // The lift destination reference count dropping to one while the lift
          // destination request is on the outside means the task that prompted
          // the lift usage was cancelled while the robot was outside of the lift.
          // Therefore we should release the usage of the lift.
          RCLCPP_INFO(
            _node->get_logger(),
            "Requesting lift [%s] to be released for [%s] because it is outside "
            "the lift and not holding a claim for an extended period of time.",
            _lift_destination->lift_name.c_str(),
            requester_id().c_str());
          release_lift();
        }
      }
      else
      {
        _initial_time_idle_outside_lift = now;
      }
    }
    else if (_lift_destination && !_current_task_id.has_value())
    {
      const Eigen::Vector2d p = position().block<2, 1>(0, 0);
      const auto& graph = navigation_graph();
      const auto found_lift =
        graph.find_known_lift(_lift_destination->lift_name);

      bool inside_lift = false;
      if (found_lift)
      {
        inside_lift = found_lift->is_in_lift(p);
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
          if (lapse > std::chrono::seconds(10))
          {
            RCLCPP_INFO(
              _node->get_logger(),
              "Releasing lift [%s] for robot [%s] because it has remained idle "
              "outside of the lift.",
              _lift_destination->lift_name.c_str(),
              requester_id().c_str());
            release_lift();
          }
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
      msg.request_time = _node->now();
      _node->lift_request()->publish(msg);
    }

    if (_lift_destination && _lift_destination->lift_name == state.lift_name)
    {
      if (!_lift_stubbornness)
      {
        // Be a stubborn negotiator while using the lift
        _lift_stubbornness = be_stubborn();
      }
    }
  }
  else if (_lift_destination && _lift_destination->lift_name == state.lift_name)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "[%s] is waiting to begin a session with lift [%s] but the lift is "
      "currently held by [%s]",
      _requester_id.c_str(),
      _lift_destination->lift_name.c_str(),
      state.session_id.c_str());
  }

  if (_lift_destination && _lift_destination->lift_name == state.lift_name)
  {
    _publish_lift_destination();
  }
}

//==============================================================================
void RobotContext::_publish_lift_destination()
{
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
void RobotContext::_check_door_supervisor(
  const rmf_door_msgs::msg::SupervisorHeartbeat& state)
{
  const auto now = std::chrono::steady_clock::now();
  const auto dt = std::chrono::seconds(10);
  if (_current_task_id.has_value())
  {
    _last_active_task_time = now;
  }
  else
  {
    if (_last_active_task_time + dt < now)
    {
      // Do not hold a door if a robot is idle for more than 10 seconds
      _holding_door = std::nullopt;
    }
  }

  for (const auto& door : state.all_sessions)
  {
    for (const auto& session : door.sessions)
    {
      if (session.requester_id == _requester_id)
      {
        if (!_holding_door.has_value() || *_holding_door != door.door_name)
        {
          // We should not be holding this door open
          _node->door_request()->publish(
            rmf_door_msgs::build<rmf_door_msgs::msg::DoorRequest>()
            .request_time(_node->now())
            .requester_id(_requester_id)
            .door_name(door.door_name)
            .requested_mode(
              rmf_door_msgs::build<rmf_door_msgs::msg::DoorMode>()
              .value(rmf_door_msgs::msg::DoorMode::MODE_CLOSED)));
        }
      }
    }
  }
}

//==============================================================================
void RobotContext::_check_mutex_groups(
  const rmf_fleet_msgs::msg::MutexGroupStates& states)
{
  // Make sure to release any mutex groups that this robot is not trying to
  // lock right now.
  for (const auto& assignment : states.assignments)
  {
    if (assignment.claimant != participant_id())
    {
      if (_requesting_mutex_groups.count(assignment.group) > 0)
      {
        if (assignment.claimant == (uint64_t)(-1))
        {
          RCLCPP_INFO(
            _node->get_logger(),
            "[%s] is waiting for the mutex superviser to receive its request "
            "for mutex group [%s]",
            _requester_id.c_str(),
            assignment.group.c_str());
        }
        else
        {
          std::string holder;
          if (const auto p = _schedule->get_participant(assignment.claimant))
          {
            holder = p->owner() + "/" + p->name();
          }
          else
          {
            holder = "unknown participant #"
              + std::to_string(assignment.claimant);
          }

          RCLCPP_INFO(
            _node->get_logger(),
            "[%s] is waiting to lock mutex group [%s] but that mutex is "
            "currently held by [%s]",
            _requester_id.c_str(),
            assignment.group.c_str(),
            holder.c_str());
        }
      }

      continue;
    }

    if (_requesting_mutex_groups.count(assignment.group) > 0)
    {
      _requesting_mutex_groups.erase(assignment.group);
      _locked_mutex_groups[assignment.group] = assignment.claim_time;
      _mutex_group_lock_subject.get_subscriber().on_next(assignment.group);
    }
    else if (_locked_mutex_groups.count(assignment.group) == 0)
    {
      // This assignment does not match either the requested nor any currently
      // locked mutex groups. This is an error so let's release it.
      _node->mutex_group_request()->publish(
        rmf_fleet_msgs::build<rmf_fleet_msgs::msg::MutexGroupRequest>()
        .group(assignment.group)
        .claimant(participant_id())
        .claim_time(assignment.claim_time)
        .mode(rmf_fleet_msgs::msg::MutexGroupRequest::MODE_RELEASE));
    }
  }
}

//==============================================================================
void RobotContext::_retain_mutex_groups(
  const std::unordered_set<std::string>& retain,
  std::unordered_map<std::string, TimeMsg>& groups)
{
  std::vector<MutexGroupData> release;
  for (const auto& [name, time] : groups)
  {
    if (retain.count(name) == 0)
    {
      release.push_back(MutexGroupData{name, time});
    }
  }

  for (const auto& data : release)
  {
    _release_mutex_group(data);
    _locked_mutex_groups.erase(data.name);
  }
}

//==============================================================================
void RobotContext::_release_mutex_group(const MutexGroupData& data) const
{
  if (data.name.empty())
  {
    return;
  }

  _node->mutex_group_request()->publish(
    rmf_fleet_msgs::build<rmf_fleet_msgs::msg::MutexGroupRequest>()
    .group(data.name)
    .claimant(participant_id())
    .claim_time(data.claim_time)
    .mode(rmf_fleet_msgs::msg::MutexGroupRequest::MODE_RELEASE));
}

//==============================================================================
void RobotContext::_publish_mutex_group_requests()
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
      if (!_requesting_mutex_groups.empty()
        || !_locked_mutex_groups.empty())
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

        for (const auto& [name, time] : _requesting_mutex_groups)
        {
          warning(name);
          _release_mutex_group(MutexGroupData{name, time});
        }
        _requesting_mutex_groups.clear();

        for (const auto& [name, time] : _locked_mutex_groups)
        {
          warning(name);
          _release_mutex_group(MutexGroupData{name, time});
        }
        _locked_mutex_groups.clear();
      }
    }
  }

  auto publish = [&](const MutexGroupData& data)
    {
      _node->mutex_group_request()->publish(
        rmf_fleet_msgs::build<rmf_fleet_msgs::msg::MutexGroupRequest>()
        .group(data.name)
        .claimant(participant_id())
        .claim_time(data.claim_time)
        .mode(rmf_fleet_msgs::msg::MutexGroupRequest::MODE_LOCK));
    };

  for (const auto& [name, time] : _requesting_mutex_groups)
  {
    publish(MutexGroupData{name, time});
  }

  for (const auto& [name, time] : _locked_mutex_groups)
  {
    publish(MutexGroupData{name, time});
  }
}

//==============================================================================
void RobotContext::_set_allocated_destination(
  const rmf_reservation_msgs::msg::ReservationAllocation& ticket)
{
  _reservation_mgr.replace_ticket(ticket);
}

//==============================================================================
void RobotContext::_set_parking_spot_manager(const bool enabled)
{
  _use_parking_spot_reservations = enabled;
}

//==============================================================================
bool RobotContext::_parking_spot_manager_enabled()
{
  return _use_parking_spot_reservations;
}

//==============================================================================
std::string RobotContext::_get_reserved_location()
{
  return _reservation_mgr.get_reserved_location();
}

//==============================================================================
bool RobotContext::_has_ticket() const
{
  return _reservation_mgr.has_ticket();
}

//==============================================================================
uint32_t RobotContext::_begin_dynamic_event(
  std::string description,
  std::shared_ptr<DynamicEventCallbacks> callbacks)
{
  ++_dynamic_event_seq;
  _dynamic_event_goal.reset();
  _dynamic_event_callbacks = std::move(callbacks);

  auto msg = rmf_task_msgs::build<DynamicEventDescription>()
    .fleet(_itinerary.description().owner())
    .robot(_itinerary.description().name())
    .dynamic_event_seq(_dynamic_event_seq)
    .description(std::move(description))
    .start_time(_node->now());

  // We publish to both the general topic and the robot-specific topic
  _node->all_dynamic_event_descriptions()->publish(msg);
  _individual_dynamic_event_description_pub->publish(msg);

  return _dynamic_event_seq;
}

//==============================================================================
void RobotContext::_publish_dynamic_event_status(
  rmf_task::Event::State::Status status,
  uint64_t id)
{
  uint8_t dynamic_state = DynamicEventStatus::DYNAMIC_STATE_INACTIVE;
  if (!_dynamic_event_callbacks.expired())
  {
    if (_dynamic_event_goal.expired())
    {
      dynamic_state = DynamicEventStatus::DYNAMIC_STATE_WAITING;
    }
    else
    {
      dynamic_state = DynamicEventStatus::DYNAMIC_STATE_RUNNING;
    }
  }

  auto msg = rmf_task_msgs::build<DynamicEventStatus>()
    .dynamic_event_seq(_dynamic_event_seq)
    .dynamic_state(dynamic_state)
    .status(status_to_string(status))
    .id(id)
    .time(_node->now());

  _dynamic_event_status_pub->publish(msg);
}

//==============================================================================
std::vector<rmf_traffic::agv::Plan::Goal>
RobotContext::_find_and_sort_parking_spots(
  const bool same_floor) const
{
  std::vector<rmf_traffic::agv::Plan::Goal> final_result;
  // Retrieve nav graph
  const auto& graph = navigation_graph();

  // Get current location
  auto current_location = location();
  if (current_location.size() == 0)
  {
    // Could not localize should probably log an error
    RCLCPP_ERROR(node()->get_logger(), "Unable to localize.");
    return final_result;
  }

  // Order wait points by the distance from the destination.
  std::vector<std::tuple<double, rmf_traffic::agv::Plan::Goal>>
  waitpoints_order;
  for (std::size_t wp_idx = 0; wp_idx < graph.num_waypoints(); ++wp_idx)
  {
    const auto& wp = graph.get_waypoint(wp_idx);

    if (!wp.is_parking_spot())
    {
      continue;
    }

    if (same_floor)
    {

      // Check if same map. If not don't consider location. This is to ensure
      // the robot does not try to board a lift.
      if (wp.get_map_name() != map())
      {
        RCLCPP_INFO(
          node()->get_logger(),
          "Skipping [%lu] as it is on map [%s] but robot is on map [%s].",
          wp_idx,
          wp.get_map_name().c_str(),
          map().c_str());
        continue;
      }
    }
    auto result = planner()->quickest_path(current_location, wp_idx);
    if (!result.has_value())
    {
      RCLCPP_INFO(
        node()->get_logger(),
        "No path found for waypoint #%lu",
        wp_idx);
      continue;
    }

    rmf_traffic::agv::Plan::Goal goal(wp_idx);
    waitpoints_order.emplace_back(result->cost(), goal);

  }

  //Sort waiting points
  std::sort(waitpoints_order.begin(), waitpoints_order.end(),
    [](const std::tuple<double, rmf_traffic::agv::Plan::Goal>& a,
    const std::tuple<double, rmf_traffic::agv::Plan::Goal>& b)
    {
      return std::get<0>(a) < std::get<0>(b);
    });


  for (auto&[_, waitpoint]: waitpoints_order)
  {
    final_result.push_back(waitpoint);
  }
  return final_result;
}

//==============================================================================
std::vector<rmf_traffic::agv::Plan::Goal>
RobotContext::_find_and_sort_parking_spots(
  const rmf_traffic::agv::Plan::Goal& dest, const bool same_floor) const
{
  std::vector<rmf_traffic::agv::Plan::Goal> final_result;
  // Retrieve nav graph
  const auto& graph = navigation_graph();

  // Get current location
  rmf_traffic::agv::Plan::StartSet start;
  start.emplace_back(now(), dest.waypoint(), 0);

  // Order wait points by the distance from the destination.
  std::vector<std::tuple<double, rmf_traffic::agv::Plan::Goal>>
  waitpoints_order;
  for (std::size_t wp_idx = 0; wp_idx < graph.num_waypoints(); ++wp_idx)
  {
    const auto& wp = graph.get_waypoint(wp_idx);

    if (!wp.is_parking_spot())
    {
      continue;
    }

    if (same_floor)
    {

      // Check if same map. If not don't consider location. This is to ensure
      // the robot does not try to board a lift.
      if (wp.get_map_name() != map())
      {
        RCLCPP_INFO(
          node()->get_logger(),
          "Skipping [%lu] as it is on map [%s] but robot is on map [%s].",
          wp_idx,
          wp.get_map_name().c_str(),
          map().c_str());
        continue;
      }
    }
    auto result = planner()->quickest_path(start, wp_idx);
    if (!result.has_value())
    {
      RCLCPP_INFO(
        node()->get_logger(),
        "No path found for waypoint #%lu",
        wp_idx);
      continue;
    }

    rmf_traffic::agv::Plan::Goal goal(wp_idx);
    waitpoints_order.emplace_back(result->cost(), goal);

  }

  //Sort waiting points
  std::sort(waitpoints_order.begin(), waitpoints_order.end(),
    [](const std::tuple<double, rmf_traffic::agv::Plan::Goal>& a,
    const std::tuple<double, rmf_traffic::agv::Plan::Goal>& b)
    {
      return std::get<0>(a) < std::get<0>(b);
    });


  for (auto&[_, waitpoint]: waitpoints_order)
  {
    final_result.push_back(waitpoint);
  }
  return final_result;
}

//==============================================================================
void RobotContext::_handle_mutex_group_manual_release(
  const rmf_fleet_msgs::msg::MutexGroupManualRelease& msg)
{
  if (msg.fleet != group())
    return;

  if (msg.robot != name())
    return;

  std::unordered_set<std::string> retain;
  for (const auto& g : _locked_mutex_groups)
  {
    retain.insert(g.first);
  }

  for (const auto& g : msg.release_mutex_groups)
  {
    retain.erase(g);
  }

  retain_mutex_groups(retain);
}

//==============================================================================
void RobotContext::_initialize_dynamic_event_server()
{
  const auto single_transient_local_qos = rclcpp::QoS(1).reliable().transient_local();
  _individual_dynamic_event_description_pub = _node
    ->create_publisher<DynamicEventDescription>(
      DynamicEventBeginTopicBase + "/" + requester_id(),
      single_transient_local_qos);

  _dynamic_event_status_pub = _node
      ->create_publisher<DynamicEventStatus>(
        DynamicEventStatusTopicBase + "/" + requester_id(),
        single_transient_local_qos);

  _dynamic_event_seq = 0;

  std::string logger_name = std::string("rmf.dynamic_event.") + group() + "." + name();
  auto logger = rclcpp::get_logger(logger_name);
  auto handle_goal = [w = weak_from_this(), logger](
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const DynamicEventAction::Goal> goal)
  {
    const auto me = w.lock();
    if (!me || goal->dynamic_event_seq != me->_dynamic_event_seq)
    {
      if (me)
      {
        RCLCPP_ERROR(
          logger,
          "Rejecting dynamic event goal because of dynamic_event_seq mismatch. "
          "Expected %d, received %d.",
          me->_dynamic_event_seq,
          goal->dynamic_event_seq);
      }
      else
      {
        RCLCPP_ERROR(
          logger,
          "Rejecting dynamic event goal because the fleet adapter is winding down.");
      }
      return rclcpp_action::GoalResponse::REJECT;
    }

    const auto callbacks = me->_dynamic_event_callbacks.lock();
    if (!callbacks)
    {
      RCLCPP_ERROR(
        logger,
        "Rejecting dynamic event goal because there is no active dynamic event.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    const bool cancellation = goal->event_type ==
      DynamicEventAction::Goal::EVENT_TYPE_CANCEL;

    if (!me->_dynamic_event_goal.expired() && !cancellation)
    {
      // There is an ongoing goal and the request is not a cancellation, so we
      // should reject it.
      RCLCPP_ERROR(
        logger,
        "Rejecting dynamic event goal because there is an active child event, "
        "and the goal was not a cancellation.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->event_type == DynamicEventAction::Goal::EVENT_TYPE_NEXT)
    {
      if (!callbacks->validator(goal->category, goal->description))
      {
        RCLCPP_ERROR(
          logger,
          "Rejecting dynamic event goal because it failed validation.");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_accepted = [w = weak_from_this()](
    const std::shared_ptr<DynamicEventHandle> handle)
  {
    const auto me = w.lock();
    if (!me)
    {
      handle->execute();
      handle->abort(dynamic_event_execution_failure("shutting down"));
      return;
    }

    const auto callbacks = me->_dynamic_event_callbacks.lock();
    const auto event_conflict = !me->_dynamic_event_goal.expired()
      && handle->get_goal()->event_type != DynamicEventAction::Goal::EVENT_TYPE_CANCEL;

    if (!callbacks || event_conflict)
    {
      handle->execute();
      handle->abort(dynamic_event_execution_failure("race condition"));
      return;
    }

    auto execution_raii = std::make_shared<rclcpp_action::GoalUUID>(handle->get_goal_id());
    me->_dynamic_event_goal = execution_raii;
    callbacks->executor(handle, execution_raii);
  };

  auto handle_cancel = [w = weak_from_this()](
    const std::shared_ptr<DynamicEventHandle> handle)
  {
    const auto me = w.lock();
    std::shared_ptr<rclcpp_action::GoalUUID> current_action_guid;
    std::shared_ptr<DynamicEventCallbacks> callbacks;
    if (me)
    {
      current_action_guid = me->_dynamic_event_goal.lock();
      callbacks = me->_dynamic_event_callbacks.lock();
    }

    if (!callbacks || !current_action_guid || *current_action_guid != handle->get_goal_id())
    {
      handle->canceled(
        dynamic_event_execution_failure("cancelled while not running"));
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    callbacks->cancel(handle);

    return rclcpp_action::CancelResponse::ACCEPT;
  };

  _dynamic_event_server = rclcpp_action::create_server<DynamicEventAction>(
    _node,
    DynamicEventActionName + "/" + requester_id(),
    handle_goal,
    handle_cancel,
    handle_accepted);
}

} // namespace agv
} // namespace rmf_fleet_adapter
