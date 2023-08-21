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

namespace rmf_fleet_adapter {
namespace agv {

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
  assert(!_location.empty());
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

//==============================================================================
const std::string& RobotContext::map() const
{
  assert(!_location.empty());
  return navigation_graph()
    .get_waypoint(_location.front().waypoint()).get_map_name();
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
  _location = std::move(location_);
  filter_closed_lanes();
  if (_location.empty())
  {
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
  if (const auto planner = *_planner)
  {
    const auto& closures = planner->get_configuration().lane_closures();
    for (std::size_t i = 0; i < _location.size(); )
    {
      if (_location[i].lane().has_value())
      {
        if (closures.is_closed(*_location[i].lane()))
        {
          if (_location.size() > 1)
          {
            _location.erase(_location.begin() + i);
            continue;
          }
          else
          {
            RCLCPP_WARN(
              node()->get_logger(),
              "Robot [%s] is being forced to use closed lane [%lu] because it "
              "has not been provided any other feasible lanes to use.",
              requester_id().c_str(),
              *_location[i].lane());
            return;
          }
        }
      }
      ++i;
    }
  }
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
        self->_charger_wp,
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
  _current_task_id = std::move(id);
  return *this;
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
std::size_t RobotContext::dedicated_charger_wp() const
{
  return _charger_wp;
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
RobotContext::RobotContext(
  std::shared_ptr<RobotCommandHandle> command_handle,
  std::vector<rmf_traffic::agv::Plan::Start> _initial_location,
  rmf_traffic::schedule::Participant itinerary,
  std::shared_ptr<const Mirror> schedule,
  std::shared_ptr<std::shared_ptr<const rmf_traffic::agv::Planner>> planner,
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
  _task_activator(std::move(activator)),
  _task_parameters(std::move(parameters)),
  _stubbornness(std::make_shared<int>(0)),
  _node(std::move(node)),
  _worker(worker),
  _maximum_delay(maximum_delay),
  _requester_id(
    _itinerary.description().owner() + "/" + _itinerary.description().name()),
  _charger_wp(state.dedicated_charging_waypoint().value()),
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

} // namespace agv
} // namespace rmf_fleet_adapter
