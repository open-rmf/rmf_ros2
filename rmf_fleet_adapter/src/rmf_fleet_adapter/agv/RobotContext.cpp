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
const std::vector<rmf_traffic::agv::Plan::Start>& RobotContext::location() const
{
  return _location;
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
auto RobotContext::schedule() const -> const std::shared_ptr<const Snappable>&
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
const rxcpp::observable<RobotContext::Empty>&
RobotContext::observe_interrupt() const
{
  return _interrupt_obs;
}

//==============================================================================
void RobotContext::trigger_interrupt()
{
  _interrupt_publisher.get_subscriber().on_next(Empty{});
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
const rmf_task::agv::State& RobotContext::current_task_end_state() const
{
  return _current_task_end_state;
}

//==============================================================================
RobotContext& RobotContext::current_task_end_state(
    const rmf_task::agv::State& state)
{
  _current_task_end_state = state;
  return *this;
}

//==============================================================================
double RobotContext::current_battery_soc() const
{
  return _current_battery_soc;
}

RobotContext& RobotContext::current_battery_soc(const double battery_soc)
{
  _current_battery_soc = battery_soc;
  _battery_soc_publisher.get_subscriber().on_next(battery_soc);
  
  return *this;
}

//==============================================================================
const rxcpp::observable<double>& RobotContext::observe_battery_soc() const
{
  return _battery_soc_obs;
}

//==============================================================================
const std::shared_ptr<const rmf_task::agv::TaskPlanner>& 
RobotContext::task_planner() const
{
  return _task_planner;
}

//==============================================================================
auto RobotContext::task_planner(
  const std::shared_ptr<const rmf_task::agv::TaskPlanner> task_planner)
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
  if (_negotiator)
    return _negotiator->respond(table_viewer, responder);

  // If there is no negotiator assigned for this robot, then use a
  // StubbornNegotiator.
  //
  // TODO(MXG): Consider if this should be scheduled on a separate thread
  // instead of executed immediately. The StubbornNegotiator doesn't do any
  // planning, so it should be able to finish quickly, but that should be
  // verified with benchmarks.
  rmf_traffic::schedule::StubbornNegotiator(_itinerary).respond(
        table_viewer, responder);
}

//==============================================================================
RobotContext::RobotContext(
  std::shared_ptr<RobotCommandHandle> command_handle,
  std::vector<rmf_traffic::agv::Plan::Start> _initial_location,
  rmf_traffic::schedule::Participant itinerary,
  std::shared_ptr<const Snappable> schedule,
  std::shared_ptr<std::shared_ptr<const rmf_traffic::agv::Planner>> planner,
  std::shared_ptr<rmf_fleet_adapter::agv::Node> node,
  const rxcpp::schedulers::worker& worker,
  rmf_utils::optional<rmf_traffic::Duration> maximum_delay,
  rmf_task::agv::State state,
  std::shared_ptr<const rmf_task::agv::TaskPlanner> task_planner)
  : _command_handle(std::move(command_handle)),
    _location(std::move(_initial_location)),
    _itinerary(std::move(itinerary)),
    _schedule(std::move(schedule)),
    _planner(std::move(planner)),
    _node(std::move(node)),
    _worker(worker),
    _maximum_delay(maximum_delay),
    _requester_id(
      _itinerary.description().owner() + "/" + _itinerary.description().name()),
    _current_task_end_state(state),
    _task_planner(std::move(task_planner))
{
  _profile = std::make_shared<rmf_traffic::Profile>(
        _itinerary.description().profile());

  _interrupt_obs = _interrupt_publisher.get_observable();

  _battery_soc_obs = _battery_soc_publisher.get_observable();
}

} // namespace agv
} // namespace rmf_fleet_adapter
