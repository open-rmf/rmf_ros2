#include "GoToZone.hpp"
#include "GoToPlace.hpp"

#include <rmf_task_sequence/events/GoToPlace.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_zone_msgs/msg/zone_request.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include <utility>
#include <vector>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
void GoToZone::add(rmf_task_sequence::Event::Initializer& initializer)
{
  initializer.add<Description>(
    [](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      std::function<void()> update) -> StandbyPtr
    {
      return Standby::make(
        id, get_state, parameters, description, std::move(update));
    },
    [](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      const nlohmann::json&,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished) -> ActivePtr
    {
      return Standby::make(
        id, get_state, parameters, description, std::move(update))
      ->begin(std::move(checkpoint), std::move(finished));
    });
}

//==============================================================================
auto GoToZone::Standby::make(
  const AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const rmf_task::ConstParametersPtr& parameters,
  const Description& description,
  std::function<void()> update)
-> std::shared_ptr<Standby>
{
  const auto state = get_state();
  const auto context = state.get<agv::GetContext>()->value;
  const auto header = description.generate_header(state, *parameters);

  auto standby = std::make_shared<Standby>(Standby{description});
  standby->_assign_id = id;
  standby->_context = context;
  standby->_time_estimate = header.original_duration_estimate();
  standby->_update = std::move(update);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    header.category(),
    header.detail(),
    rmf_task::Event::Status::Standby,
    {},
    context->clock());

  const auto record_validation_error =
    [&standby, &context](std::string msg)
    {
      standby->_state->update_status(rmf_task::Event::Status::Error);
      standby->_state->update_log().error("GoToZone: " + msg);
      standby->_validation_error = std::move(msg);
    };

  const auto& graph = context->navigation_graph();
  const auto zone_props = graph.find_known_zone(description.zone_name());

  if (!zone_props)
  {
    RCLCPP_ERROR(
      context->node()->get_logger(),
      "GoToZone: zone [%s] not found in nav graph for robot [%s]; rejecting.",
      description.zone_name().c_str(),
      context->requester_id().c_str());
    record_validation_error(
      "zone [" + description.zone_name() + "] not found in nav graph");
    return standby;
  }

  // Reject if the robot is already at a waypoint inside the target zone.
  const auto current_wp = context->current_waypoint();
  if (current_wp.has_value())
  {
    for (const auto& iv : zone_props->internal_vertices())
    {
      const auto* wp = graph.find_waypoint(iv.name());
      if (wp && wp->index() == *current_wp)
      {
        RCLCPP_ERROR(
          context->node()->get_logger(),
          "GoToZone: robot [%s] is already at a waypoint inside zone [%s]; "
          "rejecting -- robot must exit the zone first.",
          context->requester_id().c_str(),
          description.zone_name().c_str());
        record_validation_error(
          "robot is already at a waypoint inside zone ["
          + description.zone_name() + "] -- must exit the zone first");
        return standby;
      }
    }
  }

  return standby;
}

//==============================================================================
GoToZone::Standby::Standby(Description description)
: _description(std::move(description))
{
  // Do nothing
}

//==============================================================================
auto GoToZone::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration GoToZone::Standby::duration_estimate() const
{
  return _time_estimate;
}

//==============================================================================
auto GoToZone::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  if (_active)
    return _active;

  if (_validation_error.has_value())
  {
    auto empty_desc =
      rmf_task_sequence::events::GoToPlace::Description::make_for_one_of({});
    _active = GoToPlace::Active::make(
      _assign_id,
      _context,
      *empty_desc,
      std::nullopt,
      _state,
      _update,
      std::move(finished));
    return _active;
  }

  if (_description.modifiers().has_value())
  {
    const auto& mods = *_description.modifiers();
    agv::RobotContext::ZoneTaskModifiers ztm;
    ztm.group_hint = mods.group_hint;
    ztm.orientation_hint = mods.orientation_hint;
    ztm.preferred_waypoints = mods.preferred_waypoints;
    _context->set_zone_task_modifiers(std::move(ztm));
  }
  else
  {
    _context->set_zone_task_modifiers({});
  }
  _context->set_is_zone_task(true);

  const auto& graph = _context->navigation_graph();
  const auto zone_props = graph.find_known_zone(_description.zone_name());
  std::vector<rmf_traffic::agv::Plan::Goal> goals;
  if (zone_props)
  {
    for (const auto& iv : zone_props->internal_vertices())
    {
      const auto* wp = graph.find_waypoint(iv.name());
      if (!wp)
        continue;
      goals.emplace_back(wp->index());
    }
  }

  if (goals.empty())
  {
    _state->update_status(rmf_task::Event::Status::Error);
    _state->update_log().error(
      "GoToZone: zone [" + _description.zone_name()
      + "] has no resolvable waypoints; cannot proceed.");

    RCLCPP_ERROR(
      _context->node()->get_logger(),
      "GoToZone: zone [%s] has no resolvable waypoints for robot [%s]; "
      "the task cannot proceed.",
      _description.zone_name().c_str(),
      _context->requester_id().c_str());

    auto empty_desc =
      rmf_task_sequence::events::GoToPlace::Description::make_for_one_of({});
    _active = GoToPlace::Active::make(
      _assign_id,
      _context,
      *empty_desc,
      std::nullopt,
      _state,
      _update,
      std::move(finished));
    return _active;
  }

  const auto place_desc =
    rmf_task_sequence::events::GoToPlace::Description::make_for_one_of(
      std::move(goals));

  // On task completion/cancel/kill, reset task-level zone state:
  // - is_zone_task and zone_task_modifiers: no longer applicable.
  // - booked_zone_goal: reset for new tasks
  // booked_zone_waypoint is intentionally NOT cleared here: it gates
  // stubbornness and is cleared by ZoneExit / ZoneBookingRevoked so the
  // robot stays stubborn until it properly leaves the zone.
  auto wrapped_finished =
    [context = _context, finished = std::move(finished)]()
    {
      context->set_is_zone_task(false);
      context->set_zone_task_modifiers({});
      context->clear_booked_zone_goal();
      finished();
    };

  // Drive the actual task via GoToPlace::Active
  _active = GoToPlace::Active::make(
    _assign_id,
    _context,
    *place_desc,
    std::nullopt,
    _state,
    _update,
    std::move(wrapped_finished));

  return _active;
}

} // namespace events
} // namespace rmf_fleet_adapter
