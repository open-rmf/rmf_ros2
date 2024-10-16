#include "internal_utilities.hpp"

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
std::string wp_name(
  const agv::RobotContext& context,
  const rmf_traffic::agv::Plan::Goal& goal)
{
  const auto& g = context.planner()->get_configuration().graph();
  const auto& wp = g.get_waypoint(goal.waypoint());
  if (wp.name())
    return *wp.name();

  return "#" + std::to_string(goal.waypoint());
}

//==============================================================================
std::string wp_name(const agv::RobotContext& context)
{
  const auto& g = context.planner()->get_configuration().graph();
  const auto& locations = context.location();
  for (const auto& l : locations)
  {
    const auto& wp = g.get_waypoint(l.waypoint());
    if (wp.name())
      return *wp.name();
  }

  if (locations.empty())
    return "<null>";

  return "#" + std::to_string(locations.front().waypoint());
}

}
}
