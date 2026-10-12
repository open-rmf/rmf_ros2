#ifndef SRC__RMF_FLEET_ADAPTER__TASKS__ZONE_HPP
#define SRC__RMF_FLEET_ADAPTER__TASKS__ZONE_HPP

#include <rmf_task_sequence/Phase.hpp>

#include "../agv/internal_FleetUpdateHandle.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
void add_zone(
  agv::TaskDeserialization& deserialization);

} // namespace tasks
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__TASKS__ZONE_HPP
