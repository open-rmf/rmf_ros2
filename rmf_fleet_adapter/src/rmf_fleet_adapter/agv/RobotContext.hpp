#ifndef SRC__RMF_FLEET_ADAPTER__ROBOTCONTEXT_HPP
#define SRC__RMF_FLEET_ADAPTER__ROBOTCONTEXT_HPP

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class RobotContext
{
public:

  /// Get a handle to the command interface of the robot. This may return a
  /// nullptr if the robot has disconnected and/or its command API is no longer
  /// available.
  const RobotCommandHandle* command();

  /// This is the robot's current (x, y, yaw) position.
  Eigen::Vector3d position() const;

  /// This is the map that the robot is on.
  const std::string& map() const;

  /// This is the current "location" of the robot, which can be used to initiate
  /// a planning job.
  const std::vector<rmf_traffic::agv::Plan::Start>& location() const;

private:
  // TODO(MXG)
};

using RobotContextPtr = std::shared_ptr<RobotContext>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__ROBOTCONTEXT_HPP
