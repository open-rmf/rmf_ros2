#ifndef SRC__RMF_FLEET_ADAPTER__ROBOTCONTEXT_HPP
#define SRC__RMF_FLEET_ADAPTER__ROBOTCONTEXT_HPP

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

#include <rmf_traffic/schedule/Negotiator.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <rclcpp/node.hpp>

#include <rmf_rxcpp/Publisher.hpp>

#include <rxcpp/rx-observable.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class RobotContext
    : public std::enable_shared_from_this<RobotContext>,
      public rmf_traffic::schedule::Negotiator
{
public:

  /// Get a handle to the command interface of the robot. This may return a
  /// nullptr if the robot has disconnected and/or its command API is no longer
  /// available.
  std::shared_ptr<RobotCommandHandle> command();

  /// Create a RobotUpdateHandle that reports to this RobotContext
  std::shared_ptr<RobotUpdateHandle> make_updater();

  /// This is the robot's current (x, y, yaw) position.
  Eigen::Vector3d position() const;

  /// This is the map that the robot is on.
  const std::string& map() const;

  /// Get the current time
  rmf_traffic::Time now() const;

  /// This is the current "location" of the robot, which can be used to initiate
  /// a planning job
  const std::vector<rmf_traffic::agv::Plan::Start>& location() const;

  /// Get a mutable reference to the schedule of this robot
  rmf_traffic::schedule::Participant& schedule();

  /// Get a const-reference to the schedule of this robot
  const rmf_traffic::schedule::Participant& schedule() const;

  /// Get the schedule description of this robot
  const rmf_traffic::schedule::ParticipantDescription& description() const;

  /// Get the navigation graph used by this robot
  const rmf_traffic::agv::Graph& navigation_graph() const;

  /// Get a mutable reference to the planner for this robot
  rmf_traffic::agv::Planner& planner();

  /// Get a const-reference to the planner for this robot
  const rmf_traffic::agv::Planner& planner() const;

  class NegotiatorSubscription;

  /// Set the schedule negotiator that will take responsibility for this robot.
  /// Hold onto the returned subscription to remain the negotiator for this
  /// robot.
  std::shared_ptr<NegotiatorSubscription> set_negotiator(
      rmf_traffic::schedule::Negotiator* negotiator);

  struct Empty { };
  const rxcpp::observable<Empty>& observe_interrupt() const;

private:
  friend class RobotUpdateHandle;
  std::weak_ptr<RobotCommandHandle> _command_handle;
  std::vector<rmf_traffic::agv::Plan::Start> _location;
  rmf_traffic::schedule::Participant _schedule;
  rmf_traffic::schedule::Negotiator* _negotiator;
  rmf_traffic::agv::Planner _planner;

  rmf_rxcpp::Publisher<Empty> _interrupt_publisher;

  // We're assuming that a RobotContextPtr is being held by the Node, so we
  // shouldn't have to worry about the lifetime of the Node
  rclcpp::Node* _node;
};

using RobotContextPtr = std::shared_ptr<RobotContext>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__ROBOTCONTEXT_HPP
