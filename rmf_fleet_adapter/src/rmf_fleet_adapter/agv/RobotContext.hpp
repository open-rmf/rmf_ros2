#ifndef SRC__RMF_FLEET_ADAPTER__ROBOTCONTEXT_HPP
#define SRC__RMF_FLEET_ADAPTER__ROBOTCONTEXT_HPP

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>

#include <rmf_traffic/schedule/Negotiator.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Snapshot.hpp>

#include <rclcpp/node.hpp>

#include <rmf_rxcpp/Publisher.hpp>
#include <rmf_rxcpp/Transport.hpp>

#include <rxcpp/rx-observable.hpp>

#include "Node.hpp"

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
  rmf_traffic::schedule::Participant& itinerary();

  /// Get a const-reference to the schedule of this robot
  const rmf_traffic::schedule::Participant& itinerary() const;

  using Snappable = rmf_traffic::schedule::Snappable;
  /// Get a const-reference to an interface that lets you get a snapshot of the
  /// schedule.
  const std::shared_ptr<const Snappable>& schedule() const;

  /// Get the schedule description of this robot
  const rmf_traffic::schedule::ParticipantDescription& description() const;

  /// Get the name of this robot
  const std::string& name() const;

  /// Get the requester ID to use for this robot when sending requests
  const std::string& requester_id() const;

  /// Get the navigation graph used by this robot
  const rmf_traffic::agv::Graph& navigation_graph() const;

  /// Get a mutable reference to the planner for this robot
  const std::shared_ptr<const rmf_traffic::agv::Planner>& planner() const;

  class NegotiatorLicense;

  /// Set the schedule negotiator that will take responsibility for this robot.
  /// Hold onto the returned subscription to remain the negotiator for this
  /// robot.
  std::shared_ptr<NegotiatorLicense> set_negotiator(
      rmf_traffic::schedule::Negotiator* negotiator);

  struct Empty { };
  const rxcpp::observable<Empty>& observe_interrupt() const;

  /// Get a reference to the rclcpp node
  const std::shared_ptr<Node>& node();

  /// const-qualified node()
  std::shared_ptr<const Node> node() const;

  /// Get a reference to the worker for this robot. Use this worker to observe
  /// callbacks that can modify the state of the robot.
  const rxcpp::schedulers::worker& worker() const;

  // Documentation inherited from rmf_traffic::schedule::Negotiator
  void respond(
      const TableViewerPtr& table_viewer,
      const ResponderPtr& responder) final;

private:
  friend class FleetUpdateHandle;
  friend class RobotUpdateHandle;

  RobotContext(
    std::shared_ptr<RobotCommandHandle> command_handle,
    std::vector<rmf_traffic::agv::Plan::Start> _initial_location,
    rmf_traffic::schedule::Participant itinerary,
    std::shared_ptr<const Snappable> schedule,
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    std::shared_ptr<Node> node,
    const rxcpp::schedulers::worker& worker);

  std::weak_ptr<RobotCommandHandle> _command_handle;
  std::vector<rmf_traffic::agv::Plan::Start> _location;
  rmf_traffic::schedule::Participant _itinerary;
  std::shared_ptr<const Snappable> _schedule;
  std::shared_ptr<const rmf_traffic::agv::Planner> _planner;

  std::shared_ptr<void> _negotiation_license;

  rmf_rxcpp::Publisher<Empty> _interrupt_publisher;

  std::shared_ptr<Node> _node;
  rxcpp::schedulers::worker _worker;
  std::string _requester_id;

  rmf_traffic::schedule::Negotiator* _negotiator = nullptr;
};

using RobotContextPtr = std::shared_ptr<RobotContext>;
using ConstRobotContextPtr = std::shared_ptr<const RobotContext>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__ROBOTCONTEXT_HPP
