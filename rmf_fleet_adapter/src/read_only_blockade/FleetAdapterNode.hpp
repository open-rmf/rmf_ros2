#ifndef SRC__READ_ONLY_BLOCKADE__FLEETADAPTERNODE_HPP
#define SRC__READ_ONLY_BLOCKADE__FLEETADAPTERNODE_HPP

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/blockade/Participant.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>

#include <rclcpp/node.hpp>

#include <unordered_map>
#include <vector>

#include <rmf_utils/optional.hpp>

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic_ros2/blockade/Writer.hpp>

#include <rmf_fleet_adapter/agv/Waypoint.hpp>

namespace rmf_fleet_adapter {
namespace read_only_blockade {

//==============================================================================
class FleetAdapterNode : public rclcpp::Node
{
public:

  static std::shared_ptr<FleetAdapterNode> make();

  bool ignore_fleet(const std::string& fleet_name) const;

private:

  FleetAdapterNode();

  std::string _fleet_name;

  rmf_traffic::agv::VehicleTraits _traits;

  rmf_traffic::Duration _delay_threshold;

  rmf_traffic::Duration _hold_duration = std::chrono::seconds(30);

  // Graph snapping parameters
  const double _waypoint_snap_distance = 1.0; // meters
  const double _lane_snap_distance = 3.0; // meters

  std::mutex _async_mutex;

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_subscription;

  struct Connections
  {
    rmf_traffic_ros2::schedule::WriterPtr schedule_writer;
    rmf_traffic_ros2::blockade::WriterPtr blockade_writer;
    rmf_traffic_ros2::schedule::MirrorManager mirror;
    rmf_traffic_ros2::schedule::Negotiation negotiation;
    rmf_traffic::agv::Planner planner;
  };

  std::optional<Connections> _connect;

  struct Robot
  {
    std::shared_ptr<rmf_traffic::schedule::Participant> schedule;
    rmf_traffic::blockade::Participant blockade;
    std::shared_ptr<void> negotiation_license;

    std::optional<std::vector<rmf_traffic::blockade::Writer::Checkpoint>>
    current_path;

    std::optional<std::string> current_goal;
  };

  using Robots = std::unordered_map<std::string, std::unique_ptr<Robot>>;
  Robots _robots;

  void fleet_state_update(FleetState::UniquePtr new_state);

  using RobotState = rmf_fleet_msgs::msg::RobotState;
  void register_robot(const RobotState& state);
  void update_robot(const RobotState& state, const Robots::iterator& it);

  void update_progress(
    const RobotState& state,
    Robot& robot,
    rmf_traffic::Time now);

  void make_plan(
    const RobotState& state,
    Robot& robot,
    rmf_traffic::Time now);

  rmf_traffic::Duration make_delay(
    const rmf_traffic::schedule::Participant& schedule,
    rmf_traffic::Time now);

  std::vector<rmf_traffic::Route> make_hold(
    const RobotState& state,
    rmf_traffic::Time now) const;
};

} // namespace read_only_blockade
} // namespace rmf_fleet_adapter

#endif // SRC__READ_ONLY_BLOCKADE__FLEETADAPTERNODE_HPP
