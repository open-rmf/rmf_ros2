/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP

#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/closed_lanes.hpp>
#include <rmf_fleet_msgs/msg/dock_summary.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class EasyFullControl::Implementation
{
public:

  Implementation(
    Configuration config)
  : _config{std::move(config)}
  {
    // Do nothing
  }

  bool initialize_fleet(const AdapterPtr& adapter);

// private:
  const Configuration _config;
  AdapterPtr _adapter;
  std::string _fleet_name;
  YAML::Node _fleet_config;
  FleetUpdateHandlePtr _fleet_handle;
  std::shared_ptr<Graph> _graph;
  std::shared_ptr<VehicleTraits> _traits;

  double _max_delay;
  std::string _charger_waypoint;
  std::string _map_name;

  rclcpp::Publisher<rmf_fleet_msgs::msg::ClosedLanes>::SharedPtr _closed_lanes_pub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::LaneRequest>::SharedPtr _lane_closure_request_sub;
  std::unordered_set<std::size_t> _closed_lanes;
  std::unordered_map<std::string, EasyCommandHandlePtr> _robots;
};

//==============================================================================
class EasyFullControl::EasyCommandHandle
  : public RobotCommandHandle,
  public std::enable_shared_from_this<EasyFullControl::EasyCommandHandle>
{
public:

  using Target = EasyFullControl::Target;
  using GetPosition = EasyFullControl::GetPosition;
  using ProcessCompleted = EasyFullControl::ProcessCompleted;
  using Transformer = std::function<Eigen::Vector3d(Eigen::Vector3d)>;

  enum class RobotState : uint8_t
  {
    IDLE = 0,
    WAITING = 1,
    MOVING = 2
  };

  struct PlanWaypoint
  {
    std::size_t index; // Index in follow_new_path
    Eigen::Vector3d position;
    rmf_traffic::Time time;
    std::optional<std::size_t> graph_index;
    std::vector<std::size_t> approach_lanes;

    PlanWaypoint(std::size_t index_, const rmf_traffic::agv::Plan::Waypoint& wp)
    : index(index_),
      position(wp.position()),
      time(wp.time()),
      graph_index(wp.graph_index()),
      approach_lanes(wp.approach_lanes())
    {
      // Do nothing
    }
  };

  /// Constructor
  EasyCommandHandle(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name,
    const std::string& robot_name,
    std::shared_ptr<Graph> graph,
    std::shared_ptr<VehicleTraits> traits,
    Transformer rmf_to_robot_transformer,
    const std::string& map_name,
    std::optional<rmf_traffic::Duration> max_delay,
    const Planner::Start& start,
    const Eigen::Vector3d& initial_position,
    double initial_battery_soc,
    std::size_t charger_waypoint,
    GetPosition get_position,
    std::function<ProcessCompleted(const Target target)> navigate,
    std::function<ProcessCompleted(const std::string& dock_name)> dock,
    ProcessCompleted stop);

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) final;

  void stop() final;

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final;

  void set_updater(RobotUpdateHandlePtr updater);

  void start_update_thread();

  void update_state();

  void update_position(const Eigen::Vector3d& position,  // in RMF coordinates
                       const std::string& map_name);

  void update_battery_soc(double soc);

  void newly_closed_lanes(const std::unordered_set<std::size_t>& closed_lanes);

  bool initialized();

  ~EasyCommandHandle();

private:

  void parse_waypoints(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints);

  void start_follow();

  void start_dock();

  std::optional<std::size_t> get_current_lane();

  double dist(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

  std::shared_ptr<rclcpp::Node> _node;
  std::string _fleet_name;
  std::string _robot_name;
  std::shared_ptr<Graph> _graph;
  std::shared_ptr<VehicleTraits> _traits;
  Transformer _rmf_to_robot_transformer;
  std::string _map_name;
  std::optional<rmf_traffic::Duration> _max_delay = std::nullopt;
  Eigen::Vector3d _position;
  double _battery_soc;
  const std::size_t _charger_waypoint;
  GetPosition _get_position;
  std::function<ProcessCompleted(const Target target)> _navigate; // in robot coordinates
  std::function<ProcessCompleted(const std::string& dock_name)> _dock;
  ProcessCompleted _stop;
  RobotUpdateHandlePtr _updater;
  bool _is_charger_set;
  RobotState _state;
  bool _initialized;

  std::optional<std::size_t> _on_waypoint = std::nullopt;
  std::optional<std::size_t> _last_known_waypoint = std::nullopt;
  std::optional<std::size_t> _on_lane = std::nullopt;

  std::mutex _mutex;

  std::optional<PlanWaypoint> _target_waypoint;
  std::vector<PlanWaypoint> _remaining_waypoints;
  std::size_t _dock_waypoint_index;
  std::size_t _action_waypoint_index;

  std::string _dock_name;
  std::unordered_map<std::string, std::vector<rmf_fleet_msgs::msg::Location>> _docks;

  std::thread _update_thread;
  std::thread _follow_thread;
  std::thread _dock_thread;
  std::atomic_bool _stop_follow_thread;
  std::atomic_bool _stop_dock_thread;
  ProcessCompleted _navigation_cb;
  ProcessCompleted _docking_cb;
  RequestCompleted _path_finished_callback;
  RequestCompleted _docking_finished_callback;
  ArrivalEstimator _next_arrival_estimator;

  rclcpp::Subscription<rmf_fleet_msgs::msg::DockSummary>::SharedPtr _dock_summary_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr _action_execution_sub;

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP