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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYTRAFFICLIGHT_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYTRAFFICLIGHT_HPP

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

#include <rmf_fleet_adapter/agv/EasyTrafficLight.hpp>

#include "../services/FindPath.hpp"
#include "../services/Negotiate.hpp"

#include <rmf_traffic_ros2/blockade/Writer.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_traffic/schedule/Mirror.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <rmf_rxcpp/RxJobs.hpp>

#include "Node.hpp"

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class EasyTrafficLight::Implementation
{
public:

  using MirrorPtr = std::shared_ptr<const rmf_traffic::schedule::Mirror>;

  struct NegotiateManagers
  {
    rmf_rxcpp::subscription_guard subscription;
    rclcpp::TimerBase::SharedPtr timer;
  };
  using NegotiatePtr = std::shared_ptr<services::Negotiate>;
  using NegotiateServiceMap =
    std::unordered_map<NegotiatePtr, NegotiateManagers>;

  using Dependency =
    rmf_traffic::schedule::ItineraryViewer::DependencySubscription;
  using DependencyPtr = std::shared_ptr<const Dependency>;

  class DependencyTracker
  {
  public:

    void add(
      const rmf_traffic::Dependency& dep,
      const std::shared_ptr<const rmf_traffic::schedule::Mirror>& mirror);

    void set_time(rmf_traffic::Time expected_time);

    bool ready() const;

    bool deprecated(const rmf_traffic::Time current_time) const;

  private:
    std::vector<DependencyPtr> _subscriptions;
    std::optional<rmf_traffic::Time> _expected_time;
    mutable rmf_traffic::Time _last_time =
      rmf_traffic::Time(rmf_traffic::Duration(0));
  };

  struct Plan
  {
    rmf_traffic::PlanId id;
    rmf_traffic::agv::Plan plan;
    DependencyTracker immediate_stop_dependencies;
    std::map<std::size_t, DependencyTracker> dependencies;

    using Checkpoints = rmf_traffic::agv::Plan::Checkpoints;
    std::unordered_map<std::size_t, Checkpoints> arrivals;
  };

  struct Location
  {
    std::string map;
    Eigen::Vector3d position;
  };

  struct State
  {
    std::optional<Plan> proposal;
    std::optional<Plan> current_plan;

    // TODO(MXG): Consider removing this field because it is redundant with
    // blockade->path().
    std::vector<rmf_traffic::blockade::Writer::Checkpoint> checkpoints;

    rmf_traffic::blockade::ReservedRange range = {0, 0};
    std::size_t last_reached = 0;
    std::optional<std::size_t> last_passed;
    std::optional<rmf_traffic::agv::Plan::Start> last_known_location;
    std::optional<Location> idle_location;
    std::shared_ptr<rmf_traffic::agv::Planner> planner;
    std::shared_ptr<rmf_traffic::schedule::Participant> itinerary;
    std::shared_ptr<rmf_traffic::blockade::Participant> blockade;

    std::shared_ptr<services::FindPath> find_path_service;
    rxcpp::subscription find_path_subscription;

    void clear();
    rmf_traffic::schedule::Itinerary current_itinerary_slice() const;
    std::optional<Location> location() const;
  };

  using FleetState = rmf_fleet_msgs::msg::FleetState;

  struct Hooks
  {
    std::function<void()> pause_callback;
    std::function<void()> resume_callback;
    std::function<void(std::vector<Blocker> blockers)> deadlock_callback;
    std::shared_ptr<const rmf_traffic::schedule::Mirror> schedule;
    rxcpp::schedulers::worker worker;
    std::shared_ptr<Node> node;
    rmf_traffic::agv::VehicleTraits traits;
    std::shared_ptr<const rmf_traffic::Profile> profile;
    rclcpp::Publisher<FleetState>::SharedPtr fleet_state_pub;
    rclcpp::TimerBase::SharedPtr fleet_update_timer;
  };

  struct Shared : public std::enable_shared_from_this<Shared>
  {
    State state;
    Hooks hooks;
    std::size_t path_version = 0;

    // Things for populating the fleet state
    double battery_soc = 0.0;

    std::string name;

    // Negotiation fields
    std::shared_ptr<void> negotiation_license;
    NegotiateServiceMap negotiate_services;

    std::recursive_mutex mutex;
    std::unique_lock<std::recursive_mutex> lock();

    Shared(Hooks hooks);

    void follow_new_path(const std::vector<Waypoint>& new_path);

    void make_plan(
      std::size_t request_path_version,
      rmf_traffic::agv::Plan::Start start);

    std::optional<rmf_traffic::schedule::ItineraryVersion> receive_plan(
      std::size_t request_path_version,
      rmf_traffic::PlanId plan_id,
      const rmf_traffic::agv::Plan& plan);

    rmf_traffic::agv::Plan::Start current_location() const;

    void update_immediate_stop(
      std::size_t checkpoint,
      std::optional<Eigen::Vector3d> location);

    void update_delay(
      std::size_t checkpoint,
      std::optional<Eigen::Vector3d> location);

    bool update_location(
      std::size_t checkpoint,
      std::optional<Eigen::Vector3d> location);

    bool consider_proposal(
      std::size_t checkpoint,
      std::optional<Eigen::Vector3d> location);

    bool finish_immediate_stop();

    bool check_if_ready(std::size_t to_move_past_checkpoint);

    MovingInstruction moving_from(
      std::size_t checkpoint,
      Eigen::Vector3d location);

    WaitingInstruction waiting_at(std::size_t checkpoint);

    WaitingInstruction waiting_after(
      std::size_t checkpoint,
      Eigen::Vector3d location);

    void update_idle_location(
      std::string map_name,
      Eigen::Vector3d position);

    void receive_new_range(
      const rmf_traffic::blockade::ReservationId reservation_id,
      const rmf_traffic::blockade::ReservedRange& new_range);

    void respond(
      const rmf_traffic::schedule::Negotiator::TableViewerPtr& table_viewer,
      const rmf_traffic::schedule::Negotiator::ResponderPtr& responder);

    void publish_fleet_state() const;
  };

  class Negotiator : public rmf_traffic::schedule::Negotiator
  {
  public:

    Negotiator(const std::shared_ptr<Shared>& shared);

    void respond(
      const TableViewerPtr& table_viewer,
      const ResponderPtr& responder) final;

  private:
    std::weak_ptr<Shared> _shared;
  };

  std::shared_ptr<Shared> shared;

  static EasyTrafficLightPtr make(
    std::function<void()> pause_,
    std::function<void()> resume_,
    std::function<void(std::vector<Blocker>)> blocker_,
    std::shared_ptr<const rmf_traffic::schedule::Mirror> schedule_,
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<Node> node_,
    rmf_traffic::agv::VehicleTraits traits_,
    rmf_traffic::schedule::Participant itinerary_,
    std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer_,
    rmf_traffic_ros2::schedule::Negotiation* negotiation_);
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYTRAFFICLIGHT_HPP
