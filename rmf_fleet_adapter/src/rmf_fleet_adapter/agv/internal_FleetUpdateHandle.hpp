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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP

#include <rmf_task_msgs/msg/loop.hpp>

#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>

#include "Node.hpp"
#include "RobotContext.hpp"
#include "../TaskManager.hpp"

#include <rmf_traffic/schedule/Snapshot.hpp>

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// This abstract interface class allows us to use the same implementation of
/// FleetUpdateHandle whether we are running it in a distributed system or in a
/// single-process testing environment.
class ParticipantFactory
{
public:

  using ReadyCallback = std::function<void(rmf_traffic::schedule::Participant)>;

  virtual void async_make_participant(
      rmf_traffic::schedule::ParticipantDescription description,
      ReadyCallback ready_callback) = 0;

  virtual ~ParticipantFactory() = default;
};

//==============================================================================
class SimpleParticipantFactory : public ParticipantFactory
{
public:

  SimpleParticipantFactory(
      std::shared_ptr<rmf_traffic::schedule::Writer> writer)
    : _writer{std::move(writer)}
  {
    // Do nothing
  }

  void async_make_participant(
      rmf_traffic::schedule::ParticipantDescription description,
      ReadyCallback ready_callback) final
  {
    ready_callback(
        rmf_traffic::schedule::make_participant(
          std::move(description),
          _writer,
          nullptr)
    );
  }

private:
  std::shared_ptr<rmf_traffic::schedule::Writer> _writer;
};

//==============================================================================
class ParticipantFactoryRos2 : public ParticipantFactory
{
public:

  ParticipantFactoryRos2(
      rmf_traffic_ros2::schedule::WriterPtr writer)
    : _writer{std::move(writer)}
  {
    // Do nothing
  }

  void async_make_participant(
      rmf_traffic::schedule::ParticipantDescription description,
      ReadyCallback ready_callback) final
  {
    _writer->async_make_participant(
          std::move(description),
          std::move(ready_callback));
  }

private:
  rmf_traffic_ros2::schedule::WriterPtr _writer;
};

//==============================================================================
class FleetUpdateHandle::Implementation
{
public:

  std::string name;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  std::shared_ptr<Node> node;
  rxcpp::schedulers::worker worker;
  std::shared_ptr<ParticipantFactory> writer;
  std::shared_ptr<rmf_traffic::schedule::Snappable> snappable;
  std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation;

  rmf_utils::optional<rmf_traffic::Duration> default_maximum_delay =
      std::chrono::nanoseconds(std::chrono::seconds(10));

  AcceptDeliveryRequest accept_delivery = nullptr;
  std::unordered_map<RobotContextPtr, std::shared_ptr<TaskManager>> task_managers = {};

  rclcpp::Publisher<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_state_pub = nullptr;
  rclcpp::TimerBase::SharedPtr fleet_state_timer = nullptr;

  template<typename... Args>
  static std::shared_ptr<FleetUpdateHandle> make(Args&&... args)
  {
    FleetUpdateHandle handle;
    handle._pimpl = rmf_utils::make_unique_impl<Implementation>(
          Implementation{std::forward<Args>(args)...});

    handle._pimpl->fleet_state_pub = handle._pimpl->node->fleet_state();
    handle._pimpl->fleet_state_timer = handle._pimpl->node->create_wall_timer(
          std::chrono::seconds(1), [self = handle._pimpl.get()]()
    {
      self->publish_fleet_state();
    });

    return std::make_shared<FleetUpdateHandle>(std::move(handle));
  }

  struct DeliveryEstimate
  {
    rmf_traffic::Time time = rmf_traffic::Time::max();
    RobotContextPtr robot = nullptr;
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> pickup_start;
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> dropoff_start;
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> finish;
  };

  struct LoopEstimate
  {
    rmf_traffic::Time time = rmf_traffic::Time::max();
    RobotContextPtr robot = nullptr;
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> init_start;
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> loop_start;
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> loop_end;
  };

  static Implementation& get(FleetUpdateHandle& fleet)
  {
    return *fleet._pimpl;
  }

  static const Implementation& get(const FleetUpdateHandle& fleet)
  {
    return *fleet._pimpl;
  }

  // TODO(MXG): Come up with a better design for task dispatch
  rmf_utils::optional<DeliveryEstimate> estimate_delivery(
      const rmf_task_msgs::msg::Delivery& request) const;

  void perform_delivery(
      const rmf_task_msgs::msg::Delivery& request,
      const DeliveryEstimate& estimate);

  rmf_utils::optional<LoopEstimate> estimate_loop(
      const rmf_task_msgs::msg::Loop& request) const;

  void perform_loop(
      const rmf_task_msgs::msg::Loop& request,
      const LoopEstimate& estimate);

  void fleet_state_publish_period(
      std::optional<rmf_traffic::Duration> value);

  void publish_fleet_state() const;
};

//==============================================================================
void request_delivery(
    const rmf_task_msgs::msg::Delivery& request,
    const std::vector<std::shared_ptr<FleetUpdateHandle>>& fleets);

//==============================================================================
void request_loop(
    const rmf_task_msgs::msg::Loop& request,
    const std::vector<std::shared_ptr<FleetUpdateHandle>>& fleets);

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
