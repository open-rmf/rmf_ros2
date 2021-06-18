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

#include <rmf_fleet_adapter/agv/EasyTrafficLight.hpp>
#include <rmf_fleet_adapter/agv/TrafficLight.hpp>

#include <rmf_traffic/schedule/Participant.hpp>

#include <rmf_rxcpp/RxJobs.hpp>

#include "Node.hpp"

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class EasyTrafficLight::Implementation
{
public:

  using Checkpoint = TrafficLight::CommandHandle::Checkpoint;
  using OnStandby = TrafficLight::CommandHandle::OnStandby;
  using Reject = TrafficLight::CommandHandle::Reject;
  using StoppedAt = TrafficLight::CommandHandle::StoppedAt;
  using Departed = TrafficLight::CommandHandle::Departed;
  using Blocker = TrafficLight::CommandHandle::Blocker;

  class CommandHandle : public TrafficLight::CommandHandle
  {
  public:

    std::shared_ptr<EasyTrafficLight> pimpl;

    CommandHandle() = default;

    void receive_checkpoints(
      std::size_t version,
      std::vector<Checkpoint> checkpoints,
      std::size_t standby_at,
      OnStandby on_standby,
      Reject reject) final;

    void immediately_stop_until(
      std::size_t version,
      rclcpp::Time time,
      StoppedAt stopped_at,
      Departed departed) final;

    void resume(std::size_t version) final;

    void deadlock(std::vector<Blocker> blockers) final;
  };

  struct CheckpointInfo
  {
    std::vector<Checkpoint> checkpoints;
    std::size_t standby_at;
    OnStandby on_standby;
    Reject reject;
  };

  struct ImmediateStopInfo
  {
    rclcpp::Time time;
    StoppedAt stopped_at;
    Departed departed;
    std::size_t path_version;
  };

  struct ResumeInfo
  {
    std::size_t checkpoint;
    Departed departed;
    std::size_t path_version;
  };

  void receive_checkpoints(
    std::size_t version,
    std::vector<Checkpoint> checkpoints,
    std::size_t standby_at,
    OnStandby on_standby,
    Reject reject);

  void immediately_stop_until(
    std::size_t version,
    rclcpp::Time time,
    StoppedAt stopped_at,
    Departed departed);

  void resume(std::size_t version);

  void deadlock(std::vector<Blocker> blockers);

  void follow_new_path(const std::vector<Waypoint>& new_path);

  void clear();

  void accept_new_checkpoints();

  std::optional<MovingInstruction> handle_new_checkpoints_moving(
    const std::size_t last_departed_checkpoint);

  MovingInstruction moving_from(
    std::size_t checkpoint,
    Eigen::Vector3d location);

  std::optional<WaitingInstruction> handle_new_checkpoints_waiting(
    const std::optional<std::size_t> last_departed_checkpoint,
    const Eigen::Vector3d location);

  std::optional<WaitingInstruction> handle_immediate_stop(
    const std::size_t last_departed_checkpoint,
    const Eigen::Vector3d location,
    const rclcpp::Time now);

  WaitingInstruction waiting_at(std::size_t checkpoint);

  WaitingInstruction waiting_after(
    std::size_t checkpoint,
    Eigen::Vector3d location);

  std::size_t get_last_reached() const;

  std::optional<CheckpointInfo> last_received_checkpoints;
  std::optional<ImmediateStopInfo> last_received_stop_info;
  std::optional<ResumeInfo> resume_info;

  std::shared_ptr<std::optional<rclcpp::Time>> wait_until;
  // We use a nested std::shared_ptr here so we can share the timer with its own
  // callback and have the timer clear itself when it's no longer needed.
  std::shared_ptr<rclcpp::TimerBase::SharedPtr> wait_timer;

  std::size_t current_version;
  std::vector<Waypoint> current_path;
  std::vector<std::optional<Checkpoint>> current_checkpoints;
  std::size_t standby_at = 0;
  OnStandby on_standby;

  std::optional<std::size_t> last_departed_checkpoint;
  std::size_t last_reached = 0;

  std::function<void()> pause_cb;
  std::function<void()> resume_cb;
  std::function<void(std::vector<Blocker>)> blocker_cb;

  TrafficLight::UpdateHandlePtr update_handle;

  rxcpp::schedulers::worker worker;
  std::shared_ptr<Node> node;

  std::string name;
  std::string owner;

  mutable std::mutex _mutex;
  std::unique_lock<std::mutex> lock() const
  {
    std::unique_lock<std::mutex> l(_mutex, std::defer_lock);
    while (!l.try_lock())
    {
      // Intentionally busy wait
    }

    return l;
  }

  static EasyTrafficLightPtr make(
    TrafficLight::UpdateHandlePtr update_handle_,
    std::function<void()> pause_,
    std::function<void()> resume_,
    std::function<void(std::vector<Blocker>)> blocker_,
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<Node> node_,
    std::string name_,
    std::string owner_);

  Implementation(
    TrafficLight::UpdateHandlePtr update_handle_,
    std::function<void()> pause_,
    std::function<void()> resume_,
    std::function<void(std::vector<Blocker>)> blocker,
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<Node> node_,
    std::string name_,
    std::string owner_);
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYTRAFFICLIGHT_HPP
