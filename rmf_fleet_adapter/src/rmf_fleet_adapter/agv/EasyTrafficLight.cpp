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

#include "internal_EasyTrafficLight.hpp"

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
void EasyTrafficLight::Implementation::CommandHandle::receive_checkpoints(
    const std::size_t version,
    std::vector<Checkpoint> checkpoints,
    const std::size_t standby_at,
    OnStandby on_standby,
    Reject reject)
{
  auto lock = pimpl->_pimpl->lock();
  pimpl->_pimpl->receive_checkpoints(
        version,
        std::move(checkpoints),
        standby_at,
        std::move(on_standby),
        std::move(reject));
}

//==============================================================================
void EasyTrafficLight::Implementation::CommandHandle::immediately_stop_until(
    const std::size_t version,
    rclcpp::Time time,
    StoppedAt stopped_at,
    Departed departed)
{
  auto lock = pimpl->_pimpl->lock();
  pimpl->_pimpl->immediately_stop_until(
        version, time, std::move(stopped_at), std::move(departed));
}

//==============================================================================
void EasyTrafficLight::Implementation::CommandHandle::resume(
    const std::size_t version)
{
  auto lock = pimpl->_pimpl->lock();
  pimpl->_pimpl->resume(version);
}

//==============================================================================
void EasyTrafficLight::Implementation::CommandHandle::deadlock(
    std::vector<Blocker> blockers)
{
  auto lock = pimpl->_pimpl->lock();
  pimpl->_pimpl->deadlock(std::move(blockers));
}

//==============================================================================
void EasyTrafficLight::Implementation::receive_checkpoints(
    const std::size_t version,
    std::vector<Checkpoint> checkpoints,
    const std::size_t standby_at,
    OnStandby on_standby,
    Reject reject)
{
  if (version != current_version)
    return;

  if (last_received_checkpoints.has_value())
  {
    // If we already have checkpoints that we've received but haven't processed
    // yet, then we should merge the new checkpoints into the unprocessed ones.
    last_received_checkpoints->reject = reject;
    last_received_checkpoints->on_standby = on_standby;
    last_received_checkpoints->standby_at = standby_at;

    auto& old_checkpoints = last_received_checkpoints->checkpoints;
    for (const auto& new_c : checkpoints)
    {
      bool duplicate = false;
      for (auto& old_c : old_checkpoints)
      {
        if (old_c.waypoint_index == new_c.waypoint_index)
        {
          old_c = new_c;
          duplicate = true;
          break;
        }
      }

      if (!duplicate)
      {
        old_checkpoints.push_back(new_c);
      }
    }

    const auto r_it = std::remove_if(
          old_checkpoints.begin(), old_checkpoints.end(),
          [standby_at](const Checkpoint& c)
    {
      return c.waypoint_index >= standby_at;
    });

    old_checkpoints.erase(r_it, old_checkpoints.end());
  }
  else
  {
    last_received_checkpoints = CheckpointInfo{
      std::move(checkpoints),
      standby_at,
      std::move(on_standby),
      std::move(reject)
    };
  }

  if (last_departed_checkpoint.has_value())
  {
    if (standby_at < last_departed_checkpoint.value())
    {
      // If we know that the robot has already passed the standby checkpoint,
      // then we should have it pause immediately.
      pause_cb();
    }
  }
}

//==============================================================================
void EasyTrafficLight::Implementation::immediately_stop_until(
    const std::size_t version,
    rclcpp::Time time,
    StoppedAt stopped_at,
    Departed departed)
{
  if (version != current_version)
    return;

  const auto now = node->now();

  if (time >= now)
    return;

  pause_cb();

  *wait_until = time;
  if (!(*wait_timer))
  {
    // It's okay to capture `this` by value here, because if `this` destructs
    // then so does the wait_timer, in which case this callback will not be
    // triggered.
    *wait_timer =
        node->create_wall_timer(
          std::chrono::milliseconds(100),
          [w_timer = std::weak_ptr<rclcpp::TimerBase::SharedPtr>(wait_timer),
           w_node = std::weak_ptr<rclcpp::Node>(node),
           wait_until = wait_until,
           resume_cb = resume_cb]()
    {
      if (!wait_until->has_value())
      {
        if (const auto timer = w_timer.lock())
          timer->reset();
      }

      const auto node = w_node.lock();
      if (!node)
        return;

      const auto now = node->now();

      if (now <= wait_until->value())
      {
        resume_cb();
        if (const auto timer = w_timer.lock())
          timer->reset();
      }
    });
  }

  last_received_stop_info = ImmediateStopInfo{
    time,
    std::move(stopped_at),
    std::move(departed),
    version
  };
}

//==============================================================================
void EasyTrafficLight::Implementation::resume(std::size_t version)
{
  if (version != current_version)
    return;

  if (last_received_stop_info.has_value())
  {
    assert(last_received_stop_info.value().path_version == current_version);

    resume_info = ResumeInfo {
      last_reached,
      last_received_stop_info.value().departed,
      last_received_stop_info.value().path_version
    };
  }

  last_received_stop_info.reset();
  wait_timer->reset();
  wait_until->reset();

  resume_cb();
}

//==============================================================================
void EasyTrafficLight::Implementation::deadlock(std::vector<Blocker> blockers)
{
  std::stringstream ss;
  ss << "[";
  for (const auto& b : blockers)
  {
    ss << " " << b.participant_id() << "|" << b.description().owner()
       << ":" << b.description().name();
  }

  ss << " ]";

  RCLCPP_ERROR(
        node->get_logger(),
        "[%s] owned by [%s] has encountered permanent deadlock blockers: %s",
        name.c_str(), owner.c_str(), ss.str().c_str());

  if (blocker_cb)
    blocker_cb(std::move(blockers));
}

//==============================================================================
void EasyTrafficLight::Implementation::follow_new_path(
    const std::vector<Waypoint>& new_path)
{
  clear();
  current_path = new_path;
  current_checkpoints.resize(new_path.size()-1);

  current_version = update_handle->follow_new_path(new_path);
}

//==============================================================================
void EasyTrafficLight::Implementation::clear()
{
  current_path.clear();
  last_received_checkpoints.reset();
  last_received_stop_info.reset();
  resume_info.reset();
  wait_until->reset();
  wait_timer->reset();
  standby_at = 0;
  on_standby = nullptr;
  last_departed_checkpoint.reset();
  last_reached = 0;
  current_checkpoints.clear();
}

//==============================================================================
void EasyTrafficLight::Implementation::accept_new_checkpoints()
{
  assert(last_received_checkpoints.has_value());

  for (const auto& c : last_received_checkpoints.value().checkpoints)
    current_checkpoints.at(c.waypoint_index) = c;

  standby_at = last_received_checkpoints.value().standby_at;
  on_standby = last_received_checkpoints.value().on_standby;

  for (std::size_t i = standby_at; i < current_checkpoints.size(); ++i)
    current_checkpoints[i].reset();

  last_received_checkpoints.reset();
}

//==============================================================================
auto EasyTrafficLight::Implementation::handle_new_checkpoints_moving(
    const std::size_t last_departed_checkpoint)
-> std::optional<MovingInstruction>
{
  if (!last_received_checkpoints.has_value())
    return std::nullopt;

  if (last_received_checkpoints.value().standby_at <= last_departed_checkpoint)
  {
    // The robot has already moved past the checkpoint where it's supposed to
    // enter standby. We will tell the robot to pause immediately, and then
    // waiting_at(~) or waiting_after(~,~) can trigger the reject(~) callback.
    return MovingInstruction::PauseImmediately;
  }

  accept_new_checkpoints();

  return std::nullopt;
}

//==============================================================================
auto EasyTrafficLight::Implementation::moving_from(
    const std::size_t checkpoint,
    Eigen::Vector3d location) -> MovingInstruction
{
  const auto now = node->now();
  last_departed_checkpoint = checkpoint;
  last_reached = std::max(last_reached, checkpoint);

  if (checkpoint >= current_checkpoints.size())
  {
    if (current_checkpoints.empty())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "[EasyTrafficLight::moving_from] [%s] owned by [%s] is moving from an "
        "invalid checkpoint [%lu]. This robot currently does not have a path.",
        name.c_str(),
        owner.c_str(),
        checkpoint);
    }
    else
    {
      RCLCPP_WARN(
        node->get_logger(),
        "[EasyTrafficLight::moving_from] [%s] owned by [%s] is moving from "
        "an invalid checkpoint [%lu]. The highest checkpoint value that you "
        "can move from is [%lu].",
        name.c_str(),
        owner.c_str(),
        checkpoint,
        current_checkpoints.size()-1);
    }

    return MovingInstruction::MovingError;
  }

  if (last_received_stop_info.has_value())
  {
    if (last_received_stop_info.value().time < now)
    {
      RCLCPP_WARN(
            node->get_logger(),
            "[EasyTrafficLight::moving_from] [%s] owned by [%s] is moving away "
            "from checkpoint [%lu] when the robot is supposed to be stopped.",
            name.c_str(),
            owner.c_str(),
            checkpoint);
      return MovingInstruction::MovingError;
    }

    last_received_stop_info.reset();
  }

  if (const auto instruction = handle_new_checkpoints_moving(checkpoint))
    return instruction.value();

  if (!resume_info.has_value() || resume_info.value().checkpoint != checkpoint)
  {
    const auto& c = current_checkpoints.at(checkpoint);
    if (!c.has_value())
    {
      assert(standby_at <= checkpoint);
      RCLCPP_WARN(
            node->get_logger(),
            "[EasyTrafficLight::moving_from] [%s] owned by [%s] is moving away "
            "from checkpoint [%lu] when the robot was supposed to standby at "
            "[%lu].",
            name.c_str(),
            owner.c_str(),
            checkpoint, standby_at);
      return MovingInstruction::MovingError;
    }

    c.value().departed(location);
  }
  else
  {
    assert(resume_info.value().path_version == current_version);
    resume_info.value().departed(location);
  }

  assert(checkpoint < standby_at);
  if (checkpoint + 1 == standby_at)
    return MovingInstruction::WaitAtNextCheckpoint;

  return MovingInstruction::ContinueAtNextCheckpoint;
}

//==============================================================================
auto EasyTrafficLight::Implementation::handle_new_checkpoints_waiting(
    const std::optional<std::size_t> departed_checkpoint,
    const Eigen::Vector3d location) -> std::optional<WaitingInstruction>
{
  last_departed_checkpoint = departed_checkpoint;
  if (!last_received_checkpoints.has_value())
    return std::nullopt;

  const auto new_standby_at = last_received_checkpoints.value().standby_at;
  if (last_departed_checkpoint.has_value())
  {
    if (new_standby_at <= last_departed_checkpoint.value())
    {
      auto& reject = last_received_checkpoints.value().reject;
      if (reject)
      {
        reject(last_departed_checkpoint.value(), location);
        reject = nullptr;
      }

      return WaitingInstruction::Wait;
    }
  }

  accept_new_checkpoints();
  return std::nullopt;
}

//==============================================================================
auto EasyTrafficLight::Implementation::handle_immediate_stop(
    const std::size_t departed_checkpoint,
    const Eigen::Vector3d location,
    const rclcpp::Time now) -> std::optional<WaitingInstruction>
{
  if (last_received_stop_info.has_value())
  {
    assert(last_received_stop_info.value().path_version == current_version);

    if (last_received_stop_info.value().stopped_at)
    {
      last_received_stop_info.value().stopped_at(location);

      resume_info = ResumeInfo {
        departed_checkpoint,
        last_received_stop_info.value().departed,
        last_received_stop_info.value().path_version
      };

      last_received_stop_info.value().stopped_at = nullptr;
    }

    if (last_received_stop_info.value().time <= now)
    {
      resume_info = ResumeInfo {
          departed_checkpoint,
          last_received_stop_info.value().departed,
          last_received_stop_info.value().path_version
      };

      last_received_stop_info.reset();

      // Let the parent function decide whether it is ready to go
      return std::nullopt;
    }

    return WaitingInstruction::Wait;
  }

  return std::nullopt;
}

//==============================================================================
auto EasyTrafficLight::Implementation::waiting_at(
    const std::size_t checkpoint) -> WaitingInstruction
{
  if (checkpoint >= current_path.size())
  {
    RCLCPP_WARN(
      node->get_logger(),
      "[EasyTrafficLight::waiting_at] [%s] owned by [%s] is waiting at "
      "checkpoint [%lu] but the highest possible checkpoint is [%lu]",
      name.c_str(),
      owner.c_str(),
      checkpoint,
      current_path.size()-1);
    return WaitingInstruction::WaitingError;
  }

  last_reached = std::max(last_reached, checkpoint);

  const auto location = current_path.at(checkpoint).position();
  const auto departed_checkpoint = checkpoint == 0?
        std::nullopt : std::optional<std::size_t>(checkpoint-1);

  const auto new_checkpoints_instruction =
      handle_new_checkpoints_waiting(departed_checkpoint, location);
  if (new_checkpoints_instruction.has_value())
  {
    return new_checkpoints_instruction.value();
  }

  const auto now = node->now();
  const auto immediate_stop_instruction = handle_immediate_stop(
        last_departed_checkpoint.value_or(0), location, now);

  if (immediate_stop_instruction.has_value())
  {
    return immediate_stop_instruction.value();
  }

  if (checkpoint > standby_at)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "[EasyTrafficLight::waiting_at] [%s] owned by [%s] is waiting at "
      "checkpoint [%lu] but the robot was supposed to standby at checkpoint "
      "[%lu]",
      name.c_str(),
      owner.c_str(),
      checkpoint,
      standby_at);
    return WaitingInstruction::WaitingError;
  }

  if (checkpoint == standby_at)
  {
    if (on_standby)
    {
      on_standby();
      on_standby = nullptr;
    }

    return WaitingInstruction::Wait;
  }

  return WaitingInstruction::Resume;
}

//==============================================================================
auto EasyTrafficLight::Implementation::waiting_after(
    const std::size_t checkpoint,
    const Eigen::Vector3d location) -> WaitingInstruction
{
  if (checkpoint >= current_path.size())
  {
    RCLCPP_WARN(
      node->get_logger(),
      "[EasyTrafficLight::waiting_after] [%s] owned by [%s] waiting after "
      "passing checkpoint [%lu] but the highest possible checkpoint is [%lu]",
      name.c_str(),
      owner.c_str(),
      checkpoint,
      current_path.size()-1);
    return WaitingInstruction::WaitingError;
  }

  last_reached = std::max(last_reached, checkpoint);

  const auto new_checkpoints_instruction =
      handle_new_checkpoints_waiting(checkpoint, location);
  if (new_checkpoints_instruction.has_value())
  {
    return new_checkpoints_instruction.value();
  }

  const auto now = node->now();
  const auto immediate_stop_instruction =
      handle_immediate_stop(checkpoint, location, now);

  if (immediate_stop_instruction.has_value())
  {
    return immediate_stop_instruction.value();
  }

  if (checkpoint >= standby_at)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "[EasyTrafficLight::waiting_after] [%s] owned by [%s] waiting after "
      "passing checkpoint [%lu] but the robot was supposed to standby at "
      "checkpoint [%lu]",
      name.c_str(),
      owner.c_str(),
      checkpoint,
      standby_at);
    return WaitingInstruction::WaitingError;
  }

  return WaitingInstruction::Resume;
}

//==============================================================================
void EasyTrafficLight::follow_new_path(const std::vector<Waypoint>& new_path)
{
  if (new_path.size() < 2)
  {
    throw std::runtime_error(
      "[EasyTrafficLight::follow_new_path] Invalid number of waypoints given ["
      + std::to_string(new_path.size()) + "]. Must be at least 2.");
  }

  auto lock = _pimpl->lock();
  _pimpl->follow_new_path(new_path);
}

//==============================================================================
auto EasyTrafficLight::moving_from(
    const std::size_t checkpoint,
    const Eigen::Vector3d location) -> MovingInstruction
{
  auto lock = _pimpl->lock();
  return _pimpl->moving_from(checkpoint, location);
}

//==============================================================================
auto EasyTrafficLight::waiting_at(
    const std::size_t checkpoint) -> WaitingInstruction
{
  auto lock = _pimpl->lock();
  return _pimpl->waiting_at(checkpoint);
}

//==============================================================================
auto EasyTrafficLight::waiting_after(
    const std::size_t checkpoint,
    const Eigen::Vector3d location) -> WaitingInstruction
{
  auto lock = _pimpl->lock();
  return _pimpl->waiting_after(checkpoint, location);
}

//==============================================================================
std::size_t EasyTrafficLight::last_reached() const
{
  auto lock = _pimpl->lock();
  return _pimpl->last_reached;
}

//==============================================================================
EasyTrafficLight& EasyTrafficLight::update_idle_location(
  std::string map_name,
  Eigen::Vector3d position)
{
  auto lock = _pimpl->lock();
  if (!_pimpl->current_path.empty())
    _pimpl->clear();

  _pimpl->update_handle->update_idle_location(std::move(map_name), position);
  return *this;
}

//==============================================================================
EasyTrafficLight& EasyTrafficLight::update_battery_soc(double battery_soc)
{
  _pimpl->update_handle->update_battery_soc(battery_soc);
  return *this;
}

//==============================================================================
EasyTrafficLight& EasyTrafficLight::fleet_state_publish_period(
    std::optional<rmf_traffic::Duration> value)
{
  _pimpl->update_handle->fleet_state_publish_period(value);
  return *this;
}

//==============================================================================
EasyTrafficLightPtr EasyTrafficLight::Implementation::make(
    TrafficLight::UpdateHandlePtr update_handle_,
    std::function<void()> pause_,
    std::function<void()> resume_,
    std::function<void(std::vector<Blocker>)> blocker_,
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<rclcpp::Node> node_,
    std::string name_,
    std::string owner_)
{
  std::shared_ptr<EasyTrafficLight> handle(new EasyTrafficLight);
  handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
        std::move(update_handle_),
        std::move(pause_),
        std::move(resume_),
        std::move(blocker_),
        std::move(worker_),
        std::move(node_),
        std::move(name_),
        std::move(owner_));

  return handle;
}

//==============================================================================
EasyTrafficLight::Implementation::Implementation(
    TrafficLight::UpdateHandlePtr update_handle_,
    std::function<void()> pause_,
    std::function<void()> resume_,
    std::function<void(std::vector<Blocker>)> blocker_,
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<rclcpp::Node> node_,
    std::string name_,
    std::string owner_)
  : wait_until(std::make_shared<std::optional<rclcpp::Time>>()),
    wait_timer(std::make_shared<rclcpp::TimerBase::SharedPtr>()),
    pause_cb(std::move(pause_)),
    resume_cb(std::move(resume_)),
    blocker_cb(std::move(blocker_)),
    update_handle(std::move(update_handle_)),
    worker(std::move(worker_)),
    node(std::move(node_)),
    name(std::move(name_)),
    owner(std::move(owner_))
{
  // Do nothing
}

//==============================================================================
EasyTrafficLight::EasyTrafficLight()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
