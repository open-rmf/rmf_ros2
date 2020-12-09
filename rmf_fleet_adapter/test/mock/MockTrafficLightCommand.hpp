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

#ifndef RMF_FLEET_ADAPTER__TEST__MOCK__MOCKTRAFFICLIGHTCOMMAND_HPP
#define RMF_FLEET_ADAPTER__TEST__MOCK__MOCKTRAFFICLIGHTCOMMAND_HPP

#include <rmf_fleet_adapter/agv/TrafficLight.hpp>

#include <rmf_utils/optional.hpp>

#include <condition_variable>

namespace rmf_fleet_adapter_test {

//==============================================================================
class MockTrafficLightCommand
    : public rmf_fleet_adapter::agv::TrafficLight::CommandHandle
{
public:

  void receive_checkpoints(
      const std::size_t version,
      std::vector<Checkpoint> checkpoints,
      const std::size_t,
      std::function<void()> on_standby,
      Reject)
  {
    std::lock_guard<std::mutex> lock(mutex);
    current_version = version;
    current_checkpoints = std::move(checkpoints);
    standby_cb = std::move(on_standby);
    ++command_counter;
    cv.notify_all();
  }

  void deadlock(std::vector<Blocker>) final
  {
    // Do nothing
  }

  void immediately_stop_until(
      std::size_t, rclcpp::Time, StoppedAt, Departed) final
  {
    // Do nothing
  }

  void resume(std::size_t) final
  {
    // Do nothing
  }

  rmf_utils::optional<std::size_t> current_version;
  std::size_t command_counter = 0;
  std::vector<Checkpoint> current_checkpoints;
  std::function<void()> standby_cb;

  std::mutex mutex;
  std::condition_variable cv;

};

} // namespace rmf_fleet_adapter_test


#endif // RMF_FLEET_ADAPTER__TEST__MOCK__MOCKTRAFFICLIGHTCOMMAND_HPP
