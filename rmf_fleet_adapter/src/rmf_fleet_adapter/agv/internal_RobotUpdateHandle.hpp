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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ROBOTUPDATEHANDLE_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ROBOTUPDATEHANDLE_HPP

#include "RobotContext.hpp"
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>
#include <rmf_api_msgs/schemas/robot_state.hpp>
#include <rmf_api_msgs/schemas/location_2D.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class RobotUpdateHandle::ActionExecution::Implementation
{
public:

  struct Data
  {
    std::function<void()> finished;
    std::shared_ptr<rmf_task::events::SimpleEventState> state;
    std::optional<rmf_traffic::Duration> remaining_time;
    bool okay;
    // TODO: Consider adding a mutex to lock read/write

    Data(
      std::function<void()> finished_,
      std::shared_ptr<rmf_task::events::SimpleEventState> state_,
      std::optional<rmf_traffic::Duration> remaining_time_ = std::nullopt)
    {
      finished = std::move(finished_);
      state = std::move(state_);
      remaining_time = remaining_time_;
      okay = true;
    }

    ~Data()
    {
      if (finished)
        finished();
    }
  };

  static ActionExecution make(std::shared_ptr<Data> data)
  {
    ActionExecution execution;
    execution._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::move(data)});

    return execution;
  }

  std::shared_ptr<Data> data;
};

//==============================================================================
class RobotUpdateHandle::Implementation
{
public:
  std::weak_ptr<RobotContext> context;
  std::string name;
  RobotUpdateHandle::Unstable unstable = RobotUpdateHandle::Unstable();
  bool reported_loss = false;
  std::unordered_map<std::string, nlohmann::json> schema_dictionary = {};


  static std::shared_ptr<RobotUpdateHandle> make(RobotContextPtr context)
  {
    std::string name = context->description().name();

    RobotUpdateHandle handle;
    handle._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::move(context), std::move(name)});
    handle._pimpl->unstable._pimpl =
      &RobotUpdateHandle::Implementation::get(handle);

    // Initialize schema dictionary
    const std::vector<nlohmann::json> schemas = {
      rmf_api_msgs::schemas::robot_state,
      rmf_api_msgs::schemas::location_2D,
    };

    for (const auto& schema : schemas)
    {
      const auto json_uri = nlohmann::json_uri{schema["$id"]};
      handle._pimpl->schema_dictionary.insert({json_uri.url(), schema});
    }

    return std::make_shared<RobotUpdateHandle>(std::move(handle));
  }

  static Implementation& get(RobotUpdateHandle& handle)
  {
    return *handle._pimpl;
  }

  static const Implementation& get(const RobotUpdateHandle& handle)
  {
    return *handle._pimpl;
  }

  std::shared_ptr<RobotContext> get_context();

  std::shared_ptr<const RobotContext> get_context() const;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ROBOTUPDATEHANDLE_HPP
