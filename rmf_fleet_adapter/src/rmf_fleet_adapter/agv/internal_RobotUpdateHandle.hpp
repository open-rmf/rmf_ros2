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

class TriggerOnce {
public:
  TriggerOnce(std::function<void()> callback)
    : _callback(std::make_shared(std::make_shared(std::move(callback))))
  {
    // Do nothing
  }

  void trigger() const
  {
    if (!_callback)
    {
      return;
    }

    // We don't use a mutex because we assume this will always be triggered
    // inside the shared worker.
    std::shared_ptr<std::function<void()>> inner = *_callback;
    if (inner)
    {
      if (*inner)
      {
        (*inner)();
      }

      *inner = nullptr;
    }
  }

private:
  std::shared_ptr<std::shared_ptr<std::function<void()>>> _callback;
};

using LocationUpdateFn = std::function<void(
  const std::string& map,
  Eigen::Vector3d location)>;

//==============================================================================
class ActivityIdentifier::Implementation
{
public:
  /// A function that EasyFullControl will use to update the robot location info
  /// when this activity is being executed.
  LocationUpdateFn update_fn;

  static std::shared_ptr<ActivityIdentifier> make(LocationUpdateFn update_fn_)
  {
    auto identifier = std::shared_ptr<ActivityIdentifier>(new ActivityIdentifier);
    identifier->_pimpl = rmf_utils::make_unique_impl<Implementation>(
      Implementation{update_fn_});
    return identifier;
  }
};

//==============================================================================
class RobotUpdateHandle::ActionExecution::Implementation
{
public:

  struct Data
  {
    rxcpp::schedulers::worker worker;
    TriggerOnce finished;
    std::shared_ptr<rmf_task::events::SimpleEventState> state;
    std::optional<rmf_traffic::Duration> remaining_time;
    bool request_replan;
    std::optional<std::shared_ptr<RobotUpdateHandle>> handle;
    bool okay;

    Data(
      rxcpp::schedulers::worker worker_,
      std::function<void()> finished_,
      std::shared_ptr<rmf_task::events::SimpleEventState> state_,
      std::optional<rmf_traffic::Duration> remaining_time_ = std::nullopt,
      std::optional<std::shared_ptr<RobotUpdateHandle>> handle_ = std::nullopt)
      : finished(std::move(finished_))
    {
      worker = std::move(worker_);
      state = std::move(state_);
      remaining_time = remaining_time_;
      request_replan = false;
      handle = handle_;
      okay = true;
    }

    ~Data()
    {
      worker.schedule(
        [finished = std::move(this->finished)](
          const rxcpp::schedulers::schedulable&)
        {
          finished.trigger();
        });
    }
  };

  static ActionExecution make(std::shared_ptr<Data> data)
  {
    ActionExecution execution;
    execution._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(data)},
        ActivityIdentifier::Implementation::make(nullptr)
      });

    return execution;
  }

  std::shared_ptr<Data> data;
  std::shared_ptr<const ActivityIdentifier> identifier;
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
