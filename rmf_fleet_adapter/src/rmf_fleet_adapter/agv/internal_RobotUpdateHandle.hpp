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
#include <rmf_api_msgs/schemas/commission.hpp>

#include <memory>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class TriggerOnce
{
public:
  TriggerOnce(std::function<void()> callback)
  : _callback(std::make_shared<std::shared_ptr<std::function<void()>>>(
        std::make_shared<std::function<void()>>(std::move(callback))))
  {
    // Do nothing
  }

  TriggerOnce()
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

//==============================================================================
struct StubbornOverride
{
  /// We use a convoluted multi-layered reference structure for schedule
  /// override stubbornness so that we can release the stubbornness of the
  /// schedule override after the command is finished, even if the user
  /// forgets to release the override stubbornness.
  ///
  /// If we don't implement it like this, there's a risk that the agent will
  /// remain stubborn after it resumes normal operation, which would cause
  /// significant traffic management problems.
  std::shared_ptr<void> stubbornness;
};

//==============================================================================
struct ScheduleOverride
{
  rmf_traffic::Route route;
  rmf_traffic::PlanId plan_id;
  std::weak_ptr<StubbornOverride> w_stubborn;

  void overridden_update(
    const std::shared_ptr<RobotContext>& context,
    const std::string& map,
    Eigen::Vector3d location);

  void release_stubbornness();

  static std::optional<ScheduleOverride> make(
    const std::shared_ptr<RobotContext>& context,
    const std::string& map,
    const std::vector<Eigen::Vector3d>& path,
    rmf_traffic::Duration hold,
    std::shared_ptr<StubbornOverride> stubborn);
};

//==============================================================================
using LocationUpdateFn = std::function<void(
      const std::string& map,
      Eigen::Vector3d location)>;

//==============================================================================
class RobotUpdateHandle::ActivityIdentifier::Implementation
{
public:
  /// A function that EasyFullControl will use to update the robot location info
  /// when this activity is being executed.
  LocationUpdateFn update_fn;

  static std::shared_ptr<ActivityIdentifier> make(LocationUpdateFn update_fn_)
  {
    auto identifier =
      std::shared_ptr<ActivityIdentifier>(new ActivityIdentifier);
    identifier->_pimpl = rmf_utils::make_unique_impl<Implementation>(
      Implementation{update_fn_});
    return identifier;
  }

  static Implementation& get(ActivityIdentifier& self)
  {
    return *self._pimpl;
  }

  static const Implementation& get(const ActivityIdentifier& self)
  {
    return *self._pimpl;
  }
};

//==============================================================================
class RobotUpdateHandle::ActionExecution::Implementation
{
public:

  struct Data
  {
    std::weak_ptr<RobotContext> w_context;
    TriggerOnce finished;
    std::shared_ptr<rmf_task::events::SimpleEventState> state;
    std::optional<rmf_traffic::Duration> remaining_time;
    bool request_replan;
    bool okay;
    std::optional<ScheduleOverride> schedule_override;

    void update_location(
      const std::string& map,
      Eigen::Vector3d location)
    {
      const auto context = w_context.lock();
      if (!context)
        return;

      const auto nav_params = context->nav_params();
      if (nav_params)
      {
        if (const auto p = nav_params->to_rmf_coordinates(map, location))
        {
          location = *p;
        }
        else
        {
          RCLCPP_ERROR(
            context->node()->get_logger(),
            "[EasyFullControl] Unable to find a robot transform for map [%s] "
            "while updating the location of robot [%s] performing an activity. "
            "We cannot update the robot's location.",
            map.c_str(),
            context->requester_id().c_str());
          return;
        }
      }

      if (schedule_override.has_value())
      {
        return schedule_override->overridden_update(
          context, map, location);
      }

      if (nav_params)
      {
        if (context->debug_positions)
        {
          std::cout << "Searching for location from " << __FILE__ << "|" <<
            __LINE__ << std::endl;
        }
        nav_params->search_for_location(map, location, *context);
      }
    }

    Data(
      const std::shared_ptr<RobotContext>& context_,
      std::function<void()> finished_,
      std::shared_ptr<rmf_task::events::SimpleEventState> state_,
      std::optional<rmf_traffic::Duration> remaining_time_ = std::nullopt)
    : w_context(context_),
      finished(std::move(finished_)),
      state(std::move(state_)),
      remaining_time(remaining_time_),
      request_replan(false),
      okay(true)
    {
      // Do nothing
    }

    ~Data()
    {
      if (const auto context = w_context.lock())
      {
        context->worker().schedule(
          [finished = std::move(this->finished)](const auto&)
          {
            finished.trigger();
          });
      }
    }
  };

  static ActionExecution make(std::shared_ptr<Data> data)
  {
    auto update_fn = [data](
      const std::string& map,
      Eigen::Vector3d location)
      {
        data->update_location(map, location);
      };

    ActionExecution execution;
    execution._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(data),
        ActivityIdentifier::Implementation::make(update_fn)
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

  static std::shared_ptr<RobotUpdateHandle> make(RobotContextPtr context)
  {
    std::string name = context->description().name();

    RobotUpdateHandle handle;
    handle._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::move(context), std::move(name)});
    handle._pimpl->unstable._pimpl =
      &RobotUpdateHandle::Implementation::get(handle);

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

  void set_commission(Commission commission)
  {
    if (const auto context = get_context())
    {
      context->worker().schedule(
        [w = context->weak_from_this(), commission = std::move(commission)](
          const auto&)
        {
          if (const auto context = w.lock())
          {
            context->set_commission(commission);
          }
        });
    }
  }

  Commission commission() const
  {
    if (const auto context = get_context())
    {
      return context->copy_commission();
    }

    return Commission::decommission();
  }

  std::shared_ptr<RobotContext> get_context();

  std::shared_ptr<const RobotContext> get_context() const;
};

//==============================================================================
class RobotUpdateHandle::Unstable::Stubbornness::Implementation
{
public:
  std::shared_ptr<void> stubbornness;

  static Stubbornness make(std::shared_ptr<void> stubbornness)
  {
    Stubbornness output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{stubbornness});

    return output;
  }
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ROBOTUPDATEHANDLE_HPP
