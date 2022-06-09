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

#include "internal_RobotUpdateHandle.hpp"
#include "../TaskManager.hpp"

#include <rmf_traffic_ros2/Time.hpp>

#include <iostream>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
std::shared_ptr<RobotContext> RobotUpdateHandle::Implementation::get_context()
{
  auto output = context.lock();
  if (output)
    return output;

  if (reported_loss)
    return nullptr;

  std::cerr << "ERROR: [RobotUpdateHandle] Robot named [" << name << "] is no "
            << "longer available" << std::endl;
  reported_loss = true;
  return nullptr;
}

//==============================================================================
std::shared_ptr<const RobotContext>
RobotUpdateHandle::Implementation::get_context() const
{
  return const_cast<Implementation&>(*this).get_context();
}

//==============================================================================
void RobotUpdateHandle::interrupted()
{
  replan();
}

//==============================================================================
void RobotUpdateHandle::replan()
{
  if (const auto context = _pimpl->get_context())
  {
    context->_interrupt_publisher.get_subscriber().on_next(
      RobotContext::Empty());
  }
}

//==============================================================================
void RobotUpdateHandle::update_position(
  std::size_t waypoint,
  double orientation)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, waypoint, orientation](const auto&)
      {
        context->_location = {
          rmf_traffic::agv::Plan::Start(
            rmf_traffic_ros2::convert(context->node()->now()),
            waypoint, orientation)
        };
      });
  }
}

//==============================================================================
void RobotUpdateHandle::update_position(
  const Eigen::Vector3d& position,
  const std::vector<std::size_t>& lanes)
{
  if (const auto context = _pimpl->get_context())
  {
    if (lanes.empty())
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[RobotUpdateHandle::update_position] No lanes specified for "
        "function signature that requires at least one lane.");
      // *INDENT-ON*
    }

    const auto now = rmf_traffic_ros2::convert(context->node()->now());
    rmf_traffic::agv::Plan::StartSet starts;
    for (const auto l : lanes)
    {
      const auto& graph = context->navigation_graph();
      const auto wp = graph.get_lane(l).exit().waypoint_index();
      starts.push_back(
        {
          now, wp, position[2], Eigen::Vector2d(position.block<2, 1>(0, 0)), l
        });
    }

    context->worker().schedule(
      [context, starts = std::move(starts)](const auto&)
      {
        context->_location = std::move(starts);
      });
  }
}

//==============================================================================
void RobotUpdateHandle::update_position(
  const Eigen::Vector3d& position,
  const std::size_t waypoint)
{
  if (const auto& context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, position, waypoint](const auto&)
      {
        context->_location = {
          rmf_traffic::agv::Plan::Start(
            rmf_traffic_ros2::convert(context->node()->now()),
            waypoint, position[2], Eigen::Vector2d(position.block<2, 1>(0, 0)))
        };
      });
  }
}

//==============================================================================
void RobotUpdateHandle::update_position(
  const std::string& map_name,
  const Eigen::Vector3d& position,
  const double max_merge_waypoint_distance,
  const double max_merge_lane_distance,
  const double min_lane_length)
{
  if (const auto context = _pimpl->get_context())
  {
    const auto now = rmf_traffic_ros2::convert(context->node()->now());
    auto starts = rmf_traffic::agv::compute_plan_starts(
      context->navigation_graph(), map_name, position, now,
      max_merge_waypoint_distance, max_merge_lane_distance,
      min_lane_length);

    if (starts.empty())
    {
      RCLCPP_ERROR(
        context->node()->get_logger(),
        "[RobotUpdateHandle::update_position] The robot [%s] has diverged "
        "from its navigation graph, currently located at <%f, %f, %f> on "
        "map [%s]", context->requester_id().c_str(),
        position[0], position[1], position[2], map_name.c_str());
      return;
    }

    context->worker().schedule(
      [context, starts = std::move(starts)](const auto&)
      {
        context->_location = std::move(starts);
      });
  }
}

//==============================================================================
void RobotUpdateHandle::update_position(
  rmf_traffic::agv::Plan::StartSet position)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, starts = std::move(position)](const auto&)
      {
        context->_location = starts;
      });
  }
}

//==============================================================================
RobotUpdateHandle& RobotUpdateHandle::set_charger_waypoint(
  const std::size_t charger_wp)
{
  if (const auto context = _pimpl->get_context())
  {
    auto end_state = context->current_task_end_state();
    end_state.dedicated_charging_waypoint(charger_wp);
    context->current_task_end_state(end_state);
    RCLCPP_INFO(
      context->node()->get_logger(),
      "Charger waypoint for robot [%s] set to index [%ld]",
      context->name().c_str(),
      charger_wp);
  }

  return *this;
}

//==============================================================================
void RobotUpdateHandle::update_battery_soc(const double battery_soc)
{
  if (battery_soc < 0.0 || battery_soc > 1.0)
    return;

  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, battery_soc](const auto&)
      {
        context->current_battery_soc(battery_soc);
      });
  }
}

//==============================================================================
void RobotUpdateHandle::override_status(std::optional<std::string> status)
{

  if (const auto context = _pimpl->get_context())
  {

    if (status.has_value())
    {
      // Here we capture [this] to avoid potential costly copy of
      // schema_dictionary when more enties are inserted in the future.
      // It is permissible here since the lambda will only be used within the
      // scope of this function.
      const auto loader =
        [context, this](
        const nlohmann::json_uri& id,
        nlohmann::json& value)
        {
          const auto it = _pimpl->schema_dictionary.find(id.url());
          if (it == _pimpl->schema_dictionary.end())
          {
            RCLCPP_ERROR(
              context->node()->get_logger(),
              "url: %s not found in schema dictionary. "
              "Status for robot [%s] will not be overwritten.",
              id.url().c_str(),
              context->name().c_str());
            return;
          }

          value = it->second;
        };

      try
      {
        static const auto validator =
          nlohmann::json_schema::json_validator(
          rmf_api_msgs::schemas::robot_state, loader);

        nlohmann::json dummy_msg;
        dummy_msg["status"] = status.value();
        validator.validate(dummy_msg);

      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(
          context->node()->get_logger(),
          "Encountered error: %s. Please ensure the override status is a "
          "valid string as per the robot_state.json schema. The status for "
          "robot [%s] will not over overwritten.",
          e.what(),
          context->name().c_str()
        );
        return;
      }
    }

    context->worker().schedule(
      [context, status](const auto&)
      {
        context->override_status(status);
      });
  }
}

//==============================================================================
RobotUpdateHandle& RobotUpdateHandle::maximum_delay(
  rmf_utils::optional<rmf_traffic::Duration> value)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, value](const auto&)
      {
        context->maximum_delay(value);
      });
  }

  return *this;
}

//==============================================================================
rmf_utils::optional<rmf_traffic::Duration>
RobotUpdateHandle::maximum_delay() const
{
  if (const auto context = _pimpl->get_context())
    return context->maximum_delay();

  return rmf_utils::nullopt;
}

//==============================================================================
void RobotUpdateHandle::set_action_executor(
  RobotUpdateHandle::ActionExecutor action_executor)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, action_executor](const auto&)
      {
        context->action_executor(action_executor);
      });
  }
}

//==============================================================================
void RobotUpdateHandle::submit_direct_request(
  nlohmann::json task_request,
  std::string request_id,
  std::function<void(nlohmann::json)> receive_response)
{
  auto context_missing_error = [receive_response]()
    {
      nlohmann::json response;
      response["success"] = false;

      nlohmann::json error;
      error["code"] = 18;
      error["category"] = "Shutdown";
      error["detail"] = "Robot is shutting down";

      response["errors"] = std::vector<nlohmann::json>({std::move(error)});
      receive_response(response);
    };

  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [
        task_request = std::move(task_request),
        request_id = std::move(request_id),
        receive_response,
        c = context->weak_from_this(),
        context_missing_error
      ](const auto&)
      {
        const auto context = c.lock();
        if (!context)
          return context_missing_error();

        const auto mgr = context->task_manager();
        if (!mgr)
          return context_missing_error();

        auto response = mgr->submit_direct_request(task_request, request_id);
        receive_response(std::move(response));
      });
  }
  else
  {
    context_missing_error();
  }
}

//==============================================================================
class RobotUpdateHandle::Interruption::Implementation
{
public:
  std::shared_ptr<TaskManager::Interruption> interruption;

  static Interruption make()
  {
    Interruption output;
    output._pimpl->interruption = std::make_shared<TaskManager::Interruption>();

    return output;
  }

  static std::shared_ptr<TaskManager::Interruption> get_impl(
    Interruption& handle)
  {
    return handle._pimpl->interruption;
  }
};

//==============================================================================
void RobotUpdateHandle::Interruption::resume(
  std::vector<std::string> labels)
{
  _pimpl->interruption->resume(std::move(labels));
}

//==============================================================================
RobotUpdateHandle::Interruption::Interruption()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto RobotUpdateHandle::interrupt(
  std::vector<std::string> labels,
  std::function<void()> robot_is_interrupted)
-> Interruption
{
  Interruption handle = Interruption::Implementation::make();
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [
        labels = std::move(labels),
        robot_is_interrupted = std::move(robot_is_interrupted),
        c = context->weak_from_this(),
        handle = Interruption::Implementation::get_impl(handle)
      ](const auto&)
      {
        const auto context = c.lock();
        if (!context)
          return;

        const auto mgr = context->task_manager();
        if (!mgr)
          return;

        handle->w_mgr = mgr;
        mgr->interrupt_robot(
          handle,
          std::move(labels),
          std::move(robot_is_interrupted));
      });
  }

  return handle;
}

//==============================================================================
void RobotUpdateHandle::cancel_task(
  std::string task_id,
  std::vector<std::string> labels,
  std::function<void(bool)> on_cancellation)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [
        task_id = std::move(task_id),
        labels = std::move(labels),
        on_cancellation = std::move(on_cancellation),
        c = context->weak_from_this()
      ](const auto&)
      {
        const auto context = c.lock();
        if (!context)
          return;

        const auto mgr = context->task_manager();
        if (!mgr)
          return;

        const auto result = mgr->cancel_task(task_id, labels);
        if (on_cancellation)
          on_cancellation(result);
      });
  }
}

//==============================================================================
void RobotUpdateHandle::kill_task(
  std::string task_id,
  std::vector<std::string> labels,
  std::function<void(bool)> on_kill)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [
        task_id = std::move(task_id),
        labels = std::move(labels),
        on_kill = std::move(on_kill),
        c = context->weak_from_this()
      ](const auto&)
      {
        const auto context = c.lock();
        if (!context)
          return;

        const auto mgr = context->task_manager();
        if (!mgr)
          return;

        const auto result = mgr->kill_task(task_id, labels);
        if (on_kill)
          on_kill(result);
      });
  }
}

//==============================================================================
class RobotUpdateHandle::IssueTicket::Implementation
{
public:

  std::unique_ptr<Reporting::Ticket> ticket;

  static IssueTicket make(std::unique_ptr<Reporting::Ticket> ticket)
  {
    IssueTicket output;
    output._pimpl = rmf_utils::make_unique_impl<Implementation>(
      Implementation{std::move(ticket)});
    return output;
  }
};

//==============================================================================
void RobotUpdateHandle::IssueTicket::resolve(nlohmann::json msg)
{
  _pimpl->ticket->resolve(std::move(msg));
}

//==============================================================================
RobotUpdateHandle::IssueTicket::IssueTicket()
{
  // Do nothing
}

//==============================================================================
auto RobotUpdateHandle::create_issue(
  Tier tier, std::string category, nlohmann::json detail) -> IssueTicket
{
  const auto context = _pimpl->get_context();
  if (!context)
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[RobotUpdateHandle::create_issue] Robot context is unavailable.");
    // *INDENT-ON*
  }

  auto inner_tier = [](Tier tier) -> rmf_task::Log::Tier
    {
      switch (tier)
      {
        case Tier::Info: return rmf_task::Log::Tier::Info;
        case Tier::Warning: return rmf_task::Log::Tier::Warning;
        case Tier::Error: return rmf_task::Log::Tier::Error;
        default: return rmf_task::Log::Tier::Uninitialized;
      }
    } (tier);

  auto ticket = context->reporting()
    .create_issue(inner_tier, std::move(category), std::move(detail));

  return RobotUpdateHandle::IssueTicket::Implementation
    ::make(std::move(ticket));
}

//==============================================================================
void RobotUpdateHandle::log_info(std::string text)
{
  const auto context = _pimpl->get_context();

  // Should we throw an exception when the context is gone?
  if (!context)
    return;

  auto& report = context->reporting();
  std::lock_guard<std::mutex> lock(report.mutex());
  report.log().info(std::move(text));
}

//==============================================================================
void RobotUpdateHandle::log_warning(std::string text)
{
  const auto context = _pimpl->get_context();
  if (!context)
    return;

  auto& report = context->reporting();
  std::lock_guard<std::mutex> lock(report.mutex());
  report.log().warn(std::move(text));
}

//==============================================================================
void RobotUpdateHandle::log_error(std::string text)
{
  const auto context = _pimpl->get_context();
  if (!context)
    return;

  auto& report = context->reporting();
  std::lock_guard<std::mutex> lock(report.mutex());
  report.log().error(std::move(text));
}

//==============================================================================
RobotUpdateHandle::RobotUpdateHandle()
{
  // Do nothing
}

//==============================================================================
RobotUpdateHandle::Unstable& RobotUpdateHandle::unstable()
{
  return _pimpl->unstable;
}

//==============================================================================
const RobotUpdateHandle::Unstable& RobotUpdateHandle::unstable() const
{
  return _pimpl->unstable;
}

//==============================================================================
rmf_traffic::schedule::Participant*
RobotUpdateHandle::Unstable::get_participant()
{
  if (const auto context = _pimpl->get_context())
  {
    auto& itinerary = context->itinerary();
    return &itinerary;
  }
  return nullptr;
}

//==============================================================================
void RobotUpdateHandle::Unstable::declare_holding(
  std::string on_map,
  Eigen::Vector3d at_position,
  rmf_traffic::Duration for_duration)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [
        w = context->weak_from_this(),
        on_map = std::move(on_map),
        at_position,
        for_duration
      ](const auto&)
      {
        if (const auto context = w.lock())
        {
          const auto now = context->now();
          const auto zero = Eigen::Vector3d::Zero();
          rmf_traffic::Trajectory holding;
          holding.insert(now, at_position, zero);
          holding.insert(now + for_duration, at_position, zero);

          context->itinerary().set(
            context->itinerary().assign_plan_id(),
            {{std::move(on_map), std::move(holding)}});
        }
      });
  }
}

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

//==============================================================================
void RobotUpdateHandle::Unstable::Stubbornness::release()
{
  _pimpl->stubbornness = nullptr;
}

//==============================================================================
RobotUpdateHandle::Unstable::Stubbornness::Stubbornness()
{
  // Do nothing
}

//==============================================================================
auto RobotUpdateHandle::Unstable::be_stubborn() -> Stubbornness
{
  if (auto context = _pimpl->get_context())
    return Stubbornness::Implementation::make(context->be_stubborn());

  return Stubbornness::Implementation::make(nullptr);
}

//==============================================================================
void RobotUpdateHandle::Unstable::set_lift_entry_watchdog(
  Watchdog watchdog,
  rmf_traffic::Duration wait_duration)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, watchdog = std::move(watchdog), wait_duration](const auto&)
      {
        context->set_lift_entry_watchdog(watchdog, wait_duration);
      });
  }
}

//==============================================================================
void RobotUpdateHandle::ActionExecution::update_remaining_time(
  rmf_traffic::Duration remaining_time_estimate)
{
  _pimpl->data->remaining_time = remaining_time_estimate;
}

//==============================================================================
void RobotUpdateHandle::ActionExecution::underway(
  std::optional<std::string> text)
{
  _pimpl->data->state->update_status(rmf_task::Event::Status::Underway);
  if (text.has_value())
    _pimpl->data->state->update_log().info(*text);
}

//==============================================================================
void RobotUpdateHandle::ActionExecution::error(
  std::optional<std::string> text)
{
  _pimpl->data->state->update_status(rmf_task::Event::Status::Error);
  if (text.has_value())
    _pimpl->data->state->update_log().error(*text);
}

//==============================================================================
void RobotUpdateHandle::ActionExecution::delayed(
  std::optional<std::string> text)
{
  _pimpl->data->state->update_status(rmf_task::Event::Status::Delayed);
  if (text.has_value())
    _pimpl->data->state->update_log().warn(*text);
}

//==============================================================================
void RobotUpdateHandle::ActionExecution::blocked(
  std::optional<std::string> text)
{
  _pimpl->data->state->update_status(rmf_task::Event::Status::Blocked);
  if (text.has_value())
    _pimpl->data->state->update_log().warn(*text);
}

//==============================================================================
void RobotUpdateHandle::ActionExecution::finished()
{
  if (!_pimpl->data->finished)
    return;

  _pimpl->data->finished();
  _pimpl->data->finished = nullptr;
}

//==============================================================================
bool RobotUpdateHandle::ActionExecution::okay() const
{
  return _pimpl->data->okay;
}

//==============================================================================
RobotUpdateHandle::ActionExecution::ActionExecution()
{
  // Do nothing
}


} // namespace agv
} // namespace rmf_fleet_adapter
