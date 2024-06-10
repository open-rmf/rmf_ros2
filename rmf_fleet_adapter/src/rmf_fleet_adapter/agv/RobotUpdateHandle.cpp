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

#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_utils/math.hpp>

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
    context->request_replan();
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
        rmf_traffic::agv::Plan::StartSet starts = {
          rmf_traffic::agv::Plan::Start(
            rmf_traffic_ros2::convert(context->node()->now()),
            waypoint, orientation)
        };
        if (context->debug_positions)
        {
          std::stringstream ss;
          ss << __FILE__ << "|" << __LINE__ << ": " << starts.size()
             << " starts:" << print_starts(starts, context->navigation_graph());
          std::cout << ss.str() << std::endl;
        }
        context->set_location(starts);
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
        if (context->debug_positions)
        {
          std::stringstream ss;
          ss << __FILE__ << "|" << __LINE__ << ": " << starts.size()
             << " starts:" << print_starts(starts, context->navigation_graph());
          std::cout << ss.str() << std::endl;
        }
        context->set_location(std::move(starts));
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
        rmf_traffic::agv::Plan::StartSet starts = {
          rmf_traffic::agv::Plan::Start(
            rmf_traffic_ros2::convert(context->node()->now()),
            waypoint, position[2], Eigen::Vector2d(position.block<2, 1>(0, 0)))
        };
        if (context->debug_positions)
        {
          std::stringstream ss;
          ss << __FILE__ << "|" << __LINE__ << ": " << starts.size()
             << " starts:" << print_starts(starts, context->navigation_graph());
          std::cout << ss.str() << std::endl;
        }
        context->set_location(std::move(starts));
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

      context->worker().schedule(
        [context, now, map_name, position](
          const auto&)
        {
          if (context->debug_positions)
          {
            std::cout << __FILE__ << "|" << __LINE__ << ": setting robot to LOST | "
                      << map_name << " <" << position.block<2, 1>(0,
            0).transpose()
                      << "> orientation " << position[2] * 180.0 / M_PI << std::endl;
          }
          context->set_lost(Location { now, map_name, position });
        });
      return;
    }

    context->worker().schedule(
      [context, starts = std::move(starts)](const auto&)
      {
        if (context->debug_positions)
        {
          std::stringstream ss;
          ss << __FILE__ << "|" << __LINE__ << ": " << starts.size()
             << " starts:" << print_starts(starts, context->navigation_graph());
          std::cout << ss.str() << std::endl;
        }
        context->set_location(std::move(starts));
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
        if (context->debug_positions)
        {
          std::stringstream ss;
          ss << __FILE__ << "|" << __LINE__ << ": " << starts.size()
             << " starts:" << print_starts(starts, context->navigation_graph());
          std::cout << ss.str() << std::endl;
        }
        context->set_location(starts);
      });
  }
}

//==============================================================================
RobotUpdateHandle& RobotUpdateHandle::set_charger_waypoint(
  const std::size_t charger_wp)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule([charger_wp, w = context->weak_from_this()](
        const auto&)
      {
        const auto self = w.lock();
        if (!self)
          return;

        self->_set_charging(charger_wp, false);
        RCLCPP_INFO(
          self->node()->get_logger(),
          "Charger waypoint for robot [%s] set to index [%ld]",
          self->requester_id().c_str(),
          charger_wp);
      });
  }

  return *this;
}

//==============================================================================
void RobotUpdateHandle::update_battery_soc(const double battery_soc)
{
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
std::function<void(const nlohmann::json_uri& id, nlohmann::json& value)>
make_schema_loader(const rclcpp::Node::SharedPtr& node)
{
  // Initialize schema dictionary
  const std::vector<nlohmann::json> schemas = {
    rmf_api_msgs::schemas::robot_state,
    rmf_api_msgs::schemas::location_2D,
    rmf_api_msgs::schemas::commission,
  };

  std::unordered_map<std::string, nlohmann::json> schema_dictionary;

  for (const auto& schema : schemas)
  {
    const auto json_uri = nlohmann::json_uri{schema["$id"]};
    schema_dictionary.insert({json_uri.url(), schema});
  }

  return [schema_dictionary = std::move(schema_dictionary), node](
    const nlohmann::json_uri& id,
    nlohmann::json& value)
    {
      const auto it = schema_dictionary.find(id.url());
      if (it == schema_dictionary.end())
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "url: %s not found in schema dictionary. "
          "Status for robot will not be overwritten.",
          id.url().c_str());
        return;
      }

      value = it->second;
    };
}

//==============================================================================
void RobotUpdateHandle::override_status(std::optional<std::string> status)
{
  if (const auto context = _pimpl->get_context())
  {
    if (status.has_value())
    {
      try
      {
        static const auto validator =
          nlohmann::json_schema::json_validator(
          rmf_api_msgs::schemas::robot_state,
          make_schema_loader(context->node()));

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
bool RobotUpdateHandle::ActivityIdentifier::operator==(
  const ActivityIdentifier& other) const
{
  return _pimpl == other._pimpl;
}

//==============================================================================
RobotUpdateHandle::ActivityIdentifier::ActivityIdentifier()
{
  // Do nothing
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
const std::string RobotUpdateHandle::current_task_id() const
{
  if (const auto context = _pimpl->get_context())
    return context->copy_current_task_id();

  return {};
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
void RobotUpdateHandle::enable_responsive_wait(bool value)
{
  const auto context = _pimpl->get_context();
  if (!context)
    return;

  context->worker().schedule(
    [mgr = context->task_manager(), value](const auto&)
    {
      mgr->enable_responsive_wait(value);
    });
}

//==============================================================================
void RobotUpdateHandle::release_lift()
{
  const auto context = _pimpl->get_context();
  if (!context)
    return;

  context->worker().schedule(
    [context](const auto&)
    {
      if (const auto* lift = context->current_lift_destination())
      {
        RCLCPP_INFO(
          context->node()->get_logger(),
          "Releasing lift [%s] for [%s] because of a user request",
          lift->lift_name.c_str(),
          context->requester_id().c_str());
      }
      context->release_lift();
    });
}

//==============================================================================
class RobotUpdateHandle::Commission::Implementation
{
public:
  bool is_accepting_dispatched_tasks = true;
  bool is_accepting_direct_tasks = true;
  bool is_performing_idle_behavior = true;
};

//==============================================================================
RobotUpdateHandle::Commission::Commission()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto RobotUpdateHandle::Commission::decommission() -> Commission
{
  return Commission()
    .accept_dispatched_tasks(false)
    .accept_direct_tasks(false)
    .perform_idle_behavior(false);
}

//==============================================================================
auto RobotUpdateHandle::Commission::accept_dispatched_tasks(bool decision)
-> Commission&
{
  _pimpl->is_accepting_dispatched_tasks = decision;
  return *this;
}

//==============================================================================
bool RobotUpdateHandle::Commission::is_accepting_dispatched_tasks() const
{
  return _pimpl->is_accepting_dispatched_tasks;
}

//==============================================================================
auto RobotUpdateHandle::Commission::accept_direct_tasks(bool decision)
-> Commission&
{
  _pimpl->is_accepting_direct_tasks = decision;
  return *this;
}

//==============================================================================
bool RobotUpdateHandle::Commission::is_accepting_direct_tasks() const
{
  return _pimpl->is_accepting_direct_tasks;
}

//==============================================================================
auto RobotUpdateHandle::Commission::perform_idle_behavior(bool decision)
-> Commission&
{
  _pimpl->is_performing_idle_behavior = decision;
  return *this;
}

//==============================================================================
bool RobotUpdateHandle::Commission::is_performing_idle_behavior() const
{
  return _pimpl->is_performing_idle_behavior;
}

//==============================================================================
void RobotUpdateHandle::set_commission(Commission commission)
{
  _pimpl->set_commission(std::move(commission));
}

//==============================================================================
auto RobotUpdateHandle::commission() const -> Commission
{
  return _pimpl->commission();
}

//==============================================================================
void RobotUpdateHandle::reassign_dispatched_tasks()
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context](const auto&)
      {
        const auto mgr = context->task_manager();
        if (mgr)
          mgr->reassign_dispatched_requests([]() {}, [](auto) {});
      });
  }
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
bool RobotUpdateHandle::Unstable::is_commissioned() const
{
  if (const auto context = _pimpl->get_context())
    return context->copy_commission().is_accepting_dispatched_tasks();

  return false;
}

//==============================================================================
void RobotUpdateHandle::Unstable::decommission()
{
  _pimpl->set_commission(
    _pimpl->commission()
    .accept_dispatched_tasks(false));
}

//==============================================================================
void RobotUpdateHandle::Unstable::recommission()
{
  _pimpl->set_commission(
    _pimpl->commission()
    .accept_dispatched_tasks(true));
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
void RobotUpdateHandle::Unstable::change_participant_profile(
  double footprint_radius,
  double vicinity_radius)
{
  const auto vicinity = [&]() -> rmf_traffic::geometry::FinalConvexShapePtr
    {
      if (vicinity_radius <= footprint_radius)
        return nullptr;

      return rmf_traffic::geometry::make_final_convex(
        rmf_traffic::geometry::Circle(vicinity_radius));
    } ();

  const auto footprint = rmf_traffic::geometry::make_final_convex(
    rmf_traffic::geometry::Circle(footprint_radius));

  rmf_traffic::Profile profile(footprint, vicinity);
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [
        w = context->weak_from_this(),
        profile = std::move(profile)
      ](const auto&)
      {
        if (const auto context = w.lock())
        {
          context->itinerary().change_profile(profile);
        }
      });
  }
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
rmf_traffic::PlanId RobotUpdateHandle::Unstable::current_plan_id() const
{
  return _pimpl->get_context()->itinerary().current_plan_id();
}

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
void RobotUpdateHandle::Unstable::debug_positions(bool on)
{
  if (const auto context = _pimpl->get_context())
  {
    // No need to worry about race conditions or data races here because this is
    // a mostly inconsequential bool
    context->debug_positions = on;
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
auto RobotUpdateHandle::ActionExecution::override_schedule(
  std::string map,
  std::vector<Eigen::Vector3d> path,
  rmf_traffic::Duration hold) -> Stubbornness
{
  auto stubborn = std::make_shared<StubbornOverride>();
  if (const auto context = _pimpl->data->w_context.lock())
  {
    context->worker().schedule(
      [
        context,
        stubborn,
        data = _pimpl->data,
        identifier = _pimpl->identifier,
        map = std::move(map),
        path = std::move(path),
        hold
      ](const auto&)
      {
        if (!ActivityIdentifier::Implementation::get(*identifier).update_fn)
        {
          // Don't do anything because this command is finished
          return;
        }

        if (data->schedule_override.has_value())
        {
          data->schedule_override->release_stubbornness();
        }
        data->schedule_override = ScheduleOverride::make(
          context, map, path, hold, stubborn);
      }
    );
  }

  return Stubbornness::Implementation::make(stubborn);
}

//==============================================================================
void RobotUpdateHandle::ActionExecution::finished()
{
  if (_pimpl->data)
  {
    if (const auto context = _pimpl->data->w_context.lock())
    {
      context->worker().schedule(
        [finished = _pimpl->data->finished](const auto&)
        {
          finished.trigger();
        });
    }
  }
}

//==============================================================================
bool RobotUpdateHandle::ActionExecution::okay() const
{
  return _pimpl->data->okay;
}

//==============================================================================
auto RobotUpdateHandle::ActionExecution::identifier() const
-> ConstActivityIdentifierPtr
{
  return _pimpl->identifier;
}

//==============================================================================
RobotUpdateHandle::ActionExecution::ActionExecution()
{
  // Do nothing
}

//==============================================================================
void ScheduleOverride::overridden_update(
  const std::shared_ptr<RobotContext>& context,
  const std::string& map,
  Eigen::Vector3d location)
{
  const auto nav_params = context->nav_params();
  if (!nav_params)
    return;

  auto p = Eigen::Vector2d(location[0], location[1]);
  std::optional<std::pair<std::size_t, double>> closest_lane;
  std::size_t i0 = 0;
  std::size_t i1 = 1;
  const double proj_lower_bound = -nav_params->max_merge_lane_distance;
  for (; i1 < route.trajectory().size(); ++i0, ++i1)
  {
    // We approximate the trajectory as linear with constant velocity even
    // though it could technically be a cubic spline. The linear
    // approximation simplifies the math considerably, and we will be
    // phasing out support for cubic splines in the future.
    const Eigen::Vector2d p0 =
      route.trajectory().at(i0).position().block<2, 1>(0, 0);
    const Eigen::Vector2d p1 =
      route.trajectory().at(i1).position().block<2, 1>(0, 0);
    const auto lane_length = (p1 - p0).norm();
    if (lane_length < 1e-6)
    {
      const double dist_to_lane = (p - p0).norm();
      if (dist_to_lane > nav_params->max_merge_lane_distance)
      {
        continue;
      }

      if (!closest_lane.has_value() || dist_to_lane < closest_lane->second)
      {
        closest_lane = std::make_pair(i0, dist_to_lane);
      }
    }
    else
    {
      const auto lane_u = (p1 - p0)/lane_length;
      const auto proj = (p - p0).dot(lane_u);
      const double proj_upper_bound =
        lane_length + nav_params->max_merge_lane_distance;
      if (proj < proj_lower_bound || proj_upper_bound < proj)
      {
        continue;
      }

      double dist_to_lane = (p - p0 - proj * lane_u).norm();
      if (proj < 0.0)
        dist_to_lane += std::abs(proj);
      else if (lane_length < proj)
        dist_to_lane += std::abs(proj - lane_length);

      if (!closest_lane.has_value() || dist_to_lane < closest_lane->second)
      {
        closest_lane = std::make_pair(i0, dist_to_lane);
      }
    }
  }

  const auto now = rmf_traffic_ros2::convert(context->node()->now());
  const auto delay_thresh = std::chrono::milliseconds(100);
  if (closest_lane.has_value())
  {
    const auto& wp0 = route.trajectory().at(closest_lane->first);
    const auto& wp1 = route.trajectory().at(closest_lane->first + 1);
    const Eigen::Vector2d p0 = wp0.position().block<2, 1>(0, 0);
    const Eigen::Vector2d p1 = wp1.position().block<2, 1>(0, 0);
    rmf_traffic::Time t_expected = wp0.time();
    const auto lane_length = (p1 - p0).norm();
    if (lane_length > 1e-6)
    {
      const auto lane_u = (p1 - p0)/lane_length;
      const auto proj = (p - p0).dot(lane_u);
      const auto s = proj/lane_length;
      const double dt = rmf_traffic::time::to_seconds(wp1.time() - wp0.time());
      t_expected += rmf_traffic::time::from_seconds(s*dt);
    }
    else
    {
      const double total_delta_yaw =
        rmf_utils::wrap_to_pi(wp1.position()[2] - wp0.position()[2]);
      const double remaining_delta_yaw =
        rmf_utils::wrap_to_pi(wp1.position()[2] - location[2]);
      const double s = remaining_delta_yaw / total_delta_yaw;
      const double dt = rmf_traffic::time::to_seconds(wp1.time() - wp0.time());
      t_expected += rmf_traffic::time::from_seconds(s*dt);
    }
    const auto delay = now - t_expected;
    context->itinerary().cumulative_delay(plan_id, delay, delay_thresh);
  }
  else
  {
    // Find the waypoint that the agent is closest to and estimate the delay
    // based on the agent being at that waypoint. This is a very fallible
    // estimation, but it's the best we can do with limited information.
    std::optional<std::pair<rmf_traffic::Time, double>> closest_time;
    for (std::size_t i = 0; i < route.trajectory().size(); ++i)
    {
      const auto& wp = route.trajectory().at(i);
      const Eigen::Vector2d p_wp = wp.position().block<2, 1>(0, 0);
      const double dist = (p - p_wp).norm();
      if (!closest_time.has_value() || dist < closest_time->second)
      {
        closest_time = std::make_pair(wp.time(), dist);
      }
    }

    if (closest_time.has_value())
    {
      const auto delay = now - closest_time->first;
      context->itinerary().cumulative_delay(plan_id, delay, delay_thresh);
    }

    // If no closest time was found then there are no waypoints in the
    // route. There's no point updating the delay of an empty route.
  }

  const auto& itin = context->itinerary().itinerary();
  for (std::size_t i = 0; i < itin.size(); ++i)
  {
    const auto& traj = itin[i].trajectory();
    const auto t_it = traj.find(now);
    if (t_it != traj.end() && t_it != traj.begin())
    {
      uint64_t checkpoint = 0;
      if (t_it->time() == now)
      {
        checkpoint = t_it->index();
      }
      else
      {
        checkpoint = t_it->index() - 1;
      }

      if (checkpoint < context->itinerary().reached().at(i))
      {
        // The robot has backtracked along its path.
        // This can mess up traffic dependency relationships, so we need to
        // resend the schedule.

        // WARNING: We will have to change this implementation if it is ever
        // possible for the ScheduleOverride class to contain multiple routes in
        // its itinerary.
        const auto cumulative_delay = context->itinerary()
          .cumulative_delay(plan_id).value_or(rmf_traffic::Duration(0));

        plan_id = context->itinerary().assign_plan_id();
        context->itinerary().set(plan_id, {route});
        context->itinerary().cumulative_delay(
          plan_id, cumulative_delay, delay_thresh);
      }

      context->itinerary().reached(plan_id, i, checkpoint);
    }
  }

  if (context->debug_positions)
  {
    std::cout << "Search for location from " << __FILE__ << "|" << __LINE__ <<
      std::endl;
  }
  nav_params->search_for_location(map, location, *context);
}

//==============================================================================
void ScheduleOverride::release_stubbornness()
{
  if (const auto stubborn = w_stubborn.lock())
  {
    // Clear out the previous stubborn handle
    stubborn->stubbornness = nullptr;
  }
}

//==============================================================================
std::optional<ScheduleOverride> ScheduleOverride::make(
  const std::shared_ptr<RobotContext>& context,
  const std::string& map,
  const std::vector<Eigen::Vector3d>& path,
  rmf_traffic::Duration hold,
  std::shared_ptr<StubbornOverride> stubborn)
{
  auto planner = context->planner();
  if (!planner)
  {
    RCLCPP_WARN(
      context->node()->get_logger(),
      "Planner unavailable for robot [%s], cannot override its "
      "schedule",
      context->requester_id().c_str());
    return std::nullopt;
  }

  const auto now = context->now();
  const auto& traits = planner->get_configuration().vehicle_traits();
  auto trajectory = rmf_traffic::agv::Interpolate::positions(
    traits, now, path);
  if (hold > rmf_traffic::Duration(0) && !trajectory.empty())
  {
    const auto& last_wp = trajectory.back();
    trajectory.insert(
      last_wp.time() + hold,
      last_wp.position(),
      Eigen::Vector3d(0.0, 0.0, 0.0));
  }
  std::set<uint64_t> checkpoints;
  for (uint64_t i = 1; i < trajectory.size(); ++i)
  {
    checkpoints.insert(i);
  }
  auto route = rmf_traffic::Route(map, std::move(trajectory));
  route.checkpoints(checkpoints);

  const auto plan_id = context->itinerary().assign_plan_id();
  context->itinerary().set(plan_id, {route});
  stubborn->stubbornness = context->be_stubborn();

  return ScheduleOverride{
    std::move(route),
    plan_id,
    stubborn
  };
}

} // namespace agv
} // namespace rmf_fleet_adapter
