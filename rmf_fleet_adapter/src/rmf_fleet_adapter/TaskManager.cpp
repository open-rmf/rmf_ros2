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

#include "TaskManager.hpp"

#include <rmf_task/requests/ChargeBattery.hpp>
#include <rmf_task/requests/Clean.hpp>
#include <rmf_task/requests/Delivery.hpp>
#include <rmf_task/requests/Loop.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include "tasks/Clean.hpp"
#include "tasks/ChargeBattery.hpp"
#include "tasks/Delivery.hpp"
#include "tasks/Loop.hpp"

#include "phases/ResponsiveWait.hpp"
#include "events/EmergencyPullover.hpp"

#include <rmf_api_msgs/schemas/task_state_update.hpp>
#include <rmf_api_msgs/schemas/task_state.hpp>
#include <rmf_api_msgs/schemas/task_log_update.hpp>
#include <rmf_api_msgs/schemas/task_log.hpp>
#include <rmf_api_msgs/schemas/log_entry.hpp>

namespace rmf_fleet_adapter {

//==============================================================================
TaskManagerPtr TaskManager::make(
  agv::RobotContextPtr context,
  std::weak_ptr<BroadcastClient> broadcast_client)
{
  auto mgr = TaskManagerPtr(
    new TaskManager(
      std::move(context),
      std::move(broadcast_client)));

  auto begin_pullover = [w = mgr->weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_context->worker().schedule(
        [w = self->weak_from_this()](const auto&)
      {
        const auto self = w.lock();

        if (!self->_emergency_active)
          return;

        // TODO(MXG): Consider subscribing to the emergency pullover update
        self->_emergency_pullover = events::EmergencyPullover::Standby::make(
          rmf_task_sequence::Event::AssignID::make(), self->_context, [](){})
            ->begin([](){}, self->_make_resume());
      });
    };

  mgr->_emergency_sub = mgr->_context->node()->emergency_notice()
    .observe_on(rxcpp::identity_same_worker(mgr->_context->worker()))
    .subscribe(
    [w = mgr->weak_from_this(), begin_pullover](const auto& msg)
    {
      if (auto mgr = w.lock())
      {
        if (mgr->_emergency_active == msg->data)
          return;

        mgr->_emergency_active = msg->data;
        if (msg->data)
        {
          if (auto task = mgr->_active_task)
          {
            mgr->_resume_task = task->interrupt(begin_pullover);
          }
          else
          {
            mgr->_resume_task = std::nullopt;
            begin_pullover();
          }
        }
        else
        {
          if (auto pullover = mgr->_emergency_pullover)
            pullover->cancel();
        }
      }
    });

  mgr->_task_timer = mgr->context()->node()->try_create_wall_timer(
    std::chrono::seconds(1),
    [w = mgr->weak_from_this()]()
    {
      if (auto mgr = w.lock())
      {
        mgr->_begin_next_task();
      }
    });

  mgr->_retreat_timer = mgr->context()->node()->try_create_wall_timer(
    std::chrono::seconds(10),
    [w = mgr->weak_from_this()]()
    {
      if (auto mgr = w.lock())
      {
        mgr->retreat_to_charger();
      }
    });

  mgr->_begin_waiting();

  mgr->_travel_estimator = std::make_shared<rmf_task::TravelEstimator>(
    mgr->_context->task_planner()->configuration().parameters());

  mgr->_update_timer = mgr->_context->node()->try_create_wall_timer(
    std::chrono::milliseconds(100),
    [w = mgr->weak_from_this()]()
  {
    if (const auto self = w.lock())
      self->_consider_publishing_updates();
  });

  auto schema = rmf_api_msgs::schemas::task_state;
  nlohmann::json_uri json_uri = nlohmann::json_uri{schema["$id"]};
  mgr->_schema_dictionary.insert({json_uri.url(), schema});
  schema = rmf_api_msgs::schemas::task_log;
  json_uri = nlohmann::json_uri{schema["$id"]};
  mgr->_schema_dictionary.insert({json_uri.url(), schema});
  schema = rmf_api_msgs::schemas::log_entry;
  json_uri = nlohmann::json_uri{schema["$id"]};
  mgr->_schema_dictionary.insert({json_uri.url(), schema});
  schema = rmf_api_msgs::schemas::task_state_update;
  json_uri = nlohmann::json_uri{schema["$id"]};
  mgr->_schema_dictionary.insert({json_uri.url(), schema});
  schema = rmf_api_msgs::schemas::task_log_update;
  json_uri = nlohmann::json_uri{schema["$id"]};
  mgr->_schema_dictionary.insert({json_uri.url(), schema});

  return mgr;
}

//==============================================================================
TaskManager::TaskManager(
  agv::RobotContextPtr context,
  std::weak_ptr<BroadcastClient> broadcast_client)
: _context(std::move(context)),
  _broadcast_client(std::move(broadcast_client)),
  _last_update_time(std::chrono::steady_clock::now() - std::chrono::seconds(1))
{
  // Do nothing. The make() function does all further initialization.
}

//==============================================================================
auto TaskManager::expected_finish_location() const -> StartSet
{
  if (_expected_finish_location)
    return {*_expected_finish_location};

  return _context->location();
}

//==============================================================================
std::optional<std::string> TaskManager::current_task_id() const
{
  if (_active_task)
    return _active_task->tag()->booking()->id();

  return std::nullopt;
}

//==============================================================================
std::string TaskManager::robot_status() const
{
  if (!_active_task)
    return "idle";

  // TODO(MXG): Identify if the robot is charging and report that status here
  return "working";
}

//==============================================================================
auto TaskManager::expected_finish_state() const -> State
{
  // If an active task exists, return the estimated finish state of that task
  /// else update the current time and battery level for the state and return
  if (_active_task)
    return _context->current_task_end_state();

  // Update battery soc and finish time in the current state
  auto finish_state = _context->current_task_end_state();
  auto location = finish_state.extract_plan_start().value();
  finish_state.time(rmf_traffic_ros2::convert(_context->node()->now()));

  const double current_battery_soc = _context->current_battery_soc();
  finish_state.battery_soc(current_battery_soc);

  return finish_state;
}

//==============================================================================
const agv::RobotContextPtr& TaskManager::context()
{
  return _context;
}

//==============================================================================
agv::ConstRobotContextPtr TaskManager::context() const
{
  return _context;
}

//==============================================================================
std::weak_ptr<BroadcastClient> TaskManager::broadcast_client() const
{
  return _broadcast_client;
}

//==============================================================================
void TaskManager::set_queue(
  const std::vector<TaskManager::Assignment>& assignments)
{
  // We indent this block as _mutex is also locked in the _begin_next_task()
  // function that is called at the end of this function.
  {
    std::lock_guard<std::mutex> guard(_mutex);
    _queue = assignments;
    _publish_task_queue();
  }

  _begin_next_task();
}

//==============================================================================
const std::vector<rmf_task::ConstRequestPtr> TaskManager::requests() const
{
  using namespace rmf_task::requests;
  std::vector<rmf_task::ConstRequestPtr> requests;
  requests.reserve(_queue.size());
  for (const auto& task : _queue)
  {
    if (task.request()->booking()->automatic())
    {
      continue;
    }

    requests.push_back(task.request());
  }
  return requests;
}

//==============================================================================
TaskManager::RobotModeMsg TaskManager::robot_mode() const
{
  const auto mode = rmf_fleet_msgs::build<RobotModeMsg>()
    .mode(_active_task == nullptr ?
      RobotModeMsg::MODE_IDLE :
      _context->current_mode())
    .mode_request_id(0);

  return mode;
}

//==============================================================================
std::vector<nlohmann::json> TaskManager::task_log_updates() const
{
  std::vector<nlohmann::json> logs;
  for (const auto& it : _task_logs)
  {
    nlohmann::json update_msg = _task_log_update_msg;
    update_msg["data"] = it.second;
    std::string error = "";
    if (_validate_json(
        update_msg, rmf_api_msgs::schemas::task_log_update, error))
    {
      logs.push_back(update_msg);
    }
    else
    {
      RCLCPP_ERROR(
        _context->node()->get_logger(),
        "%s", error.c_str());
    }
  }
  return logs;
}

//==============================================================================
void TaskManager::_begin_next_task()
{
  if (_active_task)
    return;

  std::lock_guard<std::mutex> guard(_mutex);

  if (_queue.empty())
  {
    if (!_waiting)
      _begin_waiting();

    return;
  }

  if (_waiting)
  {
    _waiting->cancel();
    return;
  }

  const rmf_traffic::Time now = rmf_traffic_ros2::convert(
    _context->node()->now());
  const auto next_task = _queue.front();
  // We take the minimum of the two to deal with cases where the deployment_time
  // as computed by the task planner is greater than the earliest_start_time
  // which is greater than now. This can happen for example if the previous task
  // completed earlier than estimated.
  // TODO: Reactively replan task assignments across agents in a fleet every
  // time as task is completed.
  const auto deployment_time = std::min(
    next_task.deployment_time(),
    next_task.request()->booking()->earliest_start_time());

  if (now >= deployment_time)
  {
    // Update state in RobotContext and Assign active task
    const auto id = _queue.front().request()->booking()->id();
    _context->current_task_end_state(_queue.front().finish_state());
    _context->current_task_id(id);
    const auto assignment = _queue.front();
    _active_task = _context->task_activator()->activate(
      _context->make_get_state(),
      _context->task_parameters(),
      *assignment.request(),
      _update_cb(),
      _checkpoint_cb(),
      _phase_finished_cb(),
      _task_finished(id));

    _queue.erase(_queue.begin());

    if (!_active_task)
    {
      const auto info = assignment.request()->description()->generate_info(
        _context->make_get_state()(), *_context->task_parameters());
      RCLCPP_ERROR(
        _context->node()->get_logger(),
        "[rmf_fleet_adapter::TaskManager::_begin_next_task] Failed to "
        "instantiate task type [%s] for ID [%s]. This indicates that the "
        "fleet adapter is incorrectly configured.",
        info.category.c_str(),
        assignment.request()->booking()->id().c_str());

      _context->worker().schedule(
        [w = weak_from_this()](const auto&)
        {
          if (const auto self = w.lock())
            self->_begin_next_task();
        });

      return;
    }

    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Beginning new task [%s] for [%s]. Remaining queue size: %ld",
      _active_task->tag()->booking()->id().c_str(),
      _context->requester_id().c_str(),
      _queue.size());

    _register_executed_task(_active_task->tag()->booking()->id());
  }
  else
  {
    if (!_waiting)
      _begin_waiting();
  }
}

//==============================================================================
void TaskManager::_begin_waiting()
{
  // Determine the waypoint closest to the robot
  std::size_t waiting_point = _context->location().front().waypoint();
  double min_dist = std::numeric_limits<double>::max();
  const auto& robot_position = _context->position();
  for (const auto& start : _context->location())
  {
    const auto waypoint = start.waypoint();
    const auto& waypoint_location =
      _context->navigation_graph().get_waypoint(waypoint).get_location();
    const auto dist = (robot_position.block<2, 1>(0, 0) -
      waypoint_location).norm();
    if (dist < min_dist)
    {
      min_dist = dist;
      waiting_point = waypoint;
    }
  }

  _waiting = phases::ResponsiveWait::make_indefinite(
    _context, waiting_point)->begin();

  _task_sub = _waiting->observe()
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
    [me = weak_from_this()](LegacyTask::StatusMsg msg)
    {
      const auto self = me.lock();
      if (!self)
        return;

      self->_populate_task_summary(nullptr, msg.state, msg);
      self->_context->node()->task_summary()->publish(msg);
    },
    [me = weak_from_this()](std::exception_ptr e)
    {
      const auto self = me.lock();
      if (!self)
        return;

      rmf_task_msgs::msg::TaskSummary msg;

      try
      {
        std::rethrow_exception(e);
      }
      catch (const std::exception& e)
      {
        msg.status = e.what();
      }

      self->_populate_task_summary(nullptr, msg.STATE_FAILED, msg);
      self->_context->node()->task_summary()->publish(msg);

      RCLCPP_WARN(
        self->_context->node()->get_logger(),
        "Robot [%s] encountered an error while doing a ResponsiveWait: %s",
        self->_context->requester_id().c_str(), msg.status.c_str());

      // Go back to waiting if an error has occurred
      self->_begin_waiting();
    },
    [me = weak_from_this()]()
    {
      const auto self = me.lock();
      if (!self)
        return;

      self->_waiting = nullptr;
      self->_begin_next_task();
    });
}

//==============================================================================
std::function<void()> TaskManager::_make_resume()
{
  return [w = weak_from_this()]()
  {
    const auto self = w.lock();
    if (!self)
      return;

    self->_resume();
  };
}

//==============================================================================
void TaskManager::_resume()
{
  _context->worker().schedule(
    [w = weak_from_this()](const auto&)
    {
      const auto self = w.lock();
      if (!self)
        return;

      if (self->_emergency_active)
        return;

      self->_emergency_pullover = nullptr;
      if (self->_resume_task.has_value())
      {
        auto resume = *std::move(self->_resume_task);
        self->_resume_task = std::nullopt;
        resume();
      }
      else
      {
        self->_begin_next_task();
      }
    });
}

//==============================================================================
void TaskManager::retreat_to_charger()
{
  {
    std::lock_guard<std::mutex> guard(_mutex);
    if (_active_task || !_queue.empty())
      return;
  }

  const auto task_planner = _context->task_planner();
  if (!task_planner)
    return;

  if (!task_planner->configuration().constraints().drain_battery())
    return;

  const auto current_state = expected_finish_state();
  const auto charging_waypoint =
    current_state.dedicated_charging_waypoint().value();
  if (current_state.waypoint() == charging_waypoint)
    return;

  const auto& constraints = task_planner->configuration().constraints();
  const double threshold_soc = constraints.threshold_soc();
  const double retreat_threshold = 1.2 * threshold_soc; // safety factor
  const double current_battery_soc = _context->current_battery_soc();

  const auto& parameters = task_planner->configuration().parameters();
  // TODO(YV): Expose the TravelEstimator in the TaskPlanner to benefit from
  // caching
  const rmf_traffic::agv::Planner::Goal retreat_goal{charging_waypoint};
  const auto result = _travel_estimator->estimate(
    current_state.extract_plan_start().value(), retreat_goal);
  if (!result.has_value())
  {
    RCLCPP_WARN(
      _context->node()->get_logger(),
      "Unable to compute estimate of journey back to charger for robot [%s]",
      _context->name().c_str());
    return;
  }

  const double battery_soc_after_retreat =
    current_battery_soc - result->change_in_charge();

  if ((battery_soc_after_retreat < retreat_threshold) &&
    (battery_soc_after_retreat > threshold_soc))
  {
    // Add a new charging task to the task queue
    const auto charging_request = rmf_task::requests::ChargeBattery::make(
      current_state.time().value());
    const auto model = charging_request->description()->make_model(
      current_state.time().value(),
      parameters);

    const auto finish = model->estimate_finish(
      current_state,
      constraints,
      *_travel_estimator);

    if (!finish)
      return;

    rmf_task::TaskPlanner::Assignment charging_assignment(
      charging_request,
      finish.value().finish_state(),
      current_state.time().value());

    set_queue({charging_assignment});

    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Initiating automatic retreat to charger for robot [%s]",
      _context->name().c_str());
  }

  if ((battery_soc_after_retreat < retreat_threshold) &&
    (battery_soc_after_retreat < threshold_soc))
  {
    RCLCPP_WARN(
      _context->node()->get_logger(),
      "Robot [%s] needs to be charged but has insufficient battery remaining "
      "to retreat to its designated charger.",
      _context->name().c_str());
  }
}

//==============================================================================
const std::vector<std::string>& TaskManager::get_executed_tasks() const
{
  return _executed_task_registry;
}

//==============================================================================
void TaskManager::_register_executed_task(const std::string& id)
{
  // Currently the choice of storing 100 executed tasks is arbitrary.
  // TODO: Save a time stamp for when tasks are completed and cull entries after
  // a certain time window instead.
  if (_executed_task_registry.size() >= 100)
    _executed_task_registry.erase(_executed_task_registry.begin());

  _executed_task_registry.push_back(id);
}

//==============================================================================
void TaskManager::_populate_task_summary(
  std::shared_ptr<LegacyTask> task,
  uint32_t task_summary_state,
  TaskManager::TaskSummaryMsg& msg)
{
  if (task == nullptr) // ResponsiveWait
  {
    msg.task_id = _context->requester_id() + ":waiting";

    msg.start_time = _context->node()->now();
    msg.end_time = _queue.empty() ? msg.start_time : rmf_traffic_ros2::convert(
      _queue.front().deployment_time());
    // Make the task type explicit
    msg.task_profile.description.task_type.type =
      rmf_task_msgs::msg::TaskType::TYPE_STATION;
  }

  else
  {
    msg.task_id = task->id();
    msg.start_time = rmf_traffic_ros2::convert(
      task->deployment_time());
    msg.end_time = rmf_traffic_ros2::convert(
      task->finish_state().time().value());
    msg.task_profile = task->task_profile();
  }

  msg.fleet_name = _context->description().owner();
  msg.robot_name = _context->name();

  msg.state = task_summary_state;
}

//==============================================================================
void TaskManager::_schema_loader(
  const nlohmann::json_uri& id, nlohmann::json& value) const
{
  const auto it = _schema_dictionary.find(id.url());
  if (it == _schema_dictionary.end())
  {
    RCLCPP_ERROR(
      _context->node()->get_logger(),
      "[TaskManager] url: %s not found in schema dictionary", id.url().c_str());
    return;
  }

  value = it->second;
}

//==============================================================================
void TaskManager::_validate_and_publish_json(
  const nlohmann::json& msg,
  const nlohmann::json& schema) const
{
  std::string error = "";
  if (!_validate_json(msg, schema, error))
  {
    RCLCPP_ERROR(
      _context->node()->get_logger(),
      "[%s]",
      error.c_str());
    return;
  }

  const auto client = _broadcast_client.lock();
  if (!client)
  {
    RCLCPP_ERROR(
      _context->node()->get_logger(),
      "Unable to lock BroadcastClient within TaskManager of robot [%s]",
      _context->name().c_str());
    return;
  }
  client->publish(msg);
}


//==============================================================================
rmf_task::State TaskManager::_get_state() const
{
  return _context->current_task_end_state();
}

//==============================================================================
void TaskManager::_consider_publishing_updates()
{
  const auto now = std::chrono::steady_clock::now();
  const auto time_elapsed = now - _last_update_time;
  // TODO(MXG): Make max elapsed time configurable
  const auto max_time_elapsed = std::chrono::seconds(1);
  if (_task_state_update_available || time_elapsed > max_time_elapsed)
  {
    _task_state_update_available = false;
    _last_update_time = now;
    _publish_task_state();
  }
}

namespace {
//==============================================================================
std::chrono::milliseconds to_millis(rmf_traffic::Duration duration)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
}

//==============================================================================
std::string status_to_string(rmf_task::Event::Status status)
{
  using Status = rmf_task::Event::Status;
  switch (status)
  {
  case Status::Uninitialized:
    return "uninitialized";
  case Status::Blocked:
    return "blocked";
  case Status::Error:
    return "error";
  case Status::Failed:
    return "failed";
  case Status::Standby:
    return "standby";
  case Status::Underway:
    return "underway";
  case Status::Delayed:
    return "delayed";
  case Status::Skipped:
    return "skipped";
  case Status::Canceled:
    return "canceled";
  case Status::Killed:
    return "killed";
  case Status::Completed:
    return "completed";
  default:
    return "uninitialized";
  }
}

//==============================================================================
std::string tier_to_string(rmf_task::Log::Entry::Tier tier)
{
  using Tier = rmf_task::Log::Entry::Tier;
  switch (tier)
  {
  case Tier::Info:
    return "info";
  case Tier::Warning:
    return "warning";
  case Tier::Error:
    return "error";
  default:
    return "uninitialized";
  }
}

//==============================================================================
nlohmann::json log_to_json(const rmf_task::Log::Entry& entry)
{
  nlohmann::json output;
  output["seq"] = entry.seq();
  output["tier"] = tier_to_string(entry.tier());
  output["unix_millis_time"] =
    to_millis(entry.time().time_since_epoch()).count();
  output["text"] = entry.text();

  return output;
}

//==============================================================================
nlohmann::json& copy_phase_data(
  nlohmann::json& phases,
  const rmf_task::Phase::Active& snapshot,
  rmf_task::Log::Reader& reader,
  nlohmann::json& all_phase_logs)
{
  const auto& tag = *snapshot.tag();
  const auto& header = tag.header();
  const auto id = tag.id();
  auto& phase = phases[std::to_string(id)];
  phase["id"] = id;
  phase["category"] = header.category();
  phase["detail"] = header.detail();
  phase["original_estimate_millis"] =
    std::max(0l, to_millis(header.original_duration_estimate()).count());
  phase["estimate_millis"] =
    std::max(0l, to_millis(snapshot.estimate_remaining_time()).count());
  phase["final_event_id"] = snapshot.final_event()->id();

  // TODO(MXG): Add in skip request information

  std::vector<rmf_task::Event::ConstStatePtr> event_queue;
  event_queue.push_back(snapshot.final_event());

  auto& phase_logs = all_phase_logs[std::to_string(id)];
  auto& event_logs = phase_logs["events"];

  while (!event_queue.empty())
  {
    const auto top = event_queue.back();
    event_queue.pop_back();

    nlohmann::json event_state;
    event_state["id"] = top->id();
    event_state["status"] = status_to_string(top->status());

    // TODO(MXG): Keep a VersionedString Reader to know when to actually update
    // this string
    event_state["name"] =
      *rmf_task::VersionedString::Reader().read(top->name());

    event_state["detail"] =
      *rmf_task::VersionedString::Reader().read(top->detail());

    std::vector<nlohmann::json> logs;
    for (const auto& log : reader.read(top->log()))
      logs.push_back(log_to_json(log));

    if (!logs.empty())
      event_logs[std::to_string(top->id())] = std::move(logs);

    std::vector<uint32_t> deps;
    deps.reserve(top->dependencies().size());
    for (const auto& dep : top->dependencies())
    {
      event_queue.push_back(dep);
      deps.push_back(dep->id());
    }

    event_state["deps"] = std::move(deps);
  }

  return phase;
}

//==============================================================================
void copy_phase_data(
  nlohmann::json& phases,
  const rmf_task::Phase::Pending& pending)
{
  const auto id = pending.tag()->id();
  auto& phase = phases[std::to_string(id)];
  phase["id"] = id;

  const auto& header = pending.tag()->header();
  phase["category"] = header.category();
  phase["detail"] = header.detail();
  phase["estimate_millis"] =
    std::max(0l, to_millis(header.original_duration_estimate()).count());
}

//==============================================================================
void copy_booking_data(
  nlohmann::json& booking_json,
  const rmf_task::Task::Booking& booking)
{
  booking_json["id"] = booking.id();
  booking_json["unix_millis_earliest_start_time"] =
    to_millis(booking.earliest_start_time().time_since_epoch()).count();
  // TODO(MXG): Add priority and labels
}

//==============================================================================
void copy_assignment(
  nlohmann::json& assigned_to_json,
  const agv::RobotContext& context)
{
  assigned_to_json["group"] = context.group();
  assigned_to_json["name"] = context.name();
}

} // anonymous namespace

//==============================================================================
void TaskManager::_publish_task_state()
{
  if (!_active_task)
    return;

  auto task_state_update = _task_state_update_json;

  const auto& booking = *_active_task->tag()->booking();
  copy_booking_data(_active_task_state["booking"], booking);

  const auto& header = _active_task->tag()->header();
  _active_task_state["category"] = header.category();
  _active_task_state["detail"] = header.detail();
  // TODO(MXG): Add unix_millis_start_time and unix_millis_finish_time
  _active_task_state["original_estimate_millis"] =
    std::max(0l, to_millis(header.original_duration_estimate()).count());
  _active_task_state["estimate_millis"] =
    std::max(0l, to_millis(_active_task->estimate_remaining_time()).count());
  copy_assignment(_active_task_state["assigned_to"], *_context);
  _active_task_state["status"] =
    status_to_string(_active_task->status_overview());

  auto& phases = _active_task_state["phases"];

  nlohmann::json task_logs;
  auto& phase_logs = task_logs["phases"];

  std::vector<uint64_t> completed_ids;
  completed_ids.reserve(_active_task->completed_phases().size());
  for (const auto& completed : _active_task->completed_phases())
  {
    const auto& snapshot = completed->snapshot();
    auto& phase = copy_phase_data(phases, *snapshot, _log_reader, phase_logs);

    phase["unix_millis_start_time"] =
      completed->start_time().time_since_epoch().count();

    phase["unix_millis_finish_time"] =
      completed->finish_time().time_since_epoch().count();

    completed_ids.push_back(snapshot->tag()->id());
  }
  _active_task_state["completed"] = std::move(completed_ids);

  const auto active_phase = _active_task->active_phase();
  copy_phase_data(phases, *active_phase, _log_reader, phase_logs);

  _active_task_state["active"] = active_phase->tag()->id();

  std::vector<uint64_t> pending_ids;
  pending_ids.reserve(_active_task->pending_phases().size());
  for (const auto& pending : _active_task->pending_phases())
  {
    copy_phase_data(phases, pending);
    pending_ids.push_back(pending.tag()->id());
  }
  _active_task_state["pending"] = std::move(pending_ids);

  task_state_update["data"] = _active_task_state;
  _validate_and_publish_json(
    task_state_update, rmf_api_msgs::schemas::task_state_update);
}

//==============================================================================
void TaskManager::_publish_task_queue()
{
  rmf_task::State expected_state = _context->current_task_end_state();
  const auto& parameters = *_context->task_parameters();
  for (const auto& pending : _queue)
  {
    const auto info = pending.request()->description()->generate_info(
      expected_state, parameters);

    nlohmann::json pending_json;
    const auto& booking = *pending.request()->booking();
    copy_booking_data(pending_json["booking"], booking);

    pending_json["category"] = info.category;
    pending_json["detail"] = info.detail;

    const auto estimate =
      pending.finish_state().time().value() - pending.deployment_time();
    pending_json["original_estimate_millis"] =
      std::max(0l, to_millis(estimate).count());
    copy_assignment(pending_json["assigned_to"], *_context);
    pending_json["status"] = "standby";

    auto task_state_update = _task_state_update_json;
    task_state_update["data"] = pending_json;

    _validate_and_publish_json(
      task_state_update, rmf_api_msgs::schemas::task_state_update);

    expected_state = pending.finish_state();
  }
}

//==============================================================================
bool TaskManager::_validate_json(
  const nlohmann::json& json,
  const nlohmann::json& schema,
  std::string& error) const
{
  try
  {
    nlohmann::json_schema::json_validator validator(
      schema,
      [w = weak_from_this()](const nlohmann::json_uri& id,
      nlohmann::json& value)
      {
        const auto self = w.lock();
        if (!self)
          return;
        self->_schema_loader(id, value);
      });
    validator.validate(json);
  }
  catch (const std::exception& e)
  {
    error = e.what();
    return false;
  }

  return true;
}

//==============================================================================
std::function<void(rmf_task::Phase::ConstSnapshotPtr)>
TaskManager::_update_cb()
{
  return [w = weak_from_this()](rmf_task::Phase::ConstSnapshotPtr)
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_task_state_update_available = true;
      // TODO(MXG): Use this callback to make the state updates more efficient
    };
}

//==============================================================================
std::function<void(rmf_task::Task::Active::Backup)>
TaskManager::_checkpoint_cb()
{
  return [w = weak_from_this()](rmf_task::Task::Active::Backup backup)
  {
    const auto self = w.lock();
    if (!self)
      return;

    // TODO(MXG): Save the backup
  };
}

//==============================================================================
std::function<void(rmf_task::Phase::ConstCompletedPtr)>
TaskManager::_phase_finished_cb()
{
  return [w = weak_from_this()](rmf_task::Phase::ConstCompletedPtr)
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_task_state_update_available = true;
      // TODO(MXG): Use this callback to make the state updates more efficient
    };
}

//==============================================================================
std::function<void()> TaskManager::_task_finished(std::string id)
{
  return [w = weak_from_this(), id]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      // Publish the final state of the task before destructing it
      self->_publish_task_state();
      self->_active_task = nullptr;

      self->_context->worker().schedule(
        [w = self->weak_from_this()](const auto&)
        {
          if (const auto self = w.lock())
            self->_begin_next_task();
        });
    };
}

} // namespace rmf_fleet_adapter
