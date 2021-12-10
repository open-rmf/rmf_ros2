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
  auto mgr = TaskManagerPtr(new TaskManager(
        std::move(context), std::move(broadcast_client)));
  mgr->_emergency_sub = mgr->_context->node()->emergency_notice()
    .observe_on(rxcpp::identity_same_worker(mgr->_context->worker()))
    .subscribe(
    [w = mgr->weak_from_this()](const auto& msg)
    {
      if (auto mgr = w.lock())
      {
        if (auto task = mgr->_active_task)
        {
          if (auto phase = task->current_phase())
            phase->emergency_alarm(msg->data);
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

  mgr->_activator = std::make_shared<rmf_task::Activator>();

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
  _broadcast_client(std::move(broadcast_client))
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
const LegacyTask* TaskManager::current_task() const
{
  return _active_task.get();
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
  const std::vector<TaskManager::Assignment>& assignments,
  const TaskManager::TaskProfiles& task_profiles)
{
  // We indent this block as _mutex is also locked in the _begin_next_task()
  // function that is called at the end of this function.
  {
    std::lock_guard<std::mutex> guard(_mutex);
    _queue.clear();

    // We use dynamic cast to determine the type of request and then call the
    // appropriate make(~) function to convert the request into a task
    for (std::size_t i = 0; i < assignments.size(); ++i)
    {
      const auto& a = assignments[i];
      auto start =
        _context->current_task_end_state().extract_plan_start().value();
      if (i != 0)
        start = assignments[i-1].finish_state().extract_plan_start().value();
      start.time(a.deployment_time());
      const auto request = a.request();

      TaskProfileMsg task_profile;
      bool auto_request = request->booking()->automatic();
      const auto it = task_profiles.find(request->booking()->id());
      if (it != task_profiles.end())
      {
        task_profile = it->second;
      }
      else
      {
        assert(auto_request);
        // We may have an auto-generated request
        task_profile.task_id = request->booking()->id();
        task_profile.submission_time = _context->node()->now();
        task_profile.description.start_time = rmf_traffic_ros2::convert(
          request->booking()->earliest_start_time());
      }

      using namespace rmf_task::requests;

      if (std::dynamic_pointer_cast<const Clean::Description>(
          request->description()) != nullptr)
      {
        auto task = rmf_fleet_adapter::tasks::make_clean(
          request,
          _context,
          start,
          a.deployment_time(),
          a.finish_state());

        // Populate task_profile for auto-generated Clean request
        if (auto_request)
        {
          std::shared_ptr<const rmf_task::requests::Clean::Description>
          description = std::dynamic_pointer_cast<
            const Clean::Description>(request->description());
          const auto start_waypoint = description->start_waypoint();
          const auto waypoint_name =
            _context->navigation_graph().get_waypoint(start_waypoint).name();
          task_profile.description.task_type.type =
            rmf_task_msgs::msg::TaskType::TYPE_CLEAN;
          task_profile.description.clean.start_waypoint =
            waypoint_name != nullptr ? *waypoint_name : "";
        }
        task->task_profile(task_profile);

        _queue.push_back(task);
      }

      else if (std::dynamic_pointer_cast<const ChargeBattery::Description>(
          request->description()) != nullptr)
      {
        const auto task = tasks::make_charge_battery(
          request,
          _context,
          start,
          a.deployment_time(),
          a.finish_state());

        // Populate task_profile for auto-generated ChargeBattery request
        if (auto_request)
        {
          task_profile.description.task_type.type =
            rmf_task_msgs::msg::TaskType::TYPE_CHARGE_BATTERY;
        }
        task->task_profile(task_profile);

        _queue.push_back(task);
      }

      else if (std::dynamic_pointer_cast<const Delivery::Description>(
          request->description()) != nullptr)
      {
        const auto task = tasks::make_delivery(
          request,
          _context,
          start,
          a.deployment_time(),
          a.finish_state(),
          task_profile.description.delivery);

        // Populate task_profile for auto-generated Delivery request
        if (auto_request)
        {
          std::shared_ptr<const rmf_task::requests::Delivery::Description>
          description = std::dynamic_pointer_cast<
            const Delivery::Description>(request->description());
          const auto& graph = _context->navigation_graph();
          const auto pickup_waypoint = description->pickup_waypoint();
          const auto pickup_name =
            graph.get_waypoint(pickup_waypoint).name();
          const auto dropoff_waypoint = description->dropoff_waypoint();
          const auto dropoff_name =
            graph.get_waypoint(dropoff_waypoint).name();
          task_profile.description.task_type.type =
            rmf_task_msgs::msg::TaskType::TYPE_DELIVERY;
          task_profile.description.delivery.pickup_place_name =
            pickup_name != nullptr ? *pickup_name : "";
          task_profile.description.delivery.dropoff_place_name =
            dropoff_name != nullptr ? *dropoff_name : "";
        }
        task->task_profile(task_profile);

        _queue.push_back(task);
      }

      else if (std::dynamic_pointer_cast<const Loop::Description>(request->
        description()) != nullptr)
      {
        const auto task = tasks::make_loop(
          request,
          _context,
          start,
          a.deployment_time(),
          a.finish_state());

        // Populate task_profile for auto-generated Loop request
        if (auto_request)
        {
          std::shared_ptr<const rmf_task::requests::Loop::Description>
          description = std::dynamic_pointer_cast<
            const Loop::Description>(request->description());
          const auto& graph = _context->navigation_graph();
          const auto start_waypoint = description->start_waypoint();
          const auto start_name =
            graph.get_waypoint(start_waypoint).name();
          const auto finish_waypoint = description->finish_waypoint();
          const auto finish_name =
            graph.get_waypoint(finish_waypoint).name();
          task_profile.description.loop.num_loops = description->num_loops();
          task_profile.description.task_type.type =
            rmf_task_msgs::msg::TaskType::TYPE_LOOP;
          task_profile.description.loop.start_name =
            start_name != nullptr ? *start_name : "";
          task_profile.description.loop.finish_name =
            finish_name != nullptr ? *finish_name : "";
        }
        task->task_profile(task_profile);

        _queue.push_back(task);
      }

      else
      {
        RCLCPP_WARN(
          _context->node()->get_logger(),
          "[TaskManager] Un-supported request type in assignment list. "
          "Please update the implementation of TaskManager::set_queue() to "
          "support request with task_id:[%s]",
          a.request()->booking()->id().c_str());

        continue;
      }

      // publish queued task
      TaskSummaryMsg msg;
      _populate_task_summary(_queue.back(), msg.STATE_QUEUED, msg);
      _context->node()->task_summary()->publish(msg);
    }
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
    if (task->request()->booking()->automatic())
    {
      continue;
    }

    requests.push_back(task->request());
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
    next_task->deployment_time(),
    next_task->request()->booking()->earliest_start_time());

  if (now >= deployment_time)
  {
    // Update state in RobotContext and Assign active task
    _context->current_task_end_state(_queue.front()->finish_state());
    _active_task = std::move(_queue.front());
    _queue.erase(_queue.begin());

    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Beginning new task [%s] for [%s]. Remaining queue size: %ld",
      _active_task->id().c_str(),
      _context->requester_id().c_str(),
      _queue.size());

    _task_sub = _active_task->observe()
      .observe_on(rxcpp::identity_same_worker(_context->worker()))
      .subscribe(
      [me = weak_from_this(), active_task = _active_task](LegacyTask::StatusMsg msg)
      {
        const auto self = me.lock();
        if (!self)
          return;

        self->_populate_task_summary(active_task, msg.state, msg);
        self->_context->node()->task_summary()->publish(msg);
      },
      [me = weak_from_this(), active_task = _active_task](std::exception_ptr e)
      {
        const auto self = me.lock();
        if (!self)
          return;

        TaskSummaryMsg msg;

        try
        {
          std::rethrow_exception(e);
        }
        catch (const std::exception& e)
        {
          msg.status = e.what();
        }

        self->_populate_task_summary(active_task, msg.STATE_FAILED, msg);
        self->_context->node()->task_summary()->publish(msg);
      },
      [me = weak_from_this(), active_task = _active_task]()
      {
        const auto self = me.lock();
        if (!self)
          return;

        TaskSummaryMsg msg;
        self->_populate_task_summary(
          active_task, msg.STATE_COMPLETED, msg);
        self->_context->node()->task_summary()->publish(msg);

        self->_active_task = nullptr;
      });

    _active_task->begin();
    _register_executed_task(_active_task->id());
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
      _queue.front()->deployment_time());
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
void TaskManager::_update(rmf_task::Phase::ConstSnapshotPtr snapshot)
{
  // TODO

  auto task_state_update = _task_state_update_json;
  task_state_update["data"] = _active_task_state;
  _validate_and_publish_json(
    task_state_update, rmf_api_msgs::schemas::task_state_update);
}

//==============================================================================
void TaskManager::_checkpoint(rmf_task::Task::Active::Backup backup)
{
  // TODO

  auto task_state_update = _task_state_update_json;
  task_state_update["data"] = _active_task_state;
  _validate_and_publish_json(
    task_state_update, rmf_api_msgs::schemas::task_state_update);
}

//==============================================================================
void TaskManager::_phase_finished(
  rmf_task::Phase::ConstCompletedPtr completed_phase)
{
  // TODO

  auto task_state_update = _task_state_update_json;
  task_state_update["data"] = _active_task_state;
  _validate_and_publish_json(
    task_state_update, rmf_api_msgs::schemas::task_state_update);
}

//==============================================================================
void TaskManager::_task_finished()
{
  // TODO

  auto task_state_update = _task_state_update_json;
  task_state_update["data"] = _active_task_state;
  _validate_and_publish_json(
    task_state_update, rmf_api_msgs::schemas::task_state_update);

  auto task_log_update = _task_log_update_msg;
  task_log_update["data"] = _task_logs[_active_task_state["booking"]["id"]];
  _validate_and_publish_json(
    task_log_update, rmf_api_msgs::schemas::task_log_update);

  _active_task_state = {};
}

} // namespace rmf_fleet_adapter
