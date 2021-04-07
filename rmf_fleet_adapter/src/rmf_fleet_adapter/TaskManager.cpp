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

namespace rmf_fleet_adapter {

//==============================================================================
void populate_task_summary(
  rmf_task_msgs::msg::TaskSummary& msg,
  const agv::RobotContextPtr context,
  const std::shared_ptr<Task> task = nullptr)
{
  msg.robot_name = context->name();
  msg.fleet_name = context->description().owner();
  if (task != nullptr)
  {
    msg.task_id = task->id(); // duplicated
    msg.task_profile = task->profile_msg();
    msg.start_time = rmf_traffic_ros2::convert(
      task->deployment_time());
    msg.end_time = rmf_traffic_ros2::convert(
      task->finish_state().finish_time());
  }
}

//==============================================================================
TaskManagerPtr TaskManager::make(agv::RobotContextPtr context)
{
  auto mgr = TaskManagerPtr(new TaskManager(std::move(context)));
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

  mgr->_task_timer = mgr->context()->node()->create_wall_timer(
    std::chrono::seconds(1),
    [w = mgr->weak_from_this()]()
    {
      if (auto mgr = w.lock())
      {
        mgr->_begin_next_task();
      }
    });

  mgr->_retreat_timer = mgr->context()->node()->create_wall_timer(
    std::chrono::seconds(10),
    [w = mgr->weak_from_this()]()
    {
      if (auto mgr = w.lock())
      {
        mgr->retreat_to_charger();
      }
    });
  return mgr;
}

//==============================================================================
TaskManager::TaskManager(agv::RobotContextPtr context)
  : _context(std::move(context))
{
  _begin_waiting();
}

//==============================================================================
void TaskManager::queue_task(std::shared_ptr<Task> task)
{
  {
    std::lock_guard<std::mutex> guard(_mutex);
    _queue.push_back(std::move(task));

  RCLCPP_INFO(
        _context->node()->get_logger(),
        "Queuing new task [%s] for [%s]. New queue size: %d",
         _queue.back()->id().c_str(), _context->requester_id().c_str(),
        _queue.size());

    rmf_task_msgs::msg::TaskSummary msg;
    populate_task_summary(msg, _context, _queue.back());
    msg.state = msg.STATE_QUEUED;
    this->_context->node()->task_summary()->publish(msg);
  }

  if (!_active_task)
    _begin_next_task();
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
  auto location = finish_state.location();
  location.time(rmf_traffic_ros2::convert(_context->node()->now()));
  finish_state.location(location);
  
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
const Task* TaskManager::current_task() const
{
  return _active_task.get();
}

//==============================================================================
agv::ConstRobotContextPtr TaskManager::context() const
{
  return _context;
}

//==============================================================================
const std::vector<std::shared_ptr<Task>> TaskManager::task_queue() const
{
  return _queue;
}

//==============================================================================
void TaskManager::set_queue(
  const std::vector<std::shared_ptr<Task>>& tasks)
{
  // We indent this block as _mutex is also locked in the _begin_next_task()
  // function that is called at the end of this function.
  {
    std::lock_guard<std::mutex> guard(_mutex);
    _queue.clear();
  }

  for (const auto t : tasks)
    queue_task(t);
}

//==============================================================================
void TaskManager::set_queue(
  const std::vector<TaskManager::Assignment>& assignments,
  const std::unordered_map<std::string, TaskProfile>& task_profiles)
{
  std::vector<std::shared_ptr<Task>> task_queue;
  std::shared_ptr<Task> task = nullptr;
  for (std::size_t i = 0; i < assignments.size(); ++i)
  {
    const auto& a = assignments[i];
    auto start = _context->current_task_end_state().location();
    if (i != 0)
      start = assignments[i-1].state().location();
    start.time(a.deployment_time());

    const auto req = a.request();
    const auto id = req->id();
    const auto req_desc = req->description();

    rmf_task_msgs::msg::TaskProfile profile;
    if (task_profiles.find(id) != task_profiles.end())
    {
      profile = task_profiles.at(id);
    }
    else
    {
      // This is likely a self-generated task
      profile.task_id = id;
      profile.submission_time = _context->node()->now();
      profile.description.start_time =
        rmf_traffic_ros2::convert(a.deployment_time());
    }

    // We use dynamic cast to determine the type of request and then call the
    // appropriate make(~) function to convert the request into a task
    using namespace rmf_task::requests;
    using TaskType = rmf_task_msgs::msg::TaskType;

    /// CHARGE BATTERY TASK (self-Generated)
    if (std::dynamic_pointer_cast<const ChargeBatteryDescription>(req_desc))
    {
      task = tasks::make_charge_battery(
        req, _context, start, a.deployment_time(), a.state());
      profile.description.task_type.type = TaskType::TYPE_CHARGE_BATTERY;
    }
    /// CLEAN TASK (User Request)
    else if (std::dynamic_pointer_cast<const CleanDescription>(req_desc))
    {
      task = tasks::make_clean(
        req, _context, start, a.deployment_time(), a.state());
    }
    /// DELIVERY TASK (User Request)
    else if (std::dynamic_pointer_cast<const DeliveryDescription>(req_desc))
    {
      task = tasks::make_delivery(
        req, _context, start, a.deployment_time(), a.state());
    }
    /// LOOP TASK (User Request)
    else if (std::dynamic_pointer_cast<const LoopDescription>(req_desc))
    {
      task = tasks::make_loop(
        req, _context, start, a.deployment_time(), a.state());
    }
    else
    {
      RCLCPP_WARN(
        _context->node()->get_logger(),
        "[TaskManager] Un-supported request type in assignment list. "
        "Please update the implementation of TaskManager::set_queue() to "
        "support request with task_id:[%s]", id.c_str());
      continue;
    }

    task->profile_msg(profile);
    task_queue.push_back(task);
  }
  this->set_queue(task_queue);
}

//==============================================================================
const std::vector<rmf_task::ConstRequestPtr> TaskManager::requests() const
{
  std::vector<rmf_task::ConstRequestPtr> requests;
  requests.reserve(_queue.size());
  for (const auto& task : _queue)
  {
    if (std::dynamic_pointer_cast<
      const rmf_task::requests::ChargeBatteryDescription>(
        task->request()->description()))
      continue;
    requests.push_back(task->request());
  }
  return requests;
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
  const auto deployment_time = next_task->deployment_time();

  if (now >= deployment_time)
  {
    // Update state in RobotContext and Assign active task
    _context->current_task_end_state(_queue.front()->finish_state());
    _active_task = std::move(_queue.front());
    _queue.erase(_queue.begin());

    RCLCPP_INFO(
          _context->node()->get_logger(),
          "Beginning new task [%s] for [%s]. Remaining queue size: %d",
          _active_task->id().c_str(), _context->requester_id().c_str(),
          _queue.size());

    _task_sub = _active_task->observe()
        .observe_on(rxcpp::identity_same_worker(_context->worker()))
        .subscribe(
          [this, id = _active_task->id()](Task::StatusMsg msg)
    {
      populate_task_summary(msg, _context, _active_task);
      _context->node()->task_summary()->publish(msg);
    },
          [this, id = _active_task->id()](std::exception_ptr e)
    {
      rmf_task_msgs::msg::TaskSummary msg;
      populate_task_summary(msg, _context, _active_task);
      msg.state = msg.STATE_FAILED;

      try {
        std::rethrow_exception(e);
      }
      catch(const std::exception& e) {
        msg.status = e.what();
      }

      _context->node()->task_summary()->publish(msg);
    },
          [this, id = _active_task->id()]()
    {
      rmf_task_msgs::msg::TaskSummary msg;
      populate_task_summary(msg, _context, _active_task);
      msg.state = msg.STATE_COMPLETED;
       this->_context->node()->task_summary()->publish(msg);
      _active_task = nullptr;
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
  const std::size_t waiting_point = _context->location().front().waypoint();

  _waiting = phases::ResponsiveWait::make_indefinite(
        _context, waiting_point)->begin();

  _task_sub = _waiting->observe()
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
      [this](Task::StatusMsg msg)
      {
        populate_task_summary(msg, _context);
        msg.task_id = _context->requester_id() + ":waiting";
        // TODO: Fill in end_time with the beginning time of the next task if
        // the next task is known.
        msg.start_time = _context->node()->now();
        msg.end_time = msg.start_time;

        _context->node()->task_summary()->publish(msg);
      },
      [this](std::exception_ptr e)
      {
        rmf_task_msgs::msg::TaskSummary msg;
        populate_task_summary(msg, _context);
        msg.state = msg.STATE_FAILED;

        try {
          std::rethrow_exception(e);
        }
        catch (const std::exception& e) {
          msg.status = e.what();
        }

        msg.task_id = _context->requester_id() + ":waiting";
        _context->node()->task_summary()->publish(msg);
        // TODO: empty id string?

        RCLCPP_WARN(
          _context->node()->get_logger(),
          "Robot [%s] encountered an error while doing a ResponsiveWait: %s",
          _context->requester_id().c_str(), msg.status.c_str());

        // Go back to waiting if an error has occurred
        _begin_waiting();
      },
      [this]()
      {
        _waiting = nullptr;
        _begin_next_task();
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

  const auto current_state = expected_finish_state();
  if (current_state.waypoint() == current_state.charging_waypoint())
    return;

  const double threshold_soc =
    _context->task_planning_constraints().threshold_soc();
  const double retreat_threshold = 1.2 * threshold_soc;
  const double current_battery_soc = _context->current_battery_soc();

  const auto& task_planner_config = task_planner->config();
  const auto estimate_cache = task_planner->estimate_cache();

  double retreat_battery_drain = 0.0;
  const auto endpoints = std::make_pair(current_state.waypoint(),
    current_state.charging_waypoint());
  const auto& cache_result = estimate_cache->get(endpoints);

  if (cache_result)
  {
    retreat_battery_drain = cache_result->dsoc;
  }
  else
  {
    const rmf_traffic::agv::Planner::Goal retreat_goal{
      current_state.charging_waypoint()};
    const auto result_to_charger = task_planner_config.planner()->plan(
      current_state.location(), retreat_goal);

    // We assume we can always compute a plan
    double dSOC_motion = 0.0;
    double dSOC_device = 0.0;
    rmf_traffic::Duration retreat_duration = rmf_traffic::Duration{0};
    rmf_traffic::Time itinerary_start_time = current_state.finish_time();

    for (const auto& itinerary : result_to_charger->get_itinerary())
    {
      const auto& trajectory = itinerary.trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const rmf_traffic::Duration itinerary_duration =
        finish_time - itinerary_start_time;

      dSOC_motion =
        task_planner_config.motion_sink()->compute_change_in_charge(
          trajectory);
      dSOC_device =
        task_planner_config.ambient_sink()->compute_change_in_charge(
          rmf_traffic::time::to_seconds(itinerary_duration));
      retreat_battery_drain += dSOC_motion + dSOC_device;
      retreat_duration +=itinerary_duration;
      itinerary_start_time = finish_time;
    }
    estimate_cache->set(endpoints, retreat_duration,
      retreat_battery_drain);
  }

  const double battery_soc_after_retreat =
    current_battery_soc - retreat_battery_drain;

  if ((battery_soc_after_retreat < retreat_threshold) &&
    (battery_soc_after_retreat > threshold_soc))
  {
    // Add a new charging task to the task queue
    auto charging_request = rmf_task::requests::ChargeBattery::make(
      task_planner_config.battery_system(),
      task_planner_config.motion_sink(),
      task_planner_config.ambient_sink(),
      task_planner_config.planner(),
      current_state.finish_time());

    const auto finish = charging_request->description()->estimate_finish(
      current_state,
      _context->task_planning_constraints(),
      estimate_cache);
    
    if (!finish)
      return;

    rmf_task::agv::TaskPlanner::Assignment charging_assignment(
      charging_request,
      finish.value().finish_state(),
      current_state.finish_time());

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


} // namespace rmf_fleet_adapter
