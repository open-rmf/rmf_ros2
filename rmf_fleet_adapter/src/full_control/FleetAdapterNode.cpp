/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "FleetAdapterNode.hpp"

#include "../rmf_fleet_adapter/ParseGraph.hpp"
#include "../rmf_fleet_adapter/load_param.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rclcpp/executors.hpp>

#include "Actions.hpp"
#include "Tasks.hpp"

#include <rmf_traffic/geometry/Circle.hpp>

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make()
{
  const auto node = std::shared_ptr<FleetAdapterNode>(new FleetAdapterNode);

  node->_perform_deliveries =
    node->declare_parameter<bool>("perform_deliveries", false);

  const std::string nav_graph_param_name = "nav_graph_file";
  std::string graph_file =
    node->declare_parameter(nav_graph_param_name, std::string());

  if (graph_file.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [" + nav_graph_param_name + "] parameter!");

    return nullptr;
  }

  const std::string fleet_name = node->get_fleet_name().empty() ?
    "all fleets" : "[" + node->get_fleet_name() + "]";
  RCLCPP_INFO(
    node->get_logger(),
    "Launching fleet adapter for " + fleet_name);

  auto traits = get_traits_or_default(*node, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5);

  node->_delay_threshold =
    get_parameter_or_default_time(*node, "delay_threshold", 5.0);

  node->_retry_wait =
    get_parameter_or_default_time(*node, "retry_wait", 5.0);

  node->_plan_time =
    get_parameter_or_default_time(*node, "planning_timeout", 5.0);

  auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
    *node, rmf_traffic::schedule::query_all());

  rmf_utils::optional<GraphInfo> graph_info =
    parse_graph(graph_file, traits, *node);

  if (!graph_info)
    return nullptr;

  using namespace std::chrono_literals;

  auto writer = rmf_traffic_ros2::schedule::Writer::make(*node);

  const auto wait_time =
    get_parameter_or_default_time(*node, "discovery_timeout", 10.0);

  const auto stop_time = std::chrono::steady_clock::now() + wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    bool ready = true;
    ready &= writer->ready();
    ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

    if (ready)
    {
      node->start(
        Fields{
          *node,
          std::move(*graph_info),
          std::move(traits),
          mirror_future.get(),
          std::move(writer)
        });

      return node;
    }
  }

  RCLCPP_ERROR(
    node->get_logger(),
    "Timeout after waiting ["
    + std::to_string(rmf_traffic::time::to_seconds(wait_time))
    + "] to connect to the schedule");

  return nullptr;
}

//==============================================================================
FleetAdapterNode::RobotContext::RobotContext(
  std::string name,
  Location location_,
  ScheduleManager schedule_)
: location(std::move(location_)),
  schedule(std::move(schedule_)),
  _name(std::move(name))
{
  schedule.set_negotiator(
    [this](
      rmf_traffic::schedule::Negotiation::ConstTablePtr table,
      const rmf_traffic::schedule::Negotiator::Responder& responder,
      const bool* interrupt_flag)
    {
      this->respond(std::move(table), responder, interrupt_flag);
    });
}

//==============================================================================
void FleetAdapterNode::RobotContext::next_task()
{
  // TODO(MXG): Report the current task complete before clearing it
  if (!_task_queue.empty())
  {
    std::cout << "Starting the next task for [" << _name << "]" << std::endl;
    _task = std::move(_task_queue.front());
    _task_queue.erase(_task_queue.begin());
    _task->next();
    return;
  }

  std::cout << " ======== Task complete for [" << _name << "]" << std::endl;
  _task = nullptr;

  // TODO(MXG): If the task queue is empty, have the robot move back to its
  // home.
}

//==============================================================================
void FleetAdapterNode::RobotContext::add_task(std::unique_ptr<Task> new_task)
{
  if (!_task)
  {
    // No task is currently active, so set this new task as the active one and
    // begin executing it.
    std::cout << "beginning task" << std::endl;
    _task = std::move(new_task);
    _task->next();
  }
  else
  {
    std::cout << "appending task" << std::endl;
    // A task is currently active, so add this task to the queue. We're going to
    // do first-come first-serve for task requests for now.
    _task_queue.emplace_back(std::move(_task));
  }
}

//==============================================================================
void FleetAdapterNode::RobotContext::discard_task(Task* discarded_task)
{
  if (_task.get() == discarded_task)
  {
    return next_task();
  }

  const auto it = std::find_if(_task_queue.begin(), _task_queue.end(),
      [&](const std::unique_ptr<Task>& task)
      {
        return task.get() == discarded_task;
      });

  if (it != _task_queue.end())
    _task_queue.erase(it);
}

//==============================================================================
void FleetAdapterNode::RobotContext::interrupt()
{
  if (_task)
    _task->interrupt();
}

//==============================================================================
void FleetAdapterNode::RobotContext::resume()
{
  if (_task)
    _task->resume();
}

//==============================================================================
void FleetAdapterNode::RobotContext::respond(
  rmf_traffic::schedule::Negotiation::ConstTablePtr table,
  const rmf_traffic::schedule::Negotiator::Responder& responder,
  const bool* interrupt_flag)
{
  if (_task)
    _task->respond(std::move(table), responder, interrupt_flag);
  else
  {
    // This would be very suspicious if it happens
    std::cerr << "[FleetAdatperNode::RobotContext::respond] "
              << "Responding to a negotiation while not performing a task"
              << std::endl;
    responder.submit({});
  }
}

//==============================================================================
std::size_t FleetAdapterNode::RobotContext::num_tasks() const
{
  const std::size_t n_queue = _task_queue.size();
  return _task ? n_queue + 1 : n_queue;
}

//==============================================================================
const std::string& FleetAdapterNode::RobotContext::robot_name() const
{
  return _name;
}

//==============================================================================
void FleetAdapterNode::RobotContext::insert_listener(
  Listener<RobotState>* listener)
{
  state_listeners.insert(listener);
}

//==============================================================================
void FleetAdapterNode::RobotContext::remove_listener(
  Listener<RobotState>* listener)
{
  state_listeners.erase(listener);
}

//==============================================================================
void FleetAdapterNode::RobotContext::update_listeners(const RobotState& state)
{
  // We need to copy this container before iterating over it, because it's
  // very possible for the container to get modified by Actions while we iterate
  // over it.
  const auto current_listeners = state_listeners;
  for (auto* listener : current_listeners)
    listener->receive(state);
}

//==============================================================================
bool FleetAdapterNode::ignore_fleet(const std::string& fleet_name) const
{
  if (!_fleet_name.empty() && fleet_name != _fleet_name)
    return true;

  return false;
}

//==============================================================================
const std::string& FleetAdapterNode::get_fleet_name() const
{
  return _fleet_name;
}

//==============================================================================
rmf_traffic::Duration FleetAdapterNode::get_plan_time() const
{
  return _plan_time;
}

//==============================================================================
rmf_traffic::Duration FleetAdapterNode::get_delay_threshold() const
{
  return _delay_threshold;
}

//==============================================================================
rmf_traffic::Duration FleetAdapterNode::get_retry_wait() const
{
  return _retry_wait;
}

//==============================================================================
const rmf_traffic::agv::Planner& FleetAdapterNode::get_planner() const
{
  return _field->planner;
}

//==============================================================================
const rmf_traffic::agv::Graph& FleetAdapterNode::get_graph() const
{
  return _field->graph_info.graph;
}

//==============================================================================
auto FleetAdapterNode::get_waypoint_keys() const -> const WaypointKeys&
{
  return _field->graph_info.keys;
}

//==============================================================================
auto FleetAdapterNode::get_waypoint_names() const -> const WaypointNames&
{
  return _field->graph_info.waypoint_names;
}

//==============================================================================
const std::vector<std::size_t>& FleetAdapterNode::get_parking_spots() const
{
  return _field->graph_info.parking_spots;
}

//==============================================================================
auto FleetAdapterNode::get_fields() -> Fields&
{
  return *_field;
}

//==============================================================================
auto FleetAdapterNode::get_fields() const -> const Fields&
{
  return *_field;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode()
: rclcpp::Node("fleet_adapter"),
  _fleet_name(get_fleet_name_parameter(*this))
{
  // Do nothing
}

//==============================================================================
void FleetAdapterNode::start(Fields fields)
{
  _field = std::move(fields);
  _field->mirror.update();

  const auto default_qos = rclcpp::SystemDefaultsQoS();

  _delivery_sub = create_subscription<Delivery>(
    DeliveryTopicName, default_qos,
    [&](Delivery::UniquePtr msg)
    {
      this->delivery_request(std::move(msg));
    });

  _loop_request_sub = create_subscription<LoopRequest>(
    LoopRequestTopicName, default_qos,
    [&](LoopRequest::UniquePtr msg)
    {
      this->loop_request(std::move(msg));
    });

  _dispenser_result_sub = create_subscription<DispenserResult>(
    DispenserResultTopicName, default_qos,
    [&](DispenserResult::UniquePtr msg)
    {
      this->dispenser_result_update(std::move(msg));
    });

  _dispenser_state_sub = create_subscription<DispenserState>(
    DispenserStateTopicName, default_qos,
    [&](DispenserState::UniquePtr msg)
    {
      this->dispenser_state_update(std::move(msg));
    });

  _fleet_state_sub = create_subscription<FleetState>(
    FleetStateTopicName, default_qos,
    [&](FleetState::UniquePtr msg)
    {
      this->fleet_state_update(std::move(msg));
    });

  _door_state_sub = create_subscription<DoorState>(
    DoorStateTopicName, default_qos,
    [&](DoorState::UniquePtr msg)
    {
      this->door_state_update(std::move(msg));
    });

  _lift_state_sub = create_subscription<LiftState>(
    LiftStateTopicName, default_qos,
    [&](LiftState::UniquePtr msg)
    {
      this->lift_state_update(std::move(msg));
    });

  _emergency_notice_sub = create_subscription<EmergencyNotice>(
    rmf_traffic_ros2::EmergencyTopicName, default_qos,
    [&](EmergencyNotice::UniquePtr msg)
    {
      this->emergency_notice_update(std::move(msg));
    });

  path_request_publisher = create_publisher<PathRequest>(
    PathRequestTopicName, default_qos);

  mode_request_publisher = create_publisher<ModeRequest>(
    ModeRequestTopicName, default_qos);

  door_request_publisher = create_publisher<DoorRequest>(
    AdapterDoorRequestTopicName, default_qos);

  lift_request_publisher = create_publisher<LiftRequest>(
    AdapterLiftRequestTopicName, default_qos);

  dispenser_request_publisher = create_publisher<DispenserRequest>(
    DispenserRequestTopicName, default_qos);

  task_summary_publisher = create_publisher<TaskSummary>(
    TaskSummaryTopicName, default_qos);
}

//==============================================================================
void FleetAdapterNode::delivery_request(Delivery::UniquePtr msg)
{
  if (!_perform_deliveries)
    return;

  if (_in_emergency_mode)
  {
    RCLCPP_ERROR(
      get_logger(),
      "We do not currently support receiving tasks during emergencies!");
    return;
  }

  if (_contexts.empty())
  {
    RCLCPP_ERROR(
      get_logger(),
      "No robots appear to be online!");
    return;
  }

  auto task_insertion = _received_tasks.insert(msg->task_id);
  if (!task_insertion.second)
  {
    // We've already received and processed this task in the past, so we can
    // ignore it.
    RCLCPP_INFO(
      get_logger(),
      "Already received delivery task [" + msg->task_id + "] so it will "
      "be ignored");
    return;
  }

  if (_have_delivery_request)
    return;

  // TODO(MXG): Support multiple simultaneous deliveries
  auto context = _contexts.begin()->second.get();

  RCLCPP_INFO(
    get_logger(),
    "Assigning delivery task to [" + context->robot_name() + "]");

  std::cout << "Robot name: [" << context->robot_name() << "]  | ptr: ["
            << context << "]" << std::endl;

  auto task = make_delivery(this, context, *msg);
  if (task)
  {
    _have_delivery_request = true;
    context->add_task(std::move(task));
  }
}

//==============================================================================
void FleetAdapterNode::loop_request(LoopRequest::UniquePtr msg)
{
  if (_in_emergency_mode)
  {
    RCLCPP_ERROR(
      get_logger(),
      "We do not currently support receiving tasks during emergencies!");
    return;
  }

  if (ignore_fleet(msg->robot_type))
    return;

  std::size_t fewest = std::numeric_limits<std::size_t>::max();
  RobotContext* fewest_context = nullptr;
  for (const auto& c : _contexts)
  {
    const std::size_t n = c.second->num_tasks();
    if (n < fewest)
    {
      fewest = n;
      fewest_context = c.second.get();
    }
  }

  if (fewest_context)
  {
    // TODO(MXG): We should make a queue of tasks for the whole fleet instead of
    // queuing up like this.
    if (!_received_tasks.insert(msg->task_id).second)
    {
      RCLCPP_INFO(
        get_logger(),
        "Already received looping task request [" + msg->task_id
        + "] so it will be ignored");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Received looping task ID [" + msg->task_id + "] and assigned "
      "it to [" + fewest_context->robot_name() + "]");

    auto task = make_loop(this, fewest_context, std::move(*msg));
    if (task)
      fewest_context->add_task(std::move(task));

    return;
  }

  RCLCPP_ERROR(
    get_logger(),
    "Cannot assign a robot to looping task [" + msg->task_id
    + "] because no robots have reported their existence.");

  rmf_task_msgs::msg::TaskSummary summary;
  summary.status = "Services unavailable";
  summary.start_time = get_clock()->now();
  summary.end_time = summary.start_time;
  summary.task_id = msg->task_id;
  summary.submission_time = summary.start_time;

  task_summary_publisher->publish(summary);
}

//==============================================================================
void FleetAdapterNode::dispenser_result_update(DispenserResult::UniquePtr msg)
{
  const auto current_listeners = dispenser_result_listeners;
  for (auto* listener : current_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::dispenser_state_update(DispenserState::UniquePtr msg)
{
  const auto current_listeners = dispenser_state_listeners;
  for (auto* listener : current_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::fleet_state_update(FleetState::UniquePtr msg)
{
  const auto& fleet_state = *msg;
  if (ignore_fleet(fleet_state.name))
    return;

  for (const auto& robot : fleet_state.robots)
  {
    const auto insertion = _contexts.insert(
      std::make_pair(robot.name, nullptr));
    const bool inserted = insertion.second;
    const auto it = insertion.first;

    if (inserted)
    {
      // This robot has not been seen before, so we should create a schedule
      // manager for it and instantiate it in the map

      rmf_traffic::schedule::ParticipantDescription description{
        robot.name,
        get_fleet_name(),
        rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
        _field->traits.profile()
      };

      async_make_schedule_manager(
        *this,
        *_field->writer,
        &_field->negotiation,
        std::move(description),
        [this, name = robot.name, location = robot.location](
          ScheduleManager manager)
        {
          this->_contexts.at(name) = std::make_unique<RobotContext>(
            name, location, std::move(manager));
        }, _async_mutex);

      RCLCPP_INFO(
        get_logger(),
        "Found a robot: [" + robot.name + "]");
    }
    else if (it->second)
    {
      it->second->location = robot.location;
      it->second->update_listeners(robot);
    }
    // else
    // {
    //   we are waiting for the schedule manager to finish being created
    // }
  }
}

//==============================================================================
void FleetAdapterNode::door_state_update(DoorState::UniquePtr msg)
{
  const auto current_listeners = door_state_listeners;
  for (const auto& listener : current_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::lift_state_update(LiftState::UniquePtr msg)
{
  const auto current_listeners = lift_state_listeners;
  for (const auto& listener : current_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::emergency_notice_update(EmergencyNotice::UniquePtr msg)
{
  const bool active_emergency = msg->data;
  if (active_emergency == _in_emergency_mode)
    return;

  _in_emergency_mode = active_emergency;

  if (active_emergency)
  {
    for (const auto& c : _contexts)
      c.second->interrupt();
  }

  if (!active_emergency)
  {
    for (const auto& c : _contexts)
      c.second->resume();
  }
}

} // namespace full_control
} // namespace rmf_fleet_adapter
