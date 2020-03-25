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
#include "Actions.hpp"

#include <rmf_traffic_ros2/Time.hpp>

#include "../rmf_fleet_adapter/make_trajectory.hpp"

#include <rmf_traffic/DetectConflict.hpp>

namespace rmf_fleet_adapter {
namespace full_control {

namespace {
//==============================================================================
class DispenseAction : public Action
{
public:

  DispenseAction(
      FleetAdapterNode* node,
      FleetAdapterNode::RobotContext* context,
      Task* task,
      std::string dispenser_name,
      std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items)
  : _node(node),
    _context(context),
    _task(task),
    _dispenser_name(dispenser_name),
    _items(std::move(items)),
    _robot_state_listener(this),
    _dispenser_state_listener(this),
    _dispenser_result_listener(this),
    _duration_estimate(std::chrono::minutes(2))
  {
    // Do nothing
  }

  void execute() final
  {
    _context->insert_listener(&_robot_state_listener);
    _node->dispenser_state_listeners.insert(&_dispenser_state_listener);
    _node->dispenser_result_listeners.insert(&_dispenser_result_listener);

    send_request();
    _task->report_status();
  }

  void send_request()
  {
    if (!_request)
    {
      _request = DispenserRequest{};
      _request->target_guid = _dispenser_name;
      _request->request_guid = _task->id();
      _request->transporter_type = _node->get_fleet_name();
      _request->time = _node->get_clock()->now();
      _request->items = _items;
    }

    _node->dispenser_request_publisher->publish(*_request);
  }

  void update()
  {
    update_schedule();

    if (_request_finished)
      finish();
  }

  rmf_traffic::Route calculate_itinerary() const
  {
    const auto now = _node->get_clock()->now();

    const auto l = _context->location;
    const Eigen::Vector3d position{l.x, l.y, l.yaw};
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

    const auto start = rmf_traffic_ros2::convert(now);
    const auto finish = rmf_traffic_ros2::convert(*_last_reported_wait_time);

    const std::string map_name =
        _node->get_graph().get_waypoint(0).get_map_name();

    rmf_traffic::Trajectory trajectory;
    trajectory.insert(start, position, zero);
    trajectory.insert(finish, position, zero);

    return {std::move(map_name), std::move(trajectory)};
  }

  void update_schedule()
  {
    const auto now = _node->get_clock()->now();

    if (_last_reported_wait_time)
    {
      const auto half_duration = _duration_estimate * 0.5;
      if (*_last_reported_wait_time - now < half_duration)
      {
        const auto next_wait_time = now + _duration_estimate;
        const auto delay = next_wait_time - *_last_reported_wait_time;
        _last_reported_wait_time = next_wait_time;

        _context->schedule.push_delay(
              rmf_traffic_ros2::convert(delay),
              rmf_traffic_ros2::convert(now));
        return;
      }
    }

    _last_reported_wait_time = now + _duration_estimate;
    _context->schedule.push_routes({calculate_itinerary()}, [](){});
  }

  void finish()
  {
    assert(_request_finished);
    _task->report_status();
    _task->next();
  }

  void interrupt() final
  {
    _emergency_active = true;
    update();
  }

  void resume() final
  {
    _emergency_active = false;
    update();
  }

  void respond(
      rmf_traffic::schedule::Negotiation::ConstTablePtr table,
      const Responder& responder,
      const bool* /*interrupt_flag*/) final
  {
    const rmf_traffic::Route route = calculate_itinerary();
    const auto& trajectory = route.trajectory();
    const auto view = table->query(rmf_traffic::schedule::make_query(
                   {route.map()},
                   trajectory.start_time(),
                   trajectory.finish_time()).spacetime());
    const auto& profile = _context->schedule.description().profile();

    for (const auto& v : view)
    {
      if (rmf_traffic::DetectConflict::between(
            profile,
            trajectory,
            v.description.profile(),
            v.route.trajectory()))
      {
        responder.reject();
        return;
      }
    }

    responder.submit({route}, [](){});
  }

  Status get_status() const final
  {
    std::string status = "Waiting for dispenser [" + _dispenser_name + "]";

    if (_request_finished)
      status += " - Request finished";
    else if (_request_received)
      status += " - Request received";

    if (_emergency_active)
      status += " - Emergency Interruption";

    return {status, _last_reported_wait_time};
  }

  using RobotState = rmf_fleet_msgs::msg::RobotState;
  class RobotStateListener : public Listener<RobotState>
  {
  public:

    RobotStateListener(DispenseAction* parent)
    : _parent(parent)
    {
      // Do nothing
    }

    void receive(const RobotState& /*msg*/) final
    {
      // TODO(MXG): Consider monitoring the robot's location to make sure it's
      // in the correct place for the delivery. The MoveAction that precedes
      // this dispense action should take care of that, but it's better to
      // explicitly check and enforce that.
      //
      // One challenge is that there may be an acceptable range for the robot
      // to be in, rather than a specific (x, y, yaw) location, so that should
      // be expressed somehow before enforcing the constraint.

      _parent->update();
    }

    DispenseAction* _parent;
  };

  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  class DispenserStateListener : public Listener<DispenserState>
  {
  public:

    DispenserStateListener(DispenseAction* parent)
    : _parent(parent)
    {
      // Do nothing
    }

    void receive(const DispenserState& msg) final
    {
      // We assume that if this callback is being triggered, then a request has
      // already been sent.
      assert(_parent->_request);

      if (!_parent->_request_received)
      {
        _parent->_request_received =
            std::find(
              msg.request_guid_queue.begin(),
              msg.request_guid_queue.end(),
              _parent->_request->request_guid)
            != msg.request_guid_queue.end();

        if (_parent->_request_received)
        {
          _parent->_task->report_status();
        }
      }

      if (!_parent->_request_received)
      {
        _parent->send_request();
      }
      else
      {
        // The request has been received, so if it's no longer in the queue,
        // then we'll assume it's finished.
        _parent->_request_finished =
            std::find(
              msg.request_guid_queue.begin(),
              msg.request_guid_queue.end(),
              _parent->_request->request_guid)
            == msg.request_guid_queue.end();
      }

      _parent->update();
    }

    DispenseAction* _parent;
  };

  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  class DispenserResultListener : public Listener<DispenserResult>
  {
  public:

    DispenserResultListener(DispenseAction* parent)
    : _parent(parent)
    {
      // Do nothing
    }

    void receive(const DispenserResult& msg) final
    {
      if (msg.source_guid != _parent->_dispenser_name)
        return;

      if (msg.request_guid != _parent->_request->request_guid)
        return;

      if (msg.status == DispenserResult::ACKNOWLEDGED)
      {
        const bool already_received = _parent->_request_received;
        _parent->_request_received = true;
        if (!already_received)
          _parent->_task->report_status();
      }
      else if (msg.status == DispenserResult::SUCCESS)
      {
        const bool already_finished = _parent->_request_finished;
        _parent->_request_finished = true;
        if (already_finished)
          _parent->_task->report_status();
      }
      else if (msg.status == DispenserResult::FAILED)
      {
        _parent->_request_received = false;
        _parent->_request_finished = false;
        _parent->send_request();
        _parent->_task->report_status();
      }

      _parent->update();
    }

    DispenseAction* _parent;
  };

  ~DispenseAction()
  {
    _context->remove_listener(&_robot_state_listener);
    _node->dispenser_state_listeners.erase(&_dispenser_state_listener);
    _node->dispenser_result_listeners.erase(&_dispenser_result_listener);
  }

private:

  FleetAdapterNode* const _node;
  FleetAdapterNode::RobotContext* const _context;
  Task* const _task;
  const std::string _dispenser_name;
  const std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> _items;

  RobotStateListener _robot_state_listener;
  DispenserStateListener _dispenser_state_listener;
  DispenserResultListener _dispenser_result_listener;

  rmf_utils::optional<rclcpp::Time> _last_reported_wait_time;
  const rclcpp::Duration _duration_estimate;

  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  rmf_utils::optional<DispenserRequest> _request;
  bool _request_received = false;
  bool _request_finished = false;

  bool _emergency_active = false;
};

} // anonymous namespace

//==============================================================================
std::unique_ptr<Action> make_dispense(
    FleetAdapterNode* const node,
    FleetAdapterNode::RobotContext* const context,
    Task* const parent,
    std::string dispenser_name,
    std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items,
    const rmf_task_msgs::msg::Behavior& /*behavior*/)
{
  return std::make_unique<DispenseAction>(
        node, context, parent,
        std::move(dispenser_name),
        std::move(items));
}

} // namespace full_control
} // namespace rmf_fleet_adapter
