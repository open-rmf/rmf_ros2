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

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_utils/math.hpp>

namespace rmf_fleet_adapter {
namespace full_control {

namespace {
//==============================================================================
rmf_utils::optional<std::size_t> get_fastest_plan_index(
    const std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>>& plans)
{
  auto nearest = std::chrono::steady_clock::time_point::max();
  std::size_t i_nearest = std::numeric_limits<std::size_t>::max();
  for (std::size_t i=0; i < plans.size(); ++i)
  {
    const auto& plan = plans[i];
    if (plan)
    {
      assert(plan->get_trajectories().back().finish_time());
      const auto finish_time = *plan->get_trajectories().back().finish_time();
      if (finish_time < nearest)
      {
        nearest = finish_time;
        i_nearest = i;
      }
    }
  }

  if (i_nearest == std::numeric_limits<std::size_t>::max())
  {
    return rmf_utils::nullopt;
  }

  return i_nearest;
}

//==============================================================================
class MoveAction : public Action
{
public:

  MoveAction(
      FleetAdapterNode* node,
      Task* parent,
      FleetAdapterNode::RobotContext* state,
      const std::size_t goal_wp_index,
      const std::vector<std::size_t>& fallback_wps)
  : _node(node),
    _task(parent),
    _context(state),
    _goal_wp_index(goal_wp_index),
    _fallback_wps(fallback_wps),
    _state_listener(this),
    _event_executor(this),
    _event_listener(this),
    _emergency_active(false),
    _waiting_on_emergency(false)
  {
    // Do nothing
  }

  void find_plan()
  {
    _emergency_active = false;
    _waiting_on_emergency = false;
    _plans.clear();

    const auto& planner = _node->get_planner();
    const auto plan_starts = _node->compute_plan_starts(_context->location);

    bool interrupt_flag = false;
    auto options = planner.get_default_options();
    options.interrupt_flag(&interrupt_flag);

    bool main_plan_finished = false;
    std::condition_variable main_plan_cv;
    rmf_utils::optional<rmf_traffic::agv::Plan> main_plan;
    std::thread main_plan_thread = std::thread(
          [&]()
    {
      main_plan = planner.plan(
            plan_starts, rmf_traffic::agv::Plan::Goal(_goal_wp_index), options);
      main_plan_finished = true;
      main_plan_cv.notify_all();
    });

    std::vector<std::thread> fallback_plan_threads;
    std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> fallback_plans;
    std::mutex fallback_plan_mutex;
    for (const std::size_t goal_wp : _fallback_wps)
    {
      fallback_plan_threads.emplace_back(std::thread([&]()
      {
        auto fallback_plan = planner.plan(
              plan_starts, rmf_traffic::agv::Plan::Goal(goal_wp), options);

        std::unique_lock<std::mutex> lock(fallback_plan_mutex);
        fallback_plans.emplace_back(std::move(fallback_plan));
      }));
    }

    const auto giveup_time =
        std::chrono::steady_clock::now() + _node->get_plan_time();

    // Waiting for the main planning thread is a bit complicated, because we
    // want to avoid the possibility that the plan finishes and triggers the
    // condition variable before we check it.
    while (std::chrono::steady_clock::now() < giveup_time
           && !main_plan_finished)
    {
      std::mutex placeholder;
      std::unique_lock<std::mutex> lock(placeholder);
      main_plan_cv.wait_for(lock, std::chrono::milliseconds(100),
                            [&](){ return main_plan_finished; });
    }

    interrupt_flag = true;
    main_plan_thread.join();
    for (auto& fallback_thread : fallback_plan_threads)
      fallback_thread.join();

    if (main_plan)
    {
      _plans.emplace_back(std::move(*std::move(main_plan)));
      return;
    }

    return use_fallback(std::move(fallback_plans));
  }

  void find_and_execute_plan()
  {
    find_plan();
    execute_plan();
  }

  void resolve() final
  {
    find_and_execute_plan();
  }

  void use_fallback(
      std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> fallback_plans)
  {
    const auto i_nearest_opt = get_fastest_plan_index(fallback_plans);
    if (!i_nearest_opt)
    {
      return _task->critical_failure(
            "The robot is trapped! Human intervention may be needed!");
    }
    const auto i_nearest = *i_nearest_opt;

    const auto& fallback_plan = *fallback_plans[i_nearest];
    const std::size_t fallback_waypoint =
        fallback_plan.get_waypoints().back().graph_index();
    const double fallback_orientation =
        fallback_plan.get_waypoints().back().position()[2];
    const auto fallback_end_time =
        *fallback_plan.get_trajectories().back().finish_time();

    const auto& planner = _node->get_planner();

    bool interrupt_flag = false;
    auto options = planner.get_default_options();
    options.interrupt_flag(&interrupt_flag);

    const auto t_spread = std::chrono::seconds(15);
    bool have_resume_plan = false;
    std::vector<std::thread> resume_plan_threads;
    std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> resume_plans;
    std::mutex resume_plan_mutex;
    std::condition_variable resume_plan_cv;
    for (std::size_t i=1; i < 9; ++i)
    {
      resume_plan_threads.emplace_back(std::thread([&]()
      {
        const auto resume_time = fallback_end_time + i*t_spread;
        auto resume_plan = planner.plan(
              rmf_traffic::agv::Plan::Start(
                resume_time, fallback_waypoint, fallback_orientation),
              rmf_traffic::agv::Plan::Goal(
                _goal_wp_index),
              options);

        std::unique_lock<std::mutex> lock(resume_plan_mutex);
        if (resume_plan)
          have_resume_plan = true;
        resume_plans.emplace_back(std::move(resume_plan));
        resume_plan_cv.notify_all();
      }));
    }

    const auto giveup_time =
        std::chrono::steady_clock::now() + _node->get_plan_time();

    while (std::chrono::steady_clock::now() < giveup_time && !have_resume_plan)
    {
      std::mutex placeholder;
      std::unique_lock<std::mutex> lock(placeholder);
      resume_plan_cv.wait_for(lock, std::chrono::milliseconds(100),
                              [&](){ return have_resume_plan; });
    }

    const auto quickest_finish_opt = get_fastest_plan_index(resume_plans);
    if (!quickest_finish_opt)
    {
      return _task->critical_failure(
            "The robot will be trapped at its fallback point! "
            "Human intervention may be needed!");
    }
    else
    {
      _plans.emplace_back(*resume_plans[*quickest_finish_opt]);
    }
  }

  std::vector<rmf_traffic::Trajectory> collect_trajectories() const
  {
    std::vector<rmf_traffic::Trajectory> trajectories;
    for (const auto& plan : _plans)
    {
      for (const auto& trajectory : plan.get_trajectories())
      {
        assert(trajectory.size() > 0);

        // If the trajectory has only one point then the robot doesn't need to
        // go anywhere.
        if (trajectory.size() < 2)
          continue;

        trajectories.emplace_back(trajectory);
      }
    }

    return trajectories;
  }


  void execute_plan()
  {
    _task->schedule.push_trajectories(
          collect_trajectories(),
          [&](){ command_plan(); });
  }

  void command_plan()
  {
    _waypoints.clear();
    for (const auto& plan : _plans)
      for (const auto& wp : plan.get_waypoints())
        _waypoints.emplace_back(wp);

    _command_segment = 0;
    send_next_command();
  }

  void send_next_command()
  {
    if (_waypoints.empty())
    {
      if (_emergency_active)
        report_waiting();
      else
        _task->next();
      return;
    }

    _command.fleet_name = _node->get_fleet_name();
    _command.task_id =
        _task->id() + std::to_string(_command_segment++);

    _command.path.clear();

    const auto& graph = _node->get_graph();

    std::size_t i=0;
    for (; i < _waypoints.size(); ++i)
    {
      const auto& wp = _waypoints[i];

      const auto& p = wp.position();

      rmf_fleet_msgs::msg::Location location;
      location.t = rmf_traffic_ros2::convert(wp.time());
      location.x = p[0];
      location.y = p[1];
      location.yaw = p[2];

      location.level_name =
          graph.get_waypoint(wp.graph_index()).get_map_name();

      _command.path.emplace_back(std::move(location));

      // Break off the command wherever an event occurs, because those are
      // points where the fleet adapter will need to issue door/lift requests.
      if (wp.event())
        break;
    }

    _waypoints.erase(_waypoints.begin(), _waypoints.begin()+i);

    publish_command();
    _command_time = _node->get_clock()->now();
  }

  void publish_command()
  {
    _node->path_request_publisher->publish(_command);
  }

  virtual ~MoveAction();

  using RobotState = rmf_fleet_msgs::msg::RobotState;
  class StateListener : public Listener<RobotState>
  {
  public:

    StateListener(MoveAction* parent)
    : _parent(parent)
    {
      // Do nothing
    }

    void receive(const RobotState& msg) final
    {
      if (_parent->_task->schedule.waiting())
        return;

      if (msg.task_id != _parent->_command.task_id)
      {
        const auto now = _parent->_node->get_clock()->now();

        // Recompute a plan if the fleet driver has a huge delay.
        if (now - _parent->_command_time > std::chrono::seconds(5))
        {
          RCLCPP_ERROR(
                _parent->_node->get_logger(),
                "The fleet driver is being unresponsive to task plan ["
                + _parent->_task->id() + "]. Recomputing plan!");

          return _parent->find_and_execute_plan();
        }

        // Attempt to resend the command if the robot has not received it yet.
        if (now - _parent->_command_time > std::chrono::milliseconds(100))
          _parent->publish_command();

        return;
      }

      if (msg.path.size() <= 1)
        _parent->handle_event(msg);

      _parent->handle_delay(msg);
    }

    MoveAction* _parent;
  };

  void handle_event(const RobotState& msg)
  {
    const auto& event_wp = _waypoints.front();
    if (!event_wp.event())
      return;

    // Check if we're already handling the event
    if (_event_executor.is_active())
      return;

    const auto& l = msg.location;
    const Eigen::Vector2d p{l.x, l.y};
    const Eigen::Vector2d target = event_wp.position().block<2,1>(0,0);

    // TODO(MXG): Consider making this threshold configurable
    if ((p - target).norm() < 0.1)
      event_wp.event()->execute(_event_executor);
  }

  bool check_delay(
      const RobotState& current,
      const std::size_t path_index)
  {
    const rmf_fleet_msgs::msg::Location& target = _command.path[path_index];
    const Eigen::Vector2d current_p = {current.location.x, current.location.y};
    const Eigen::Vector2d target_p = {target.x, target.y};
    if ( (current_p - target_p).norm() < 0.1 )
    {
      const double angle_diff =
          rmf_utils::wrap_to_pi(target.yaw - current.location.yaw);

      if (std::abs(angle_diff) < 10.0*M_PI/180.0)
      {
        // We are close to the target point, so let's see if we are far behind
        // the target time for this point.
        const auto target_t = rmf_traffic_ros2::convert(target.t);
        const auto current_t = rmf_traffic_ros2::convert(current.location.t);

        const auto delay = current_t - target_t;
        if (delay > std::chrono::seconds(5))
        {
          _task->schedule.push_delay(delay, target_t);
          for (std::size_t i=path_index; i < _command.path.size(); ++i)
          {
            _command.path[i].t = rmf_traffic_ros2::convert(
                  rmf_traffic_ros2::convert(_command.path[i].t) + delay);
          }
        }
      }
    }

    return false;
  }

  void handle_delay(const RobotState& msg)
  {
    // TODO(MXG): Come up with a more robust way to determine delays
    const std::size_t next_waypoint_index =
        _command.path.size() - msg.path.size();

    if (check_delay(msg, next_waypoint_index))
      return;

    // We can also check if we're close to the last waypoint and estimate the
    // delay based on that.
    if (next_waypoint_index > 0)
      check_delay(msg, next_waypoint_index-1);
  }

  void report_waiting()
  {
    _waiting_on_emergency = true;
    const auto now = rmf_traffic_ros2::convert(_node->get_clock()->now());
    const auto& profile = _node->get_fields().traits.get_profile();
    const auto& p = _context->location;
    const Eigen::Vector3d position{p.x, p.y, p.yaw};

    rmf_traffic::Trajectory wait_trajectory{p.level_name};

    wait_trajectory.insert(now, profile, position, Eigen::Vector3d::Zero());

    wait_trajectory.insert(
          now + std::chrono::minutes(5),
          profile, position, Eigen::Vector3d::Zero());

    _task->schedule.push_trajectories({wait_trajectory}, [](){});
  }

  using DoorState = rmf_door_msgs::msg::DoorState;
  using LiftState = rmf_lift_msgs::msg::LiftState;
  class EventListener
      : public Listener<DoorState>,
        public Listener<LiftState>
  {
  public:

    EventListener(MoveAction* parent)
    : _parent(parent)
    {
      _parent->_node->door_state_listeners.insert(this);
      _parent->_node->lift_state_listeners.insert(this);
    }

    std::function<void(const DoorState&)> door;
    std::function<void(const LiftState&)> lift;

    void receive(const DoorState& msg) final
    {
      if (door)
        door(msg);
    }

    void receive(const LiftState& msg) final
    {
      if (lift)
        lift(msg);
    }

    ~EventListener()
    {
      _parent->_node->door_state_listeners.erase(this);
      _parent->_node->lift_state_listeners.erase(this);
    }

  private:
    MoveAction* const _parent;
  };

  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;

  class EventExecutor : public rmf_traffic::agv::Graph::Lane::Executor
  {
  public:

    using Lane = rmf_traffic::agv::Graph::Lane;
    using DoorMode = rmf_door_msgs::msg::DoorMode;

    EventExecutor(MoveAction* parent)
    : _parent(parent),
      _is_active(false)
    {
      // Do nothing
    }

    bool is_active() const
    {
      return _is_active;
    }

    void request_door_mode(
        const std::string& door_name,
        const uint32_t mode)
    {
      _command_time = _parent->_node->get_clock()->now();

      DoorRequest request;
      request.door_name = door_name;
      request.request_time = _command_time;
      request.requested_mode.value = mode;
      request.requester_id = _parent->_node->get_fleet_name();

      _parent->_node->door_request_publisher->publish(request);
    }

    void wait_for_door_mode(
        const DoorState& msg,
        const uint32_t mode,
        const std::string& door_name,
        const rclcpp::Time& initial_time)
    {
      const auto time = rclcpp::Time(msg.door_time);
      if (time < initial_time)
        return;

      if (msg.door_name != door_name)
        return;

      if (mode == msg.current_mode.value)
      {
        _parent->_waypoints.erase(_parent->_waypoints.begin());
        _parent->_event_listener.door = nullptr;
        _is_active = false;
        _parent->send_next_command();
        return;
      }

      if ((time - _command_time).seconds() > 0.2)
      {
        // Send the command periodically as a precaution
        request_door_mode(door_name, mode);
      }
    }

    void execute(const Lane::DoorOpen& open) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string door_name = open.name();
      const auto initial_time = _parent->_node->get_clock()->now();

      _parent->_event_listener.door =
          [=](const DoorState& msg)
      {
        this->wait_for_door_mode(
              msg, DoorMode::MODE_OPEN, door_name, initial_time);
      };

      request_door_mode(door_name, DoorMode::MODE_OPEN);
    }

    void execute(const Lane::DoorClose& close) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string door_name = close.name();
      const auto initial_time = _parent->_node->get_clock()->now();
      _command_time = initial_time;

      _parent->_event_listener.door =
          [=](const DoorState& msg)
      {
        this->wait_for_door_mode(
              msg, DoorMode::MODE_CLOSED, door_name, initial_time);
      };

      request_door_mode(door_name, DoorMode::MODE_CLOSED);
    }

    void request_lift_mode(
        const std::string& lift_name,
        const std::string& floor_name,
        uint8_t lift_mode,
        uint8_t door_state)
    {
      _command_time = _parent->_node->get_clock()->now();

      LiftRequest request;
      request.lift_name = lift_name;
      request.session_id = _parent->_node->get_fleet_name();
      request.request_type = lift_mode;
      request.destination_floor = floor_name;
      request.door_state = door_state;

      _parent->_node->lift_request_publisher->publish(request);
    }

    void wait_for_lift_mode(
        const LiftState& msg,
        const std::string& lift_name,
        const std::string& floor_name,
        const uint8_t lift_mode,
        const uint8_t door_state,
        const rclcpp::Time& initial_time)
    {
      const auto time = rclcpp::Time(msg.lift_time);
      if (time < initial_time)
        return;

      if (msg.lift_name != lift_name)
        return;

      if ((time - _command_time).seconds() > 0.2)
      {
        // Send the command periodically as a precaution
        request_lift_mode(
              lift_name, floor_name, lift_mode, door_state);
      }

      if (msg.current_floor != floor_name)
        return;

      if (msg.door_state == door_state)
      {
        _parent->_waypoints.erase(_parent->_waypoints.begin());
        _parent->_event_listener.lift = nullptr;
        _is_active = false;
        _parent->send_next_command();
        return;
      }
    }

    void execute(const Lane::LiftDoorOpen& open) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string lift_name = open.lift_name();
      const std::string floor_name = open.floor_name();
      const auto initial_time = _parent->_node->get_clock()->now();

      _parent->_event_listener.lift =
          [=](const LiftState& msg)
      {
        this->wait_for_lift_mode(
              msg, lift_name, floor_name,
              LiftRequest::REQUEST_AGV_MODE,
              LiftRequest::DOOR_OPEN,
              initial_time);
      };

      request_lift_mode(
            lift_name, floor_name,
            LiftRequest::REQUEST_AGV_MODE,
            LiftRequest::DOOR_OPEN);
    }

    void execute(const Lane::LiftDoorClose& close) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string lift_name = close.lift_name();
      const std::string floor_name = close.floor_name();
      const auto initial_time = _parent->_node->get_clock()->now();

      // We don't actually need to wait for the lift door to close, so we'll
      // just shoot off the END_SESSION message and carry on with the next
      // step of the plan.
      request_lift_mode(
            lift_name, floor_name,
            LiftRequest::REQUEST_END_SESSION,
            LiftRequest::DOOR_CLOSED);

      _parent->_waypoints.erase(_parent->_waypoints.begin());
      _parent->send_next_command();
    }

    void execute(const Lane::LiftMove& move) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string lift_name = move.lift_name();
      const std::string floor_name = move.destination_floor();
      const auto initial_time = _parent->_node->get_clock()->now();

      _parent->_event_listener.lift =
          [=](const LiftState& msg)
      {
        this->wait_for_lift_mode(
              msg, lift_name, floor_name,
              LiftRequest::REQUEST_AGV_MODE,
              LiftRequest::DOOR_OPEN,
              initial_time);
      };
    }

    void cancel()
    {
      _parent->_event_listener.door = nullptr;
      _parent->_event_listener.lift = nullptr;
      _is_active = false;
    }

  private:
    rclcpp::Time _command_time;
    MoveAction* const _parent;
    bool _is_active;
  };

  void execute() final
  {
    _context->state_listeners.insert(&_state_listener);
    find_and_execute_plan();
  }

  void find_and_execute_emergency_plan()
  {
    _emergency_active = true;

    const auto& planner = _node->get_planner();

    const auto plan_starts = _node->compute_plan_starts(_context->location);

    bool interrupt_flag = false;
    auto options = planner.get_default_options();
    options.interrupt_flag(&interrupt_flag);

    std::vector<std::thread> plan_threads;
    std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> plans;
    std::mutex plans_mutex;
    std::condition_variable plans_cv;
    bool have_plan = false;
    for (const std::size_t goal_wp : _fallback_wps)
    {
      plan_threads.emplace_back(std::thread([&]()
      {
        auto emergency_plan = planner.plan(
              plan_starts, rmf_traffic::agv::Plan::Goal(goal_wp), options);

        std::unique_lock<std::mutex> lock(plans_mutex);
        if (emergency_plan)
          have_plan = true;

        plans.emplace_back(std::move(emergency_plan));
        plans_cv.notify_all();
      }));
    }

    const auto giveup_time =
        std::chrono::steady_clock::now() + 5*_node->get_plan_time();

    while (std::chrono::steady_clock::now() < giveup_time && !have_plan)
    {
      std::unique_lock<std::mutex> lock(plans_mutex);
      plans_cv.wait_for(lock, std::chrono::milliseconds(100),
                        [&](){ return have_plan; });
    }

    const auto quickest_finish_opt = get_fastest_plan_index(plans);
    if (!quickest_finish_opt)
    {
      return _task->critical_failure(
            "The robot failed to find a viable emergency plan! "
            "Human intervention may be needed!");
    }

    _plans.emplace_back(*plans[*quickest_finish_opt]);

    return execute_plan();
  }

  void cancel()
  {
    _waypoints.clear();
    _event_executor.cancel();
    // TODO(MXG): Should we issue a command to the robot to stop?
  }

  void interrupt() final
  {
    if (_emergency_active)
      return;

    cancel();
    find_and_execute_emergency_plan();
  }

  void resume() final
  {
    if (!_emergency_active)
      return;

    cancel();
    find_and_execute_plan();
  }

  Status get_status() const final
  {
    std::string status;

    const auto& waypoint_names = _node->get_waypoint_names();

    auto name_of = [&](std::size_t wp_index) -> std::string
    {
      const auto wp_it = waypoint_names.find(wp_index);
      return wp_it == waypoint_names.end()?
            "waypoint #" + std::to_string(_goal_wp_index) : wp_it->second;
    };

    status = "Moving to dispenser at [" + name_of(_goal_wp_index) + "] - ";

    if (_emergency_active)
      status += "Emergency Interruption - ";
    else
      status = "In progress - ";

    if (_waiting_on_emergency)
    {
      status += "Waiting for emergency to end";
      return {status};
    }
    else if (_waypoints.empty())
    {
      status += "Empty Plan - May require human intervention";
      return {status};
    }

    for (const auto& wp : _waypoints)
    {
      if (wp.graph_index())
      {
        status += "next waypoint: [" + name_of(*wp.graph_index()) + "]";
        break;
      }
    }

    const auto& final_wp = _waypoints.back();
    const auto end_time = rmf_traffic_ros2::to_ros2(final_wp.time());
    return {status, end_time};
  }


private:
  FleetAdapterNode* const _node;
  Task* const _task;
  FleetAdapterNode::RobotContext* const _context;
  const std::size_t _goal_wp_index;
  const std::vector<std::size_t>& _fallback_wps;
  StateListener _state_listener;
  EventExecutor _event_executor;
  EventListener _event_listener;

  rclcpp::Time _command_time;
  std::vector<rmf_traffic::agv::Plan> _plans;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
  rmf_fleet_msgs::msg::PathRequest _command;
  std::size_t _command_segment;

  bool _emergency_active;
  bool _waiting_on_emergency;

};

MoveAction::~MoveAction()
{
  _context->state_listeners.erase(&_state_listener);
}

} // anonymous namespace

//==============================================================================
std::unique_ptr<Action> make_move(
    FleetAdapterNode* node,
    FleetAdapterNode::RobotContext* state,
    Task* parent,
    const std::size_t goal_wp_index,
    const std::vector<std::size_t>& fallback_wps)
{
  return std::make_unique<MoveAction>(
        node, parent, state, goal_wp_index, fallback_wps);
}

} // namespace full_control
} // namespace rmf_fleet_adapter
