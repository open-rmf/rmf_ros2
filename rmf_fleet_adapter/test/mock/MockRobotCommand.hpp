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

#ifndef RMF_FLEET_ADAPTER__TEST__MOCK__MOCKROBOTCOMMAND
#define RMF_FLEET_ADAPTER__TEST__MOCK__MOCKROBOTCOMMAND

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

#include <rclcpp/node.hpp>

namespace rmf_fleet_adapter_test {

//==============================================================================
class MockRobotCommand : public rmf_fleet_adapter::agv::RobotCommandHandle
{
public:

  class EventListener : public rmf_traffic::agv::Graph::Lane::Executor
  {
  public:

    EventListener(
        std::unordered_map<std::string, std::size_t>& dock_to_wp,
        std::size_t wp)
      : _dock_to_wp(dock_to_wp),
        _wp(wp)
    {
      // Do nothing
    }

    void execute(const Dock& dock) override
    {
      _dock_to_wp[dock.dock_name()] = _wp;
    }

    void execute(const DoorOpen&) override { }
    void execute(const DoorClose&) override { }
    void execute(const LiftSessionBegin&) override { }
    void execute(const LiftMove&) override { }
    void execute(const LiftDoorOpen&) override { }
    void execute(const LiftSessionEnd&) override { }
    void execute(const Wait&) override { }

  private:
    std::unordered_map<std::string, std::size_t>& _dock_to_wp;
    std::size_t _wp;
  };

  MockRobotCommand(
      std::shared_ptr<rclcpp::Node> node,
      const rmf_traffic::agv::Graph& graph)
    : _node(std::move(node))
  {
    for (std::size_t i=0; i < graph.num_lanes(); ++i)
    {
      const auto& lane = graph.get_lane(i);

      if (lane.entry().event())
      {
        EventListener executor(_dock_to_wp, lane.exit().waypoint_index());
        lane.entry().event()->execute(executor);
      }
    }
  }

  std::shared_ptr<rmf_fleet_adapter::agv::RobotUpdateHandle> updater;

  void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      ArrivalEstimator next_arrival_estimator,
      std::function<void()> path_finished_callback) final
  {
    _current_waypoint_target = 0;
    _active = true;
    _timer = _node->create_wall_timer(
          std::chrono::milliseconds(10),
          [this,
           waypoints,
           next_arrival_estimator = std::move(next_arrival_estimator),
           path_finished_callback = std::move(path_finished_callback)]
    {
      if (_pause)
        return;

      if (!_active)
        return;

      if (_current_waypoint_target < waypoints.size())
        ++_current_waypoint_target;

      if (updater)
      {
        const auto& previous_wp = waypoints[_current_waypoint_target-1];
        if (previous_wp.graph_index())
        {
          updater->update_position(
                *previous_wp.graph_index(), previous_wp.position()[2]);
          ++_visited_wps.insert({*previous_wp.graph_index(), 0}).first->second;
        }
        else
        {
          updater->update_position("test_map", previous_wp.position());
        }
      }

      if (_current_waypoint_target < waypoints.size())
      {
        const auto& wp = waypoints[_current_waypoint_target];
        const auto test_delay =
            std::chrono::milliseconds(750) * _current_waypoint_target;

        const auto delayed_arrival_time = wp.time() + test_delay;
        const auto remaining_time =
            std::chrono::steady_clock::time_point(
              std::chrono::steady_clock::duration(_node->now().nanoseconds()))
              - delayed_arrival_time;

        next_arrival_estimator(_current_waypoint_target, remaining_time);
        return;
      }

      _active = false;
      _timer.reset();
      path_finished_callback();
    });
  }

  void stop() final
  {
    _timer.reset();
  }

  void dock(
      const std::string& dock_name,
      std::function<void()> docking_finished_callback) final
  {
    assert(_dock_to_wp.find(dock_name) != _dock_to_wp.end());
    ++_dockings.insert({dock_name, 0}).first->second;
    const auto wp = _dock_to_wp.at(dock_name);
    ++_visited_wps.insert({wp, 0}).first->second;
    updater->update_position(wp, 0.0);
    docking_finished_callback();
  }

  const std::unordered_map<std::string, std::size_t>& dockings() const
  {
    return _dockings;
  }

  const std::unordered_map<std::size_t, std::size_t> visited_wps() const
  {
    return _visited_wps;
  }

  void pause(bool on)
  {
    _pause = on;
  }

private:
  bool _active = false;
  bool _pause = false;
  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::TimerBase::SharedPtr _timer;
  std::size_t _current_waypoint_target = 0;
  std::unordered_map<std::string, std::size_t> _dockings;
  std::unordered_map<std::size_t, std::size_t> _visited_wps;
  std::unordered_map<std::string, std::size_t> _dock_to_wp;
};

} // namespace rmf_fleet_adapter_test

#endif // RMF_FLEET_ADAPTER__TEST__MOCK__MOCKROBOTCOMMAND
