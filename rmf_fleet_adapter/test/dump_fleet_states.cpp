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

#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <cmath>

#include <Eigen/Geometry>


using Location = rmf_fleet_msgs::msg::Location;
using RobotState = rmf_fleet_msgs::msg::RobotState;
using RobotMode = rmf_fleet_msgs::msg::RobotMode;
using FleetState = rmf_fleet_msgs::msg::FleetState;

Location make_location(float x, float y, float yaw)
{
  Location l;
  l.x = x;
  l.y = y;
  l.yaw = yaw;
  l.level_name = "test_level";
  return l;
}

Eigen::Vector3d to_eigen(Location l)
{
  return Eigen::Vector3d(l.x, l.y, l.yaw);
}

Location to_location(Eigen::Vector3d v)
{
  return make_location(v[0], v[1], v[2]);
}


class DumpFleetStates : public rclcpp::Node
{
public:


  DumpFleetStates()
  : rclcpp::Node("dump_test_fleet_states")
  {
    fleet_state_publisher = create_publisher<FleetState>(
      "/fleet_states", rclcpp::SystemDefaultsQoS());

    timer = create_wall_timer(
      std::chrono::milliseconds(period_ms), [&]() { update(); });

    current_location = Eigen::Vector3d(0.0, 0.0, 0.0);

    path.emplace_back(make_location(distance/2.0, 0.0, 0.0));
    path.emplace_back(make_location(distance, 0.0, 0.0));
    path.emplace_back(make_location(distance, 0.0, 90.0*M_PI/180.0));
    path.emplace_back(make_location(distance, distance/4.0, 90.0*M_PI/180.0));
    path.emplace_back(make_location(distance, distance, 90.0*M_PI/180.0));
  }

private:

  void update()
  {
    std::cout << "Current location: " << current_location.transpose() <<
      std::endl;

    if (path.empty())
      return publish();

    const Eigen::Vector3d next_location = to_eigen(path.front());

    const Eigen::Vector2d p = current_location.block<2, 1>(0, 0);
    const Eigen::Vector2d p_next = next_location.block<2, 1>(0, 0);

    const Eigen::Vector2d n = p_next - p;
    const double dist = n.norm();
    if (dist <= velocity*timestep)
    {
      const double yaw = current_location[2];
      const double next_yaw = next_location[2];
      const double d_yaw = next_yaw - yaw;
      if (std::abs(d_yaw) <= velocity*timestep)
      {
        current_location = next_location;
        path.erase(path.begin());
        return publish();
      }

      current_location.block<2, 1>(0, 0) = p_next;
      current_location[2] += d_yaw;

      return publish();
    }

    const Eigen::Vector2d dir = n/dist;
    current_location.block<2, 1>(0, 0) += velocity*timestep*dir;
    return publish();
  }

  void publish()
  {
    FleetState fleet;
    fleet.name = "test_fleet";

    RobotState robot;
    robot.name = "test_robot";
    robot.mode.mode = RobotMode::MODE_MOVING;
    robot.location = to_location(current_location);
    robot.location.t = now();
    robot.path = path;
    robot.task_id = "test";
    fleet.robots.push_back(robot);

    fleet_state_publisher->publish(fleet);
  }

  const double distance = 10.0;
  const double velocity = 0.4;
  const int64_t period_ms = 200;
  const double timestep = static_cast<double>(period_ms)/1000.0;

  Eigen::Vector3d current_location;

  std::vector<Location> path;

  rclcpp::Publisher<FleetState>::SharedPtr fleet_state_publisher;

  rclcpp::TimerBase::SharedPtr timer;

};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DumpFleetStates>());
  rclcpp::shutdown();
}
