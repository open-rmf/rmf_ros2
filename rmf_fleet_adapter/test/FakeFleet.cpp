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

#include <vector>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

class FakeFleetDriver : public rclcpp::Node
{
public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using RobotState = rmf_fleet_msgs::msg::RobotState;
  using RobotMode = rmf_fleet_msgs::msg::RobotMode;
  using Location = rmf_fleet_msgs::msg::Location;

  FakeFleetDriver(const std::string& _fleet_id)
  : fleet_id(_fleet_id),
    Node(_fleet_id + "_driver")
  {
    fleet_state_pub = create_publisher<FleetState>(
        fleet_id + "/fleet_state", rclcpp::SystemDefaultsQoS());

    using namespace std::chrono_literals;
    timer = create_wall_timer(
        1000ms, std::bind(&FakeFleetDriver::publish_fleet_state, this));

    steady_clock.reset(new rclcpp::Clock(RCL_STEADY_TIME));

    // fake fleet
    fleet_state_msg.name = fleet_id;

    RobotState robot;
    robot.name = "diagonal_robot";
    robot.model = "pepper";
    RobotMode sr_mode;
    sr_mode.mode = sr_mode.MODE_MOVING;
    robot.mode = sr_mode;
    robot.battery_percent = 88.8;
    Location sr_loc;
    sr_loc.t = steady_clock->now();
    sr_loc.x = 0.0;
    sr_loc.y = 0.0;
    sr_loc.yaw = 0.0;
    sr_loc.level_name = "B1";
    robot.location = sr_loc;

    robot.path.clear();
    for (int j = 0; j < 10; ++j)
    {
      Location wp;
      wp = sr_loc;
      wp.t.sec += 1 * (j + 1);
      wp.x += 1.0 * (j + 1);
      wp.y += 1.0 * (j + 1);
      robot.path.push_back(wp);
    }

    fleet_state_msg.robots.clear();
    fleet_state_msg.robots = {robot};
  }

private:

  std::string fleet_id;

  rclcpp::Publisher<FleetState>::SharedPtr fleet_state_pub;

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<rclcpp::Clock> steady_clock;

  FleetState fleet_state_msg;

  void generate_next_fleet_state_msg()
  {
    auto robot = fleet_state_msg.robots[0];

    robot.location = robot.path[robot.path.size() - 1];
    robot.location.t = steady_clock->now();
    robot.location.x += 1.0;
    robot.location.y += 1.0;

    robot.path.clear();
    for (int i = 0; i < 10; ++i)
    {
      Location wp;
      wp = robot.location;
      wp.t.sec += 1 * (i + 1);
      wp.x += 1.0 * (i + 1);
      wp.y += 1.0 * (i + 1);
      robot.path.push_back(wp);
    }

    fleet_state_msg.robots.clear();
    fleet_state_msg.robots = {robot};
  }

  void publish_fleet_state()
  {
    RCLCPP_INFO(get_logger(), "generate next fleet state!");
    generate_next_fleet_state_msg();

    RCLCPP_INFO(get_logger(), "publishing new fleet state!");
    fleet_state_pub->publish(fleet_state_msg);
  }

};

int main(int argc, char** argv)
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  rclcpp::spin(std::make_shared<FakeFleetDriver>("fake_fleet"));

  rclcpp::shutdown();
  std::cout << "all done" << std::endl;
  return 0;
}