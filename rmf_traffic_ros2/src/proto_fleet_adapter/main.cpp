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
#include "ParseGraph.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_traffic_msgs/msg/test_task_request.hpp>
#include <rmf_msgs/msg/robot_state.hpp>

namespace {

//==============================================================================
struct Target
{
  std::string waypoint_name;
  Eigen::Vector2d location;
};

//==============================================================================
inline double wrap_to_pi(double value)
{
  while(value < -M_PI)
    value += 2.0*M_PI;

  while(M_PI < value)
    value -= 2.0*M_PI;

  return value;
}

//==============================================================================
class Looper
{
public:

  std::array<Target, 2> targets;
  std::size_t current_target_index = 0;

  rclcpp::Node& node;
  std::string fleet_name;

  double last_known_orientation = 0.0;

  using TestTaskRequest = rmf_traffic_msgs::msg::TestTaskRequest;
  using TestTaskRequestPub = rclcpp::Publisher<TestTaskRequest>;
  using TestTaskRequestHandle = TestTaskRequestPub::SharedPtr;
  TestTaskRequestHandle test_task_request_pub;

  using RobotState = rmf_msgs::msg::RobotState;
  using RobotStateSub = rclcpp::Subscription<RobotState>;
  using RobotStateHandle = RobotStateSub::SharedPtr;
  RobotStateHandle robot_state_sub;

  Looper(
      const std::string& initial_name,
      const std::string& start_name,
      const std::string& end_name,
      std::string _fleet_name,
      const std::string& graph_file,
      const rmf_traffic::agv::VehicleTraits& traits,
      rclcpp::Node& _node)
    : node(_node),
      fleet_name(_fleet_name)
  {
    rmf_traffic::agv::Graph graph;
    std::unordered_map<std::string, std::size_t> waypoint_keys;
    const bool made_graph = proto_fleet_adapter::parse_graph(
          graph_file, traits, node, graph, waypoint_keys);
    assert(made_graph);

    targets = {
      Target{start_name, graph.get_waypoint(waypoint_keys.at(start_name)).get_location()},
      Target{end_name, graph.get_waypoint(waypoint_keys.at(end_name)).get_location()}
    };

    test_task_request_pub = node.create_publisher<TestTaskRequest>(
          "test_task_request", rclcpp::SystemDefaultsQoS());

    robot_state_sub = node.create_subscription<RobotState>(
          "gazebo_robot_state", rclcpp::SystemDefaultsQoS(),
          [&](RobotState::UniquePtr msg)
    {
      if(msg->robot_name != fleet_name)
        return;

      const Eigen::Vector2d& current_target = get_current_target().location;

      const Eigen::Vector2d current_location =
          {msg->location.position.x, msg->location.position.y};

      const double dist = (current_target - current_location).norm();
//      RCLCPP_INFO(
//          node.get_logger(),
//          "[" + fleet_name + "] Current distance: " + std::to_string(dist));

      if( dist < 0.03 )
      {
        const auto q_msg = msg->location.orientation;
        const Eigen::Quaterniond q{q_msg.w, q_msg.x, q_msg.y, q_msg.z};
        last_known_orientation = wrap_to_pi(Eigen::AngleAxisd(q).angle());
        RCLCPP_INFO(
          node.get_logger(),
          "Last known orientation: " + std::to_string(last_known_orientation)
//          + " | angles: "
//          + std::to_string(angles[0]) + ", "
//          + std::to_string(angles[1]) + ", "
//          + std::to_string(angles[2])
//          + " | quaternion: "
//          + std::to_string(q.w()) + ", "
//          + std::to_string(q.x()) + ", "
//          + std::to_string(q.y()) + ", "
//          + std::to_string(q.z())
          );
        use_next_target();
      }
    });

    command_to_target(initial_name, start_name);
  }

  const Target& get_current_target() const
  {
    return targets[current_target_index];
  }

  void command_to_target(const std::string& from, const std::string& to)
  {
    TestTaskRequest msg;
    msg.fleet_name = fleet_name;
    msg.start_waypoint_name = from;
    msg.goal_waypoint_name = to;
    msg.initial_orientation = last_known_orientation;
    test_task_request_pub->publish(msg);
  }

  void use_next_target()
  {
    const std::string& last_wp = targets.at(current_target_index).waypoint_name;
    ++current_target_index;
    if(current_target_index >= targets.size())
      current_target_index = 0;

    const std::string& next_wp = targets.at(current_target_index).waypoint_name;

    RCLCPP_INFO(
          node.get_logger(),
          "Preparing next command for [" + fleet_name + "] to " + next_wp);

    std::thread delayed_command([&]()
    {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      RCLCPP_INFO(
            node.get_logger(),
            "Commanding [" + fleet_name + "] to next waypoint: " + next_wp);
      command_to_target(last_wp, next_wp);
    });

    delayed_command.detach();
  }

};
} // anonymous namespace

bool get_arg(
    const std::vector<std::string>& args,
    const std::string& key,
    std::string& value,
    const std::string& desc,
    const bool mandatory = true)
{
  const auto key_arg = std::find(args.begin(), args.end(), key);
  if(key_arg == args.end())
  {
    // TODO(MXG): See if there's a way to use RCLCPP_ERROR here without first
    // constructing a node. If not, we could consider constructing the
    // FleetAdapterNode in two parts.
    if(mandatory)
    {
      std::cerr << "You must specify a " << desc <<" using the " << key
                << " argument!" << std::endl;
    }
    return false;
  }
  else if(key_arg+1 == args.end())
  {
    std::cerr << "The " << key << " argument must be followed by a " << desc
              << "!" << std::endl;
    return false;
  }

  value = *(key_arg+1);
  return true;
}

int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string graph_file;
  if(!get_arg(args, "-g", graph_file, "graph file name"))
    return 1;

  std::string fleet_name;
  if(!get_arg(args, "-f", fleet_name, "fleet name"))
    return 1;

  std::string v_arg;
  get_arg(args, "-v", v_arg, "nominal velocity", false);
  const double v_nom = v_arg.empty()? 0.7 : std::stod(v_arg);

  std::string a_arg;
  get_arg(args, "-a", a_arg, "nominal acceleration", false);
  const double a_nom = a_arg.empty()? 0.5 : std::stod(a_arg);

  std::string r_arg;
  get_arg(args, "-r", r_arg, "tire radius", false);
  const double scaling = r_arg.empty()? 1.0 : std::stod(r_arg)/0.1;


  std::string start_wp;
  const bool has_start = get_arg(args, "-s", start_wp, "start waypoint", false);

  std::string initial_wp = start_wp;
  get_arg(args, "-i", initial_wp, "initial waypoint", false);

  std::string end_wp;
  const bool has_end = get_arg(args, "-e", end_wp, "end waypoint", false);

  if(has_start && !has_end)
  {
    std::cerr << "If you specify a start waypoint, you must also specify an "
              << "end waypoint!" << std::endl;
    return 1;
  }

  if(has_end && !has_start)
  {
    std::cerr << "If you specify an end waypoint, you must also specify a "
              << "start waypoint!";
    return 1;
  }

  // TODO(MXG): Parse arguments for specifying vehicle traits and profile
  // properties, or allow the user to pass a yaml file describing the properties
  auto profile = rmf_traffic::Trajectory::Profile::make_guided(
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(1.0));

  std::cout << "Vehicle traits: v: " << v_nom << " | a: " << a_nom << std::endl;
  rmf_traffic::agv::VehicleTraits traits{
    {v_nom*scaling, a_nom*scaling},
    {0.3*scaling, 1.5*scaling},
    profile
  };

  const auto fleet_adapter_node =
    proto_fleet_adapter::FleetAdapterNode::make(fleet_name, graph_file, traits);

  if(!fleet_adapter_node)
  {
    std::cerr << "Failed to initialize the fleet adapter node" << std::endl;
    return 1;
  }

  RCLCPP_INFO(
        fleet_adapter_node->get_logger(),
        "Beginning loop");

  if(!has_start)
  {
    rclcpp::spin(fleet_adapter_node);
  }
  else
  {
    Looper looper{initial_wp, start_wp, end_wp, fleet_name, graph_file,
          traits, *fleet_adapter_node};
    rclcpp::spin(fleet_adapter_node);
  }
  RCLCPP_INFO(
        fleet_adapter_node->get_logger(),
        "Closing down");

  rclcpp::shutdown();
}
