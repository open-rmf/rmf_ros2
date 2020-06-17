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

#ifndef TEST__PHASES__TRANSPORTFIXTURE_HPP
#define TEST__PHASES__TRANSPORTFIXTURE_HPP

#include <rmf_rxcpp/Transport.hpp>

#include <agv/RobotContext.hpp>
#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>

#include "../mock/MockRobotCommand.hpp"

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

struct MockAdapterFixture
{
  std::shared_ptr<agv::test::MockAdapter> adapter;
  std::shared_ptr<agv::FleetUpdateHandle> fleet;
  std::shared_ptr<agv::Node> node;
  std::shared_ptr<rclcpp::Node> ros_node;
  rmf_traffic::agv::Graph graph;

  struct RobotInfo
  {
    std::shared_ptr<agv::RobotContext> context;
    std::shared_ptr<rmf_fleet_adapter_test::MockRobotCommand> command;
  };

  /// Add a robot for testing purposes and get its context
  ///
  /// \param[in] name
  ///   The name for this robot.
  ///
  /// \param[in] profile
  ///   Specify its profile. Leaving this as nullopt will use default profile.
  RobotInfo add_robot(
      const std::string& name = "test_robot",
      rmf_utils::optional<rmf_traffic::Profile> profile = rmf_utils::nullopt);

  MockAdapterFixture();

  ~MockAdapterFixture();

private:

  static std::size_t _node_counter;
  std::shared_ptr<rclcpp::Context> _context;
};

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter

#endif // TEST__PHASES__TRANSPORTFIXTURE_HPP
