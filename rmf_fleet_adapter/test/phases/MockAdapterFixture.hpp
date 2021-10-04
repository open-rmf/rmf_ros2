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
  struct Data
  {
    std::shared_ptr<agv::Node> node;
    std::shared_ptr<rclcpp::Node> ros_node;
    rmf_traffic::agv::Graph graph;

    static std::size_t _node_counter;
    std::shared_ptr<rclcpp::Context> _context;

    std::shared_ptr<agv::test::MockAdapter> adapter;
    std::shared_ptr<agv::FleetUpdateHandle> fleet;
  };

  std::shared_ptr<Data> data;

  struct RobotInfo
  {
    std::shared_ptr<agv::RobotContext> context;
    std::shared_ptr<rmf_fleet_adapter_test::MockRobotCommand> command;

    // Sometimes a test needs some operation to happen on the worker so it syncs
    // correctly with other spinning operations, but also the test can't proceed
    // until that operation is finished. This function conveniently wraps up
    // that use case.
    template<typename T>
    T schedule_and_wait(std::function<T()> job) const
    {
      std::promise<T> promise;
      auto future = promise.get_future();
      context->worker().schedule(
        [&promise, job = std::move(job)](const auto&)
        {
          promise.set_value(job());
        });

      return future.get();
    }
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
};

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter

#endif // TEST__PHASES__TRANSPORTFIXTURE_HPP
