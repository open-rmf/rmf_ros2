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

#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>

#include "../internal_FleetUpdateHandle.hpp"

#include <rmf_traffic/schedule/Database.hpp>

namespace rmf_fleet_adapter {
namespace agv {
namespace test {

class MockAdapter::Implementation
{
public:

  rxcpp::schedulers::worker worker;
  std::shared_ptr<Node> node;
  std::shared_ptr<rmf_traffic::schedule::Database> database;

  std::vector<std::shared_ptr<FleetUpdateHandle>> fleets = {};

};

//==============================================================================
MockAdapter::MockAdapter(
    const std::string& node_name,
    const rclcpp::NodeOptions& node_options)
  : _pimpl{
      rmf_utils::make_unique_impl<Implementation>(
        Implementation{
          rxcpp::schedulers::make_event_loop().create_worker(),
          Node::make(node_name, node_options),
          std::make_shared<rmf_traffic::schedule::Database>()
        })
    }
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<FleetUpdateHandle> MockAdapter::add_fleet(
    const std::string& fleet_name,
    rmf_traffic::agv::VehicleTraits traits,
    rmf_traffic::agv::Graph navigation_graph)
{
  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Planner::Configuration(
          std::move(navigation_graph),
          std::move(traits)),
        rmf_traffic::agv::Planner::Options(nullptr));

  auto fleet = FleetUpdateHandle::Implementation::make(
        fleet_name, std::move(planner), _pimpl->node, _pimpl->worker,
        std::make_shared<SimpleParticipantFactory>(_pimpl->database),
        _pimpl->database, nullptr);

  _pimpl->fleets.push_back(fleet);
  return fleet;
}

//==============================================================================
std::shared_ptr<rclcpp::Node> MockAdapter::node()
{
  return _pimpl->node;
}

//==============================================================================
std::shared_ptr<const rclcpp::Node> MockAdapter::node() const
{
  return _pimpl->node;
}

//==============================================================================
void MockAdapter::start()
{
  _pimpl->node->start();
}

//==============================================================================
void MockAdapter::stop()
{
  _pimpl->node->stop();
}

//==============================================================================
void MockAdapter::request_delivery(const rmf_task_msgs::msg::Delivery& request)
{
  rmf_fleet_adapter::agv::request_delivery(request, _pimpl->fleets);
}

} // namespace test
} // namespace agv
} // namespace rmf_fleet_adapter
