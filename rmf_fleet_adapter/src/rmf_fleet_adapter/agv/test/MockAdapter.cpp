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
#include "../internal_TrafficLight.hpp"

#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_traffic_ros2/blockade/Writer.hpp>

namespace rmf_fleet_adapter {
namespace agv {
namespace test {

//==============================================================================
class MockAdapter::Implementation
{
public:

  rxcpp::schedulers::worker worker;
  std::shared_ptr<Node> node;
  std::shared_ptr<rmf_traffic::schedule::Database> database;

  // TODO(MXG): We should swap this out for a blockade Moderator that doesn't
  // rely on ROS2
  std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer;

  Implementation(
      const std::string& node_name,
      const rclcpp::NodeOptions& node_options)
    : worker{rxcpp::schedulers::make_event_loop().create_worker()},
      node{Node::make(worker, node_name, node_options)},
      database{std::make_shared<rmf_traffic::schedule::Database>()},
      blockade_writer{rmf_traffic_ros2::blockade::Writer::make(*node)}
  {
    // Do nothing
  }

  std::vector<std::shared_ptr<FleetUpdateHandle>> fleets = {};

};

//==============================================================================
MockAdapter::MockAdapter(
    const std::string& node_name,
    const rclcpp::NodeOptions& node_options)
  : _pimpl{rmf_utils::make_unique_impl<Implementation>(node_name, node_options)}
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<FleetUpdateHandle> MockAdapter::add_fleet(
    const std::string& fleet_name,
    rmf_traffic::agv::VehicleTraits traits,
    rmf_traffic::agv::Graph navigation_graph)
{
  auto planner =
    std::make_shared<std::shared_ptr<const rmf_traffic::agv::Planner>>(
      std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Planner::Configuration(
          std::move(navigation_graph),
          std::move(traits)),
        rmf_traffic::agv::Planner::Options(nullptr)));

  auto fleet = FleetUpdateHandle::Implementation::make(
        fleet_name, std::move(planner), _pimpl->node, _pimpl->worker,
        std::make_shared<SimpleParticipantFactory>(_pimpl->database),
        _pimpl->database, nullptr);

  _pimpl->fleets.push_back(fleet);
  return fleet;
}

//==============================================================================
TrafficLight::UpdateHandlePtr MockAdapter::add_traffic_light(
    std::shared_ptr<TrafficLight::CommandHandle> command,
    const std::string& fleet_name,
    const std::string& robot_name,
    rmf_traffic::agv::VehicleTraits traits,
    rmf_traffic::Profile profile)
{
  rmf_traffic::schedule::ParticipantDescription description(
      robot_name,
      fleet_name,
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile);

  auto participant = rmf_traffic::schedule::make_participant(
        std::move(description), _pimpl->database, nullptr);

  return TrafficLight::UpdateHandle::Implementation::make(
        std::move(command),
        std::move(participant),
        _pimpl->blockade_writer,
        std::move(traits),
        _pimpl->database,
        _pimpl->worker,
        _pimpl->node,
        nullptr);
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
void MockAdapter::dispatch_task(const rmf_task_msgs::msg::TaskProfile& profile)
{
  for (auto& fleet : _pimpl->fleets)
  {
    auto& fimpl = FleetUpdateHandle::Implementation::get(*fleet);
    if (!fimpl.accept_task)
      continue;

    // NOTE: althought the current adapter supports multiple fleets. The test
    // here assumses using a single fleet for each adapter
    rmf_task_msgs::msg::BidNotice bid;
    bid.task_profile = profile;
    fimpl.bid_notice_cb(
      std::make_shared<rmf_task_msgs::msg::BidNotice>(bid));
    
    rmf_task_msgs::msg::DispatchRequest req;
    req.task_profile = profile;
    req.fleet_name = fimpl.name;
    req.method = req.ADD;
    fimpl.dispatch_request_cb(
      std::make_shared<rmf_task_msgs::msg::DispatchRequest>(req));
  }
}

} // namespace test
} // namespace agv
} // namespace rmf_fleet_adapter
