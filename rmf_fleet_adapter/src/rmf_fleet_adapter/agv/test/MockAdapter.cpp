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
#include <rmf_traffic/schedule/Mirror.hpp>

#include <rmf_traffic_ros2/blockade/Writer.hpp>

namespace rmf_fleet_adapter {
namespace agv {
namespace test {

//==============================================================================
/// This class ensures that race conditions don't happen when writing to the
/// schedule. This is important for the tests because the Participant objects
/// will automatically unregister themselves during destruction, and that
/// destruction may occur on any thread.
///
/// The most important behavior of this class is to redirect calls on
/// unregister_participant() into the primary worker queue.
///
/// We also redirect all the other function calls into the worker queue, except
/// for register_participant() because it expects a return value which makes
/// it tricky to schedule. None of these other redirects should matter since the
/// tests should already be scheduling all these calls on the primary worker,
/// but the redirects shouldn't hurt either, so we'll go ahead and do it to
/// minimize the risk of race conditions. If we ever see another race condition
/// on the schedule database, then we should look into queuing the
/// register_participant() function since it should be the last remaining
/// vulnerability.
class MockScheduleNode : public rmf_traffic::schedule::Writer
{
public:

  MockScheduleNode(rxcpp::schedulers::worker worker)
  : _worker(worker),
    _database(std::make_shared<rmf_traffic::schedule::Database>())
  {
    // Do nothing
  }

  std::shared_ptr<rmf_traffic::schedule::Snappable> snappable() const
  {
    return _database;
  }

  void set(
    ParticipantId participant,
    const Input& itinerary,
    ItineraryVersion version) final
  {
    _worker.schedule(
      [
        database = _database,
        participant,
        itinerary,
        version
      ](const auto&)
      {
        database->set(participant, itinerary, version);
      });
  }

  void extend(
    ParticipantId participant,
    const Input& routes, ItineraryVersion version) final
  {
    _worker.schedule(
      [
        database = _database,
        participant,
        routes,
        version
      ](const auto&)
      {
        database->extend(participant, routes, version);
      });
  }

  void delay(
    ParticipantId participant,
    Duration delay,
    ItineraryVersion version) final
  {
    _worker.schedule(
      [
        database = _database,
        participant,
        delay,
        version
      ](const auto&)
      {
        database->delay(participant, delay, version);
      });
  }

  void erase(
    ParticipantId participant,
    ItineraryVersion version) final
  {
    _worker.schedule(
      [
        database = _database,
        participant, version
      ](const auto&)
      {
        database->erase(participant, version);
      });
  }

  void erase(
    ParticipantId participant,
    const std::vector<RouteId>& routes,
    ItineraryVersion version) final
  {
    _worker.schedule(
      [
        database = _database,
        participant,
        routes,
        version
      ](const auto&)
      {
        database->erase(participant, routes, version);
      });
  }

  Registration register_participant(
    ParticipantDescription participant_info) final
  {
    // To simplify the async return value, we'll assume that this function only
    // gets called from the primary worker thread.
    //
    // This should be a fair assumption since the main concern about race
    // conditions is for the unregister_participant() function that gets called
    // by the destructor of the Participant class. We probably don't really need
    // to explicitly use the worker for any of the functions besides
    // unregister_participant() but it isn't difficult when they have void
    // return types (unlike this function).
    return _database->register_participant(participant_info);
  }

  void unregister_participant(ParticipantId participant) final
  {
    _worker.schedule(
      [database = _database, participant](const auto&)
      {
        database->unregister_participant(participant);
      });
  }

  void update_description(
    ParticipantId participant,
    ParticipantDescription desc) final
  {
    _worker.schedule(
      [database = _database, participant, desc](const auto&)
      {
        database->update_description(participant, desc);
      });
  }

private:

  rxcpp::schedulers::worker _worker;
  std::shared_ptr<rmf_traffic::schedule::Database> _database;

};

//==============================================================================
class MockAdapter::Implementation
{
public:

  rxcpp::schedulers::worker worker;
  std::shared_ptr<Node> node;
  std::shared_ptr<MockScheduleNode> schedule;

  // TODO(MXG): We should swap this out for a blockade Moderator that doesn't
  // rely on ROS2
  std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer;

  Implementation(
    const std::string& node_name,
    const rclcpp::NodeOptions& node_options)
  : worker{rxcpp::schedulers::make_event_loop().create_worker()},
    node{Node::make(worker, node_name, node_options)},
    schedule{std::make_shared<MockScheduleNode>(worker)},
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
    std::make_shared<SimpleParticipantFactory>(_pimpl->schedule),
    _pimpl->schedule->snappable(), nullptr);

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
    std::move(description), _pimpl->schedule, nullptr);

  return TrafficLight::UpdateHandle::Implementation::make(
    std::move(command),
    std::move(participant),
    _pimpl->blockade_writer,
    std::move(traits),
    _pimpl->schedule->snappable(),
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

//==============================================================================
MockAdapter::~MockAdapter()
{
  stop();
}

} // namespace test
} // namespace agv
} // namespace rmf_fleet_adapter
