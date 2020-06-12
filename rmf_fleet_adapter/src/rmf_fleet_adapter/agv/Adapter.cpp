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

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include "Node.hpp"
#include "internal_FleetUpdateHandle.hpp"

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic_ros2/schedule/Writer.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Adapter::Implementation
{
public:

  rxcpp::schedulers::worker worker;
  std::shared_ptr<Node> node;
  std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation;
  std::shared_ptr<ParticipantFactory> writer;
  rmf_traffic_ros2::schedule::MirrorManager mirror_manager;


  using Delivery = rmf_task_msgs::msg::Delivery;
  using DeliverySub = rclcpp::Subscription<Delivery>::SharedPtr;
  DeliverySub delivery_sub;

  using Loop = rmf_task_msgs::msg::Loop;
  using LoopSub = rclcpp::Subscription<Loop>::SharedPtr;
  LoopSub loop_sub;

  std::vector<std::shared_ptr<FleetUpdateHandle>> fleets = {};

  Implementation(
      rxcpp::schedulers::worker worker_,
      std::shared_ptr<Node> node_,
      std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation_,
      std::shared_ptr<ParticipantFactory> writer_,
      rmf_traffic_ros2::schedule::MirrorManager mirror_manager_)
    : worker{std::move(worker_)},
      node{std::move(node_)},
      negotiation{std::move(negotiation_)},
      writer{std::move(writer_)},
      mirror_manager{std::move(mirror_manager_)}
  {
    const auto default_qos = rclcpp::SystemDefaultsQoS();
    delivery_sub = node->create_subscription<Delivery>(
          DeliveryTopicName, default_qos,
          [this](Delivery::SharedPtr msg)
    {
      rmf_fleet_adapter::agv::request_delivery(*msg, fleets);
    });

    loop_sub = node->create_subscription<Loop>(
          LoopRequestTopicName, default_qos,
          [this](Loop::SharedPtr msg)
    {
      rmf_fleet_adapter::agv::request_loop(*msg, fleets);
    });
  }

  static rmf_utils::unique_impl_ptr<Implementation> make(
      const std::string& node_name,
      const rclcpp::NodeOptions& node_options,
      const rmf_traffic::Duration wait_time)
  {
    auto node = Node::make(node_name, node_options);

    auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
          *node, rmf_traffic::schedule::query_all());

    auto writer = rmf_traffic_ros2::schedule::Writer::make(*node);

    using namespace std::chrono_literals;

    const auto stop_time = std::chrono::steady_clock::now() + wait_time;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
    {
      rclcpp::spin_some(node);

      bool ready = true;
      ready &= writer->ready();
      ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

      if (ready)
      {
        auto mirror_manager = mirror_future.get();

        auto negotiation =
            std::make_shared<rmf_traffic_ros2::schedule::Negotiation>(
              *node, mirror_manager.snapshot_handle());

        return rmf_utils::make_unique_impl<Implementation>(
              Implementation{
                rxcpp::schedulers::make_event_loop().create_worker(),
                std::move(node),
                std::move(negotiation),
                std::make_shared<ParticipantFactoryRos2>(std::move(writer)),
                std::move(mirror_manager)
              });
      }
    }

    return nullptr;
  }
};

//==============================================================================
std::shared_ptr<Adapter> Adapter::make(
    const std::string& node_name,
    const rclcpp::NodeOptions& node_options,
    const rmf_traffic::Duration wait_time)
{
  Adapter adapter;
  adapter._pimpl = Implementation::make(node_name, node_options, wait_time);

  if (adapter._pimpl)
    return std::make_shared<Adapter>(std::move(adapter));

  return nullptr;
}

//==============================================================================
std::shared_ptr<FleetUpdateHandle> Adapter::add_fleet(
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
        _pimpl->writer, _pimpl->mirror_manager.snapshot_handle(),
        _pimpl->negotiation);

  _pimpl->fleets.push_back(fleet);
  return fleet;
}

//==============================================================================
std::shared_ptr<rclcpp::Node> Adapter::node()
{
  return _pimpl->node;
}

//==============================================================================
std::shared_ptr<const rclcpp::Node> Adapter::node() const
{
  return _pimpl->node;
}

//==============================================================================
void Adapter::start()
{
  _pimpl->node->start();
}

//==============================================================================
void Adapter::stop()
{
  _pimpl->node->stop();
}

//==============================================================================
Adapter::Adapter()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
