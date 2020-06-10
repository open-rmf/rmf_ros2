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

#ifndef RMF_FLEET_ADAPTER__AGV__ADAPTER_HPP
#define RMF_FLEET_ADAPTER__AGV__ADAPTER_HPP

#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>

#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <rclcpp/node.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Adapter : public std::enable_shared_from_this<Adapter>
{
public:

  /// Make an adapter instance. This will instantiate an rclcpp::Node and allow
  /// you to add fleets to be adapted.
  ///
  /// \param[in] node_name
  ///   The name for the rclcpp::Node that will be produced for this Adapter.
  ///
  /// \param[in] node_options
  ///   The options that the rclcpp::Node will be constructed with.
  ///
  /// \param[in] wait_time
  ///   How long we will wait to discover the Schedule Node before giving up.
  static std::shared_ptr<Adapter> make(
      const std::string& node_name,
      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions(),
      rmf_traffic::Duration wait_time = std::chrono::minutes(1));

  /// Add a fleet to be adapted.
  ///
  /// If a single real-life fleet needs to integrate robots with varying traits
  /// or with different navigation graphs, it is okay to call this function
  /// multiple times with the same fleet_name and add a robot using whichever
  /// handle has the traits and navigation graph that match the robot.
  ///
  /// \param[in] fleet_name
  ///   The name of the fleet that is being added.
  ///
  /// \param[in] traits
  ///   Specify the approximate traits of the vehicles in this fleet.
  ///
  /// \param[in] navigation_graph
  ///   Specify the navigation graph used by the vehicles in this fleet.
  std::shared_ptr<FleetUpdateHandle> add_fleet(
      const std::string& fleet_name,
      rmf_traffic::agv::VehicleTraits traits,
      rmf_traffic::agv::Graph navigation_graph);

  /// Get the rclcpp::Node that this adapter will be using for communication.
  std::shared_ptr<rclcpp::Node> node();

  /// const-qualified node()
  std::shared_ptr<const rclcpp::Node> node() const;

  /// Begin running the event loop for this adapter. The event loop will operate
  /// in another thread, so this function is non-blocking.
  void start();

  /// Stop the event loop if it is running.
  //
  // TODO(MXG): Return a std::future from this to indicate when the thread has
  // come to a stop.
  void stop();

  class Implementation;
private:
  Adapter();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__ADAPTER_HPP
