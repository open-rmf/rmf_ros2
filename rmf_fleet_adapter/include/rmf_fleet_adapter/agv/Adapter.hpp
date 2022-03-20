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
#include <rmf_fleet_adapter/agv/EasyTrafficLight.hpp>

#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <rclcpp/node.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Adapter : public std::enable_shared_from_this<Adapter>
{
public:

  /// Initialize an rclcpp context and make an adapter instance. This will
  /// instantiate an rclcpp::Node and allow you to add fleets to be adapted.
  ///
  /// This is an easier-to-use but less customizable alternative to make().
  ///
  /// \param[in] node_name
  ///   The name for the rclcpp::Node that will be produced for this Adapter.
  ///
  /// \param[in] discovery_timeout
  ///   How long we will wait to discover the Schedule Node before giving up. If
  ///   rmf_utils::nullopt is given, then this will try to use the
  ///   discovery_timeout node paramter, or it will wait 1 minute if the
  ///   discovery_timeout node parameter was not defined.
  ///
  /// \sa make()
  static std::shared_ptr<Adapter> init_and_make(
    const std::string& node_name,
    std::optional<rmf_traffic::Duration> discovery_timeout = std::nullopt);

  /// Make an adapter instance. This will instantiate an rclcpp::Node and allow
  /// you to add fleets to be adapted.
  ///
  /// \note You must initialize rclcpp before calling this, either by using
  /// rclcpp::init(~) or rclcpp::Context::init(~). This requirement can be
  /// avoided by using init_and_make() instead of this function.
  ///
  /// \param[in] node_name
  ///   The name for the rclcpp::Node that will be produced for this Adapter.
  ///
  /// \param[in] node_options
  ///   The options that the rclcpp::Node will be constructed with.
  ///
  /// \param[in] discovery_timeout
  ///   How long we will wait to discover the Schedule Node before giving up. If
  ///   rmf_utils::nullopt is given, then this will try to use the
  ///   discovery_timeout node paramter, or it will wait 1 minute if the
  ///   discovery_timeout node parameter was not defined.
  ///
  /// \sa init_and_make()
  static std::shared_ptr<Adapter> make(
    const std::string& node_name,
    const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions(),
    std::optional<rmf_traffic::Duration> discovery_timeout = std::nullopt);

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
  ///
  /// \param[in] server_uri
  ///   Specify the URI for the websocket server that receives updates on tasks
  ///   and states. If nullopt, data will not be published.
  std::shared_ptr<FleetUpdateHandle> add_fleet(
    const std::string& fleet_name,
    rmf_traffic::agv::VehicleTraits traits,
    rmf_traffic::agv::Graph navigation_graph,
    std::optional<std::string> server_uri = std::nullopt);

  using Blockers = std::vector<EasyTrafficLight::Blocker>;

  /// Create an easy-to-use version of a traffic light to help manage robots
  /// that can only support pause and resume commands.
  ///
  /// This API is much simpler to use than the standard traffic light API, but
  /// it provides less information about the exact timing needed for the starts
  /// and stops.
  ///
  /// This API should only be used for demo purposes, or if system integrators
  /// can ensure very low-latency and reliable connections to the robots to
  /// ensure that the commands arrive on time.
  ///
  /// \param[in] handle_callback
  ///   The callback that will be triggered when the EasyTrafficLight handle is
  ///   ready to be used. This callback will only be triggered once.
  ///
  /// \param[in] fleet_name
  ///   The name of the fleet
  ///
  /// \param[in] robot_name
  ///   The name of the robot
  ///
  /// \param[in] traits
  ///   The traits of the robot
  ///
  /// \param[in] pause_callback
  ///   The callback that should be triggered by the traffic light when an
  ///   immediate pause is needed.
  ///
  /// \param[in] resume_callback
  ///   The callback that will be triggered by the traffic light when the robot
  ///   may resume moving forward.
  ///
  /// \param[in] deadlock_callback
  ///   The callback that will be triggered by the traffic light if there is a
  ///   permanent blocker disrupting the ability of this vehicle to proceed.
  ///   Manual intervention may be required in this circumstance. A callback
  ///   does not need to be provided for this. Either way, an error message will
  ///   be printed to the log.
  void add_easy_traffic_light(
    std::function<void(EasyTrafficLightPtr handle)> handle_callback,
    const std::string& fleet_name,
    const std::string& robot_name,
    rmf_traffic::agv::VehicleTraits traits,
    std::function<void()> pause_callback,
    std::function<void()> resume_callback,
    std::function<void(Blockers)> deadlock_callback = nullptr);


  /// Get the rclcpp::Node that this adapter will be using for communication.
  std::shared_ptr<rclcpp::Node> node();

  /// const-qualified node()
  std::shared_ptr<const rclcpp::Node> node() const;

  /// Begin running the event loop for this adapter. The event loop will operate
  /// in another thread, so this function is non-blocking.
  Adapter& start();

  /// Stop the event loop if it is running.
  Adapter& stop();

  /// Wait until the adapter is done spinning.
  ///
  /// \sa wait_for()
  Adapter& wait();

  /// Wait until the adapter is done spinning, or until the maximum wait time
  /// duration is reached.
  ///
  /// \sa wait()
  Adapter& wait_for(std::chrono::nanoseconds max_wait);

  class Implementation;
private:
  Adapter();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using AdapterPtr = std::shared_ptr<Adapter>;
using ConstAdapterPtr = std::shared_ptr<const Adapter>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__ADAPTER_HPP
