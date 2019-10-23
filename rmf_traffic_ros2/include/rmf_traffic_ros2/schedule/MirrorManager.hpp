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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__MIRROR_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__MIRROR_HPP

#include <rmf_traffic/schedule/Mirror.hpp>

#include <rclcpp/node.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
class MirrorManager
{
public:

  /// Options for the MirrorManager
  class Options
  {
  public:

    /// Constructor
    Options(
        bool update_on_wakeup = true,
        std::chrono::nanoseconds discovery_wait_time = std::chrono::seconds(10),
        std::chrono::nanoseconds service_wait_time = std::chrono::seconds(5));

    /// True if the mirror should be updated each time a MirrorWakeup message
    /// is received. The MirrorWakeup messages are sent out each time a change
    /// is introduced to the schedule database.
    bool get_update_on_wakeup() const;

    /// Toggle the choice to wakeup on an update.
    Options& set_update_on_wakeup(bool choice);

    /// How long to wait for a service to reply before giving up. This will be
    /// applied to every service call.
    std::chrono::nanoseconds get_service_wait_time() const;

    /// Set how long to wait for a service to reply before giving up.
    Options& set_service_wait_time(std::chrono::nanoseconds wait_time);

    /// How long to wait for a service to be discovered before giving up. This
    /// will be applied to every service discovery attempt.
    std::chrono::nanoseconds get_discovery_wait_time() const;

    /// Set how long to wait for discovery before giving up.
    Options& set_discovery_wait_time(std::chrono::nanoseconds wait_time);

    // TODO(MXG): Add a field to specify a mutex to protect the viewer

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Get the viewer of the mirror that is being managed
  const rmf_traffic::schedule::Viewer& viewer() const;

  /// Attempt to update this mirror immediately
  void update();

  /// Get the options for this mirror manager
  const Options& get_options() const;

  /// Set the options for this mirror manager
  MirrorManager& set_options(Options options);

  class Implementation;
private:
  MirrorManager();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// Create a mirror manager that uses the given node to communicate to the
/// RMF Traffic Schedule. It will filter its contents according to the
/// Spacetime Query description that it is given.
///
/// If the schedule database was not responsive while trying to register this
/// mirror's query, this function will return a nullptr.
///
/// \param[in] node
///   The rclcpp node to use
///
/// \param[in] spacetime
///   The spacetime description to filter the query
///
/// \param[in] options
///   Options to configure the management of the mirror
///
// TODO(MXG): Use std::optional here instead of std::unique_ptr when C++17 can
// be supported.
std::unique_ptr<MirrorManager> make_mirror(
    rclcpp::Node& node,
    const rmf_traffic::schedule::Query::Spacetime& spacetime,
    MirrorManager::Options options = MirrorManager::Options());

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__MIRROR_HPP
