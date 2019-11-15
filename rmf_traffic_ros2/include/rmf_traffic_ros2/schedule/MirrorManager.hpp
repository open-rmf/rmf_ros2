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
    Options(bool update_on_wakeup = true);

    /// True if the mirror should be updated each time a MirrorWakeup message
    /// is received. The MirrorWakeup messages are sent out each time a change
    /// is introduced to the schedule database.
    bool get_update_on_wakeup() const;

    /// Toggle the choice to wakeup on an update.
    Options& set_update_on_wakeup(bool choice);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  // TODO(MXG): Put a mutex on this
  /// Get the viewer of the mirror that is being managed
  const rmf_traffic::schedule::Viewer& viewer() const;

  /// Attempt to update this mirror immediately.
  ///
  /// \param[in] wait
  ///   How long to block the current thread while waiting for the mirror to
  ///   update. By default this will not block at all.
  ///
  // TODO(MXG): Consider allowing this function to accept a callback that will
  // get triggered when the update is complete.
  void update(rmf_traffic::Duration wait = rmf_traffic::Duration(0));

  /// Get the options for this mirror manager
  const Options& get_options() const;

  /// Set the options for this mirror manager
  MirrorManager& set_options(Options options);

  class Implementation;
private:
  MirrorManager();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class MirrorManagerFuture
{
public:

  /// Wait for the MirrorManager to finish initializing. This will block until
  /// initialization is finished.
  ///
  /// \note The initialization requires some ROS2 service calls, so there needs
  /// to be a separate thread running rclcpp::spin(~) while this function
  /// blocks. Otherwise it will block forever.
  void wait() const;

  /// Wait for the MirrorManager to finish initializing. This will block until
  /// initialization is finished or until the timeout duration has passed.
  ///
  /// \sa wait()
  std::future_status wait_for(const rmf_traffic::Duration& timeout) const;

  /// Wait for the MirrorManager to finish initializing. This will block until
  /// initialization is finished or until the time has been reached.
  ///
  /// \sa wait()
  std::future_status wait_until(const rmf_traffic::Time& time) const;

  /// Check if this MirrorManagerFuture is still valid. This means that get()
  /// has never been called.
  bool valid() const;

  /// Get the MirrorManager of this future. This will wait until the
  /// MirrorManager is initialized before returning it.
  MirrorManager get();

  class Implementation;
private:
  MirrorManagerFuture();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Initiate creation of a mirror manager that uses the given node to
/// communicate to  the RMF Traffic Schedule. It will filter its contents
/// according to the Spacetime Query description that it is given.
///
/// Creating a mirror manager involves some asynchronous service calls to
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
MirrorManagerFuture make_mirror(
    rclcpp::Node& node,
    rmf_traffic::schedule::Query::Spacetime spacetime,
    MirrorManager::Options options = MirrorManager::Options());

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__MIRROR_HPP
