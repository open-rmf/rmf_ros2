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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__WRITER_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__WRITER_HPP

#include <rmf_traffic/schedule/Writer.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <rclcpp/node.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
/// The Writer class provides an API that allows a Node to create schedule
/// Participants.
class Writer : public std::enable_shared_from_this<Writer>
{
public:

  /// Create an instance of a writer. The writer and all Participants it creates
  /// depend on the life of the rclcpp::Node. It's best to keep all of these as
  /// members of the Node.
  ///
  /// \param[in] node
  ///   The node that will manage the subscriptions of this writer. This will be
  ///   held as a `std::weak_ptr<rclcpp::Node>` so it is okay to store the
  ///   writer inside the node itself.
  static std::shared_ptr<Writer> make(
    const std::shared_ptr<rclcpp::Node>& node);

  /// Returns true if all the services needed by this writer are ready.
  bool ready() const;

  /// Wait for the necessary services to be available.
  void wait_for_service() const;

  /// Wait for the necessary services to be available, or for the time point to
  /// be reached, whichever happens first.
  ///
  /// \param[in] stop
  ///   The maximum time point that this will wait until
  ///
  /// \return true if the necessary services are now available, false otherwise.
  bool wait_for_service(rmf_traffic::Time stop) const;

  /// Begin creation of a schedule participant.
  ///
  /// The node of this Writer needs to be spun in order for the Participant to
  /// finish being created.
  ///
  /// \param[in] description.
  ///   The description of the participant.
  std::future<rmf_traffic::schedule::Participant> make_participant(
    rmf_traffic::schedule::ParticipantDescription description);

  /// Asynchronously create a schedule participant.
  ///
  /// When the Participant is ready to be used, the ready_callback will be
  /// triggered with the newly created Participant instance.
  ///
  /// \param[in] description
  ///   The description of the participant.
  ///
  /// \param[in] ready_callback
  ///   The callback that will be triggered when the participant is ready.
  void async_make_participant(
    rmf_traffic::schedule::ParticipantDescription description,
    std::function<void(rmf_traffic::schedule::Participant)> ready_callback);

  class Implementation;
private:
  Writer();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using WriterPtr = std::shared_ptr<Writer>;

} // namespace schedule

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__WRITER_HPP
