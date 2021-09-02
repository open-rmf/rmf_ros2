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

#ifndef RMF_TRAFFIC_ROS2__BLOCKADE__WRITER_HPP
#define RMF_TRAFFIC_ROS2__BLOCKADE__WRITER_HPP

#include <rmf_traffic/blockade/Participant.hpp>

#include <rclcpp/node.hpp>

namespace rmf_traffic_ros2 {
namespace blockade {

//==============================================================================
/// The Writer class provides an API that allows a Node to create blockade
/// Participants.
class Writer : public std::enable_shared_from_this<Writer>
{
public:

  /// Create an instance of a writer. The writer and all Participants it creates
  /// depend on the life of the rclcpp::Node. It's best to keep all of these as
  /// members of the Node.
  ///
  /// \param[in] node
  ///   The node that will manage the subscriptions of this writer.
  static std::shared_ptr<Writer> make(rclcpp::Node& node);

  using ReservedRange = rmf_traffic::blockade::ReservedRange;
  using ReservationId = rmf_traffic::blockade::ReservationId;
  using NewRangeCallback =
    std::function<void(
        const ReservationId reservation,
        const ReservedRange& range)>;

  /// Make a blockade participant.
  ///
  /// \param[in] id
  ///   The ID of the participant that is being created. This must match the
  ///   schedule ParticipantId. All blockade participants must also be schedule
  ///   participants to comply with the RMF traffic protocol.
  ///
  /// \param[in] radius
  ///   The radius around the path that the participant will occupy.
  ///
  /// \param[in] new_range_cb
  ///   This callback will get triggered when a new range arrives.
  ///
  /// \return the API for updating the blockade Participant.
  rmf_traffic::blockade::Participant make_participant(
    rmf_traffic::blockade::ParticipantId id,
    double radius,
    NewRangeCallback new_range_cb);

  class Implementation;
private:
  Writer(rclcpp::Node& node);
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using WriterPtr = std::shared_ptr<Writer>;

} // namespace blockade
} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__BLOCKADE__WRITER_HPP
