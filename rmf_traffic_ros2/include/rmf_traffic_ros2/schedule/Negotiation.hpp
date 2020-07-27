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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP

#include <rclcpp/node.hpp>

#include <rmf_traffic/schedule/Negotiator.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Snapshot.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
/// A ROS2 interface for negotiating solutions to schedule conflicts
class Negotiation
{
public:

  /// The Worker class can be used to make the Negotiation asynchronous.
  class Worker
  {
  public:

    /// Tell the worker to add a callback to its schedule. It is imperative that
    /// this function is thread-safe.
    virtual void schedule(std::function<void()> job) = 0;

    virtual ~Worker() = default;
  };

  /// Constructor
  ///
  /// \param[in] worker
  ///   If a worker is provided, the Negotiation will be performed
  ///   asynchronously. If it is not provided, then the Negotiators must be
  ///   single-threaded, and their respond() functions must block until
  ///   finished.
  Negotiation(
    rclcpp::Node& node,
    std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer,
    std::shared_ptr<Worker> worker = nullptr);

  /// Set the timeout duration for negotiators. If a negotiator does not respond
  /// within this time limit, then the negotiation will automatically be
  /// forfeited. This is important to prevent negotiations from getting hung
  /// forever.
  Negotiation& timeout_duration(rmf_traffic::Duration duration);

  /// Get the current timeout duration setting.
  rmf_traffic::Duration timeout_duration() const;

  /// Register a negotiator with this Negotiation manager.
  ///
  /// \param[in] for_participant
  ///   The ID of the participant that this negotiator will work for
  ///
  /// \param[in] negotiator
  ///   The negotiator interface to use for this participant
  ///
  /// \return a handle that should be kept by the caller. When this handle
  /// expires, this negotiator will be automatically unregistered.
  std::shared_ptr<void> register_negotiator(
    rmf_traffic::schedule::ParticipantId for_participant,
    std::unique_ptr<rmf_traffic::schedule::Negotiator> negotiator);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP
