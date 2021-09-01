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

  using TableViewPtr = rmf_traffic::schedule::Negotiation::Table::ViewerPtr;
  using ResponderPtr = rmf_traffic::schedule::Negotiator::ResponderPtr;
  using StatusUpdateCallback =
    std::function<void (uint64_t conflict_version, TableViewPtr table_view)>;

  /// Register a callback with this Negotiation manager that triggers
  /// on negotiation status updates.
  ///
  /// \param[in] cb
  ///   The callback function to be called upon status updates.
  void on_status_update(StatusUpdateCallback cb);

  using StatusConclusionCallback =
    std::function<void (uint64_t conflict_version, bool success)>;

  /// Register a callback with this Negotiation manager that triggers
  /// on negotiation status conclusions.
  ///
  /// \param[in] cb
  ///   The callback function to be called upon status conclusions.
  void on_conclusion(StatusConclusionCallback cb);

  /// Get a Negotiation::TableView that provides a view into what participants are
  /// proposing.
  ///
  /// This function does not care about table versioning.
  /// \param[in] conflict_version
  ///   The conflict version of the negotiation
  /// \param[in] sequence
  ///   The sequence of participant ids. Follows convention of other sequences
  ///   (ie. The last ParticipantId is the owner of the table)
  ///
  /// \return A TableView into what participants are proposing.
  TableViewPtr table_view(
    uint64_t conflict_version,
    const std::vector<rmf_traffic::schedule::ParticipantId>& sequence) const;

  /// Set the number of negotiations to retain.
  ///
  /// \param[in] count
  ///   The number of negotiations to retain
  void set_retained_history_count(uint count);

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

  /// Register a negotiator with this Negotiation manager.
  ///
  /// \param[in] for_participant
  ///   The ID of the participant that this negotiator will work for
  ///
  /// \param[in] negotiator
  ///   The negotiator interface to use for this participant
  ///
  /// \param[in] on_negotiation_failure
  ///   A callback that will be triggered if a negotiation for this participant
  ///   fails
  ///
  /// \return a handle that should be kept by the caller. When this handle
  /// expires, this negotiator will be automatically unregistered.
  std::shared_ptr<void> register_negotiator(
    rmf_traffic::schedule::ParticipantId for_participant,
    std::unique_ptr<rmf_traffic::schedule::Negotiator> negotiator,
    std::function<void()> on_negotiation_failure);

  /// Register a negotiator with this Negotiation manager using a lambda.
  ///
  /// \param[in] for_participant
  ///   The ID of the participant that this negotiator will work for
  ///
  /// \param[in] respond
  ///   The callback that will be used as the negotiator's response
  ///
  /// \param[in] on_negotiation_failure
  ///   A callback that will be triggered if a negotiation for this participant
  ///   fails
  ///
  /// \return a handle that should be kept by the caller. When this handle
  /// expires, this negotiator will be automatically unregistered.
  std::shared_ptr<void> register_negotiator(
    rmf_traffic::schedule::ParticipantId for_participant,
    std::function<void(TableViewPtr view, ResponderPtr responder)> respond,
    std::function<void()> on_negotiation_failure = nullptr);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP
