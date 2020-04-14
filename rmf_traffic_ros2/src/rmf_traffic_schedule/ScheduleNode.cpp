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

#include "ScheduleNode.hpp"

#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>
#include <rmf_traffic_ros2/schedule/Patch.hpp>
#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/schedule/Inconsistencies.hpp>

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/schedule/Mirror.hpp>

#include <rmf_utils/optional.hpp>

#include <unordered_map>

namespace rmf_traffic_schedule {

//==============================================================================
std::vector<ScheduleNode::ConflictSet> get_conflicts(
    const rmf_traffic::schedule::Viewer::View& view_changes,
    const rmf_traffic::schedule::Viewer& viewer)
{
  std::vector<ScheduleNode::ConflictSet> conflicts;
  const auto& participants = viewer.participant_ids();
  for (const auto participant : participants)
  {
    const auto itinerary = *viewer.get_itinerary(participant);
    const auto& description = *viewer.get_participant(participant);
    for (auto vc = view_changes.begin(); vc != view_changes.end(); ++vc)
    {
      if (vc->participant == participant)
      {
        // There's no need to check a participant against itself
        continue;
      }

      for (const auto& route : itinerary)
      {
        assert(route);
        if (route->map() != vc->route.map())
          continue;

        if (rmf_traffic::DetectConflict::between(
              vc->description.profile(),
              vc->route.trajectory(),
              description.profile(),
              route->trajectory()))
        {
          conflicts.push_back({participant, vc->participant});
        }
      }
    }
  }

  return conflicts;
}

//==============================================================================
ScheduleNode::ScheduleNode()
  : Node("rmf_traffic_schedule_node"),
    active_conflicts(database)
{
  // TODO(MXG): As soon as possible, all of these services should be made
  // multi-threaded so they can be parallel processed.

  register_query_service =
      create_service<RegisterQuery>(
        rmf_traffic_ros2::RegisterQueryServiceName,
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const RegisterQuery::Request::SharedPtr request,
            const RegisterQuery::Response::SharedPtr response)
        { this->register_query(request_header, request, response); });

  unregister_query_service =
      create_service<UnregisterQuery>(
        rmf_traffic_ros2::UnregisterQueryServiceName,
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const UnregisterQuery::Request::SharedPtr request,
            const UnregisterQuery::Response::SharedPtr response)
        { this->unregister_query(request_header, request, response); });

  register_participant_service =
      create_service<RegisterParticipant>(
        rmf_traffic_ros2::RegisterParticipantSrvName,
        [=](const request_id_ptr request_header,
            const RegisterParticipant::Request::SharedPtr request,
            const RegisterParticipant::Response::SharedPtr response)
        { this->register_participant(request_header, request, response); });

  unregister_participant_service =
      create_service<UnregisterParticipant>(
        rmf_traffic_ros2::UnregisterParticipantSrvName,
        [=](const request_id_ptr request_header,
            const UnregisterParticipant::Request::SharedPtr request,
            const UnregisterParticipant::Response::SharedPtr response)
        { this->unregister_participant(request_header, request, response); });

  mirror_update_service =
      create_service<MirrorUpdate>(
        rmf_traffic_ros2::MirrorUpdateServiceName,
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const MirrorUpdate::Request::SharedPtr request,
            const MirrorUpdate::Response::SharedPtr response)
        { this->mirror_update(request_header, request, response); });

  mirror_wakeup_publisher =
      create_publisher<MirrorWakeup>(
        rmf_traffic_ros2::MirrorWakeupTopicName,
        rclcpp::SystemDefaultsQoS());

  conflict_publisher =
      create_publisher<ScheduleConflictNotice>(
        rmf_traffic_ros2::ScheduleConflictNoticeTopicName,
        rclcpp::SystemDefaultsQoS());

  itinerary_set_sub =
      create_subscription<ItinerarySet>(
        rmf_traffic_ros2::ItinerarySetTopicName,
        rclcpp::SystemDefaultsQoS().best_effort(),
        [=](const ItinerarySet::UniquePtr msg)
  {
    this->itinerary_set(*msg);
  });

  itinerary_extend_sub =
      create_subscription<ItineraryExtend>(
        rmf_traffic_ros2::ItineraryExtendTopicName,
        rclcpp::SystemDefaultsQoS().best_effort(),
        [=](const ItineraryExtend::UniquePtr msg)
  {
    this->itinerary_extend(*msg);
  });

  itinerary_delay_sub =
      create_subscription<ItineraryDelay>(
        rmf_traffic_ros2::ItineraryDelayTopicName,
        rclcpp::SystemDefaultsQoS().best_effort(),
        [=](const ItineraryDelay::UniquePtr msg)
  {
    this->itinerary_delay(*msg);
  });

  itinerary_erase_sub =
      create_subscription<ItineraryErase>(
        rmf_traffic_ros2::ItineraryEraseTopicName,
        rclcpp::SystemDefaultsQoS().best_effort(),
        [=](const ItineraryErase::UniquePtr msg)
  {
    this->itinerary_erase(*msg);
  });

  itinerary_clear_sub =
      create_subscription<ItineraryClear>(
        rmf_traffic_ros2::ItineraryClearTopicName,
        rclcpp::SystemDefaultsQoS().best_effort(),
        [=](const ItineraryClear::UniquePtr msg)
  {
    this->itinerary_clear(*msg);
  });

  inconsistency_pub =
      create_publisher<InconsistencyMsg>(
        rmf_traffic_ros2::ScheduleInconsistencyTopicName,
        rclcpp::SystemDefaultsQoS().reliable());

  const auto negotiation_qos = rclcpp::ServicesQoS().reliable();
  conflict_ack_sub = create_subscription<ConflictAck>(
        rmf_traffic_ros2::ScheduleConflictAckTopicName, negotiation_qos,
        [&](const ConflictAck::UniquePtr msg)
  {
    this->receive_conclusion_ack(*msg);
  });

  conflict_notice_pub = create_publisher<ConflictNotice>(
        rmf_traffic_ros2::ScheduleConflictNoticeTopicName, negotiation_qos);

  conflict_proposal_sub = create_subscription<ConflictProposal>(
        rmf_traffic_ros2::ScheduleConflictProposalTopicName, negotiation_qos,
        [&](const ConflictProposal::UniquePtr msg)
  {
    this->receive_proposal(*msg);
  });

  conflict_rejection_sub = create_subscription<ConflictRejection>(
        rmf_traffic_ros2::ScheduleConflictRejectionTopicName, negotiation_qos,
        [&](const ConflictRejection::UniquePtr msg)
  {
    this->receive_rejection(*msg);
  });

  conflict_conclusion_pub = create_publisher<ConflictConclusion>(
        rmf_traffic_ros2::ScheduleConflictConclusionTopicName, negotiation_qos);

  conflict_check_quit = false;
  conflict_check_thread = std::thread(
        [&]()
  {
    rmf_traffic::schedule::Mirror mirror;
    const auto query_all = rmf_traffic::schedule::query_all();
    Version last_checked_version = 0;

    while (rclcpp::ok() && !conflict_check_quit)
    {
      rmf_utils::optional<rmf_traffic::schedule::Patch> next_patch;
      rmf_traffic::schedule::Viewer::View view_changes;

      // Use this scope to minimize how long we lock the database for
      {
        std::unique_lock<std::mutex> lock(database_mutex);
        conflict_check_cv.wait_for(lock, std::chrono::milliseconds(100), [&]()
        {
          return (database.latest_version() > last_checked_version)
              && !conflict_check_quit;
        });

        if (database.latest_version() == last_checked_version
            || conflict_check_quit)
        {
          // This is a casual wakeup to check if we're supposed to quit yet
          continue;
        }

        next_patch = database.changes(query_all, last_checked_version);

        // TODO(MXG): Check whether the database really needs to remain locked
        // during this update.
        try
        {
          mirror.update(*next_patch);
          view_changes = database.query(query_all, last_checked_version);
          last_checked_version = next_patch->latest_version();
        }
        catch(const std::exception& e)
        {
          RCLCPP_ERROR(get_logger(), e.what());
          continue;
        }
      }

      const auto conflicts = get_conflicts(view_changes, mirror);
      std::unordered_map<Version, const Negotiation*> new_negotiations;
      for (const auto& conflict : conflicts)
      {
        std::unique_lock<std::mutex> lock(active_conflicts_mutex);
        const auto new_negotiation = active_conflicts.insert(conflict);

        if (new_negotiation)
          new_negotiations[new_negotiation->first] = new_negotiation->second;
      }

      for (const auto& n : new_negotiations)
      {
        ConflictNotice msg;
        msg.conflict_version = n.first;

        const auto& participants = n.second->participants();
        msg.participants = ConflictNotice::_participants_type(
              participants.begin(), participants.end());

        conflict_notice_pub->publish(msg);
      }
    }
  });
}

//==============================================================================
ScheduleNode::~ScheduleNode()
{
  conflict_check_quit = true;
  if (conflict_check_thread.joinable())
    conflict_check_thread.join();
}

//==============================================================================
void ScheduleNode::register_query(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const RegisterQuery::Request::SharedPtr& request,
    const RegisterQuery::Response::SharedPtr& response)
{
  uint64_t query_id = last_query_id;
  uint64_t attempts = 0;
  do
  {
    ++query_id;
    ++attempts;
    if(attempts == std::numeric_limits<uint64_t>::max())
    {
      // I suspect a computer would run out of RAM before we reach this point,
      // but there's no harm in double-checking.
      response->error = "No more space for additional queries to be registered";
      RCLCPP_ERROR(
            get_logger(),
            "[ScheduleNode::register_query] " + response->error);
      return;
    }
  } while(registered_queries.find(query_id) != registered_queries.end());

  last_query_id = query_id;
  registered_queries.insert(
        std::make_pair(query_id, rmf_traffic_ros2::convert(request->query)));

  response->query_id = query_id;
  RCLCPP_INFO(
        get_logger(),
        "[" + std::to_string(query_id) + "] Registered query");
}

//==============================================================================
void ScheduleNode::unregister_query(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const UnregisterQuery::Request::SharedPtr& request,
    const UnregisterQuery::Response::SharedPtr& response)
{
  const auto it = registered_queries.find(request->query_id);
  if(it == registered_queries.end())
  {
    response->error = "No query found with the id ["
        + std::to_string(request->query_id) + "]";
    response->confirmation = false;

    RCLCPP_WARN(
          get_logger(),
          "[ScheduleNode::unregister_query] " + response->error);
    return;
  }

  registered_queries.erase(it);
  response->confirmation = true;

  RCLCPP_INFO(
        get_logger(),
        "[" + std::to_string(request->query_id) + "] Unregistered query");
}

//==============================================================================
void ScheduleNode::register_participant(
    const request_id_ptr& /*request_header*/,
    const RegisterParticipant::Request::SharedPtr& request,
    const RegisterParticipant::Response::SharedPtr& response)
{
  std::unique_lock<std::mutex> lock(database_mutex);

  // TODO(MXG): Use try on every database operation
  try
  {
    response->participant_id = database.register_participant(
          rmf_traffic_ros2::convert(request->description));

    RCLCPP_INFO(
          get_logger(),
          "Registered participant [" + std::to_string(response->participant_id)
          + "] named [" + request->description.name + "] owned by ["
          + request->description.owner + "]");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
          get_logger(),
          "Failed to register participant [" + request->description.name
          + "] owned by [" + request->description.owner + "]:" + e.what());
    response->error = e.what();
  }
}

//==============================================================================
void ScheduleNode::unregister_participant(
    const request_id_ptr& /*request_header*/,
    const UnregisterParticipant::Request::SharedPtr& request,
    const UnregisterParticipant::Response::SharedPtr& response)
{
  std::unique_lock<std::mutex> lock(database_mutex);

  const auto& p = database.get_participant(request->participant_id);
  if (!p)
  {
    response->error =
        "Failed to unregister participant ["
        + std::to_string(request->participant_id) + "] because no "
        "participant has that ID";
    response->confirmation = false;

    RCLCPP_ERROR(get_logger(), response->error);
    return;
  }

  try
  {
    // We need to copy this data before the participant is unregistered, because
    // unregistering it will invalidate the pointer p.
    const std::string name = p->name();
    const std::string owner = p->owner();

    database.unregister_participant(request->participant_id);
    response->confirmation = true;

    RCLCPP_INFO(
          get_logger(),
          "Unregistered participant [" + std::to_string(request->participant_id)
          +"] named [" + name + "] owned by [" + owner + "]");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
          get_logger(),
          "Failed to unregister participant ["
          + std::to_string(request->participant_id) + "]:" + e.what());
    response->error = e.what();
    response->confirmation = false;
  }
}

//==============================================================================
void ScheduleNode::mirror_update(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const MirrorUpdate::Request::SharedPtr& request,
    const MirrorUpdate::Response::SharedPtr& response)
{
  const auto query_it = registered_queries.find(request->query_id);
  if(query_it == registered_queries.end())
  {
    response->error = "Unrecognized query_id: "
        + std::to_string(request->query_id);
    RCLCPP_WARN(
          get_logger(),
          "[ScheduleNode::mirror_update] " + response->error);
    return;
  }

  rmf_utils::optional<rmf_traffic::schedule::Version> version;
  if (!request->initial_request)
    version = request->latest_mirror_version;

  response->patch =
      rmf_traffic_ros2::convert(database.changes(query_it->second, version));
}

//==============================================================================
void ScheduleNode::itinerary_set(const ItinerarySet& set)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  database.set(
        set.participant,
        rmf_traffic_ros2::convert(set.itinerary),
        set.itinerary_version);

  publish_inconsistencies(set.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(set.participant, set.itinerary_version);
  wakeup_mirrors();
}

//==============================================================================
void ScheduleNode::itinerary_extend(const ItineraryExtend& extend)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  database.extend(
        extend.participant,
        rmf_traffic_ros2::convert(extend.routes),
        extend.itinerary_version);

  publish_inconsistencies(extend.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(
        extend.participant, database.itinerary_version(extend.participant));
  wakeup_mirrors();
}

//==============================================================================
void ScheduleNode::itinerary_delay(const ItineraryDelay& delay)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  database.delay(
        delay.participant,
        rmf_traffic::Time(rmf_traffic::Duration(delay.from_time)),
        rmf_traffic::Duration(delay.delay),
        delay.itinerary_version);

  publish_inconsistencies(delay.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(
        delay.participant, database.itinerary_version(delay.participant));
  wakeup_mirrors();
}

//==============================================================================
void ScheduleNode::itinerary_erase(const ItineraryErase& erase)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  database.erase(
        erase.participant,
        std::vector<rmf_traffic::RouteId>(
          erase.routes.begin(), erase.routes.end()),
        erase.itinerary_version);

  publish_inconsistencies(erase.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(
        erase.participant, database.itinerary_version(erase.participant));
  wakeup_mirrors();
}

//==============================================================================
void ScheduleNode::itinerary_clear(const ItineraryClear& clear)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  database.erase(clear.participant, clear.itinerary_version);

  publish_inconsistencies(clear.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(
        clear.participant, database.itinerary_version(clear.participant));
  wakeup_mirrors();
}

//==============================================================================
void ScheduleNode::publish_inconsistencies(
    rmf_traffic::schedule::ParticipantId id)
{
  // TODO(MXG): This approach is likely to send out a lot of redundant
  // inconsistency reports. We should try to be smarter about how
  // inconsistencies get reported.
  const auto it = database.inconsistencies().find(id);
  assert(it != database.inconsistencies().end());
  if (it->ranges.size() == 0)
    return;

  inconsistency_pub->publish(rmf_traffic_ros2::convert(*it));
}

//==============================================================================
void ScheduleNode::wakeup_mirrors()
{
  rmf_traffic_msgs::msg::MirrorWakeup msg;
  msg.latest_version = database.latest_version();
  mirror_wakeup_publisher->publish(msg);

  conflict_check_cv.notify_all();
}

//==============================================================================
void print_conclusion(
    const std::unordered_map<
        ScheduleNode::Version, ScheduleNode::ConflictRecord::Wait>& _awaiting)
{
  // TODO(MXG): Instead of printing this conclusion information to the terminal,
  // we should periodically output a heartbeat with metadata on the current
  // negotiation status so that other systems can keep their negotiation caches
  // clean.
  struct Status
  {
    rmf_traffic::schedule::ParticipantId participant;
    bool known;
  };

  std::unordered_map<ScheduleNode::Version, std::vector<Status>> negotiations;
  for (const auto& entry : _awaiting)
  {
    negotiations[entry.second.negotiation_version]
        .push_back({entry.first, entry.second.itinerary_update_version.has_value()});
  }

  std::cout << "\n --- Awaiting acknowledgment of conclusions:";
  for (const auto& entry : negotiations)
  {
    std::cout << "\n   - [" << entry.first << "]:";
    for (const auto p : entry.second)
    {
      std::cout << " ";
      if (p.known)
        std::cout << "<";
      std::cout << p.participant;
      if (p.known)
        std::cout << ">";
    }
  }
  std::cout << "\n" << std::endl;
}

//==============================================================================
void ScheduleNode::receive_conclusion_ack(const ConflictAck& msg)
{
  std::unique_lock<std::mutex> lock(active_conflicts_mutex);

  for (const auto ack : msg.acknowledgments)
  {
    if (ack.updating)
    {
      active_conflicts.acknowledge(
            msg.conflict_version, ack.participant, ack.itinerary_version);
    }
    else
    {
      active_conflicts.acknowledge(
            msg.conflict_version, ack.participant, rmf_utils::nullopt);
    }
  }

//  print_conclusion(active_conflicts._waiting);
}

//==============================================================================
void ScheduleNode::receive_proposal(const ConflictProposal& msg)
{
  std::unique_lock<std::mutex> lock(active_conflicts_mutex);
  auto* negotiation_room =
      active_conflicts.negotiation(msg.conflict_version);

  if (!negotiation_room)
  {
    RCLCPP_WARN(
          get_logger(),
          "Received proposal for unknown negotiation ["
          + std::to_string(msg.conflict_version) + "]");
    return;
  }

  auto& negotiation = negotiation_room->negotiation;

  const auto table = negotiation.table(msg.for_participant, msg.to_accommodate);
  negotiation_room->check_cache();

  if (!table)
  {
    std::string error = "Received proposal in negotiation ["
        + std::to_string(msg.conflict_version) + "] for participant ["
        + std::to_string(msg.for_participant) + "] on unknown table [";
    for (const auto p : msg.to_accommodate)
      error += " " + std::to_string(p) + " ";
    error += "]";

    RCLCPP_WARN(get_logger(), error);
    negotiation_room->cached_proposals.push_back(msg);
    return;
  }

  table->submit(rmf_traffic_ros2::convert(msg.itinerary), msg.proposal_version);
  negotiation_room->check_cache();

  // TODO(MXG): This should be removed once we have a negotiation visualizer
  rmf_traffic_ros2::schedule::print_negotiation_status(msg.conflict_version, negotiation);

  if (negotiation.ready())
  {
    // TODO(MXG): If the negotiation is not complete yet, give some time for
    // more proposals to arrive before choosing one.
    const auto choose =
        negotiation.evaluate(rmf_traffic::schedule::QuickestFinishEvaluator());
    assert(choose);

    active_conflicts.conclude(msg.conflict_version);

    ConflictConclusion conclusion;
    conclusion.conflict_version = msg.conflict_version;
    conclusion.resolved = true;
    conclusion.table = choose->sequence();

    std::string output = "Resolved negotiation ["
        + std::to_string(msg.conflict_version) + ":";

    for (const auto p : conclusion.table)
      output += " " + std::to_string(p);
    RCLCPP_INFO(get_logger(), output);

    conflict_conclusion_pub->publish(std::move(conclusion));
//    print_conclusion(active_conflicts._waiting);
  }
  else if (negotiation.complete())
  {
    std::cout << " -- REJECTED" << std::endl;

    active_conflicts.conclude(msg.conflict_version);

    // This implies a complete failure
    ConflictConclusion conclusion;
    conclusion.conflict_version = msg.conflict_version;
    conclusion.resolved = false;

    conflict_conclusion_pub->publish(conclusion);
//    print_conclusion(active_conflicts._waiting);
  }
}

//==============================================================================
void ScheduleNode::receive_rejection(const ConflictRejection& msg)
{
  std::unique_lock<std::mutex> lock(active_conflicts_mutex);
  auto* negotiation_room = active_conflicts.negotiation(msg.conflict_version);

  if (!negotiation_room)
  {
    RCLCPP_WARN(
          get_logger(),
          "Received rejection for unknown negotiation ["
          + std::to_string(msg.conflict_version) + "]");
    return;
  }

  auto& negotiation = negotiation_room->negotiation;

  const auto table = negotiation.table(msg.table);
  if (!table)
  {
    std::string error = "Received rejection in negotiation ["
        + std::to_string(msg.conflict_version) + "] for unknown table [";
    for (const auto p : msg.table)
      error += " " + std::to_string(p) + " ";
    error += "]";

    RCLCPP_WARN(get_logger(), error);
    negotiation_room->cached_rejections.push_back(msg);
    return;
  }

  table->reject(msg.proposal_version);
  negotiation_room->check_cache();

  // TODO(MXG): This should be removed once we have a negotiation visualizer
  rmf_traffic_ros2::schedule::print_negotiation_status(msg.conflict_version, negotiation);

  if (negotiation.complete())
  {
    active_conflicts.conclude(msg.conflict_version);

    ConflictConclusion conclusion;
    conclusion.conflict_version = msg.conflict_version;
    conclusion.resolved = false;

    conflict_conclusion_pub->publish(conclusion);
//    print_conclusion(active_conflicts._waiting);
  }
}

} // namespace rmf_traffic_schedule
