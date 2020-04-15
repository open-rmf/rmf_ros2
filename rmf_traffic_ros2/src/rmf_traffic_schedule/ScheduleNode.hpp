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

#ifndef SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
#define SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP

#include "../rmf_traffic_ros2/schedule/NegotiationRoom.hpp"

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Negotiation.hpp>

#include <rclcpp/node.hpp>

#include <rmf_traffic_msgs/msg/mirror_wakeup.hpp>

#include <rmf_traffic_msgs/msg/itinerary_clear.hpp>
#include <rmf_traffic_msgs/msg/itinerary_delay.hpp>
#include <rmf_traffic_msgs/msg/itinerary_erase.hpp>
#include <rmf_traffic_msgs/msg/itinerary_extend.hpp>
#include <rmf_traffic_msgs/msg/itinerary_set.hpp>

#include <rmf_traffic_msgs/msg/schedule_conflict_ack.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_repeat.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_notice.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_proposal.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_rejection.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_conclusion.hpp>

#include <rmf_traffic_msgs/msg/schedule_inconsistency.hpp>

#include <rmf_traffic_msgs/srv/mirror_update.hpp>
#include <rmf_traffic_msgs/srv/register_query.hpp>
#include <rmf_traffic_msgs/srv/mirror_update.h>
#include <rmf_traffic_msgs/srv/unregister_query.hpp>
#include <rmf_traffic_msgs/srv/register_participant.hpp>
#include <rmf_traffic_msgs/srv/unregister_participant.hpp>

#include <rmf_traffic_msgs/msg/schedule_conflict_notice.hpp>

#include <rmf_utils/Modular.hpp>

#include <set>
#include <unordered_map>

namespace rmf_traffic_schedule {

//==============================================================================
class ScheduleNode : public rclcpp::Node
{
public:

  ScheduleNode();

  ~ScheduleNode();

  using request_id_ptr = std::shared_ptr<rmw_request_id_t>;

  using RegisterQuery = rmf_traffic_msgs::srv::RegisterQuery;
  using RegisterQueryService = rclcpp::Service<RegisterQuery>;

  void register_query(
    const request_id_ptr& request_header,
    const RegisterQuery::Request::SharedPtr& request,
    const RegisterQuery::Response::SharedPtr& response);

  RegisterQueryService::SharedPtr register_query_service;


  using UnregisterQuery = rmf_traffic_msgs::srv::UnregisterQuery;
  using UnregisterQueryService = rclcpp::Service<UnregisterQuery>;

  void unregister_query(
    const request_id_ptr& request_header,
    const UnregisterQuery::Request::SharedPtr& request,
    const UnregisterQuery::Response::SharedPtr& response);

  UnregisterQueryService::SharedPtr unregister_query_service;


  using RegisterParticipant = rmf_traffic_msgs::srv::RegisterParticipant;
  using RegisterParticipantSrv = rclcpp::Service<RegisterParticipant>;

  void register_participant(
    const request_id_ptr& request_header,
    const RegisterParticipant::Request::SharedPtr& request,
    const RegisterParticipant::Response::SharedPtr& response);

  RegisterParticipantSrv::SharedPtr register_participant_service;


  using UnregisterParticipant = rmf_traffic_msgs::srv::UnregisterParticipant;
  using UnregisterParticipantSrv = rclcpp::Service<UnregisterParticipant>;

  void unregister_participant(
    const request_id_ptr& request_header,
    const UnregisterParticipant::Request::SharedPtr& request,
    const UnregisterParticipant::Response::SharedPtr& response);

  UnregisterParticipantSrv::SharedPtr unregister_participant_service;


  using MirrorUpdate = rmf_traffic_msgs::srv::MirrorUpdate;
  using MirrorUpdateService = rclcpp::Service<MirrorUpdate>;

  void mirror_update(
    const request_id_ptr& request_header,
    const MirrorUpdate::Request::SharedPtr& request,
    const MirrorUpdate::Response::SharedPtr& response);

  MirrorUpdateService::SharedPtr mirror_update_service;


  using MirrorWakeup = rmf_traffic_msgs::msg::MirrorWakeup;
  using MirrorWakeupPublisher = rclcpp::Publisher<MirrorWakeup>;
  MirrorWakeupPublisher::SharedPtr mirror_wakeup_publisher;


  using ScheduleConflictNotice = rmf_traffic_msgs::msg::ScheduleConflictNotice;
  using ScheduleConflictNoticePublisher =
    rclcpp::Publisher<ScheduleConflictNotice>;
  ScheduleConflictNoticePublisher::SharedPtr conflict_publisher;


  using ItinerarySet = rmf_traffic_msgs::msg::ItinerarySet;
  void itinerary_set(const ItinerarySet& set);
  rclcpp::Subscription<ItinerarySet>::SharedPtr itinerary_set_sub;

  using ItineraryExtend = rmf_traffic_msgs::msg::ItineraryExtend;
  void itinerary_extend(const ItineraryExtend& extend);
  rclcpp::Subscription<ItineraryExtend>::SharedPtr itinerary_extend_sub;

  using ItineraryDelay = rmf_traffic_msgs::msg::ItineraryDelay;
  void itinerary_delay(const ItineraryDelay& delay);
  rclcpp::Subscription<ItineraryDelay>::SharedPtr itinerary_delay_sub;

  using ItineraryErase = rmf_traffic_msgs::msg::ItineraryErase;
  void itinerary_erase(const ItineraryErase& erase);
  rclcpp::Subscription<ItineraryErase>::SharedPtr itinerary_erase_sub;

  using ItineraryClear = rmf_traffic_msgs::msg::ItineraryClear;
  void itinerary_clear(const ItineraryClear& clear);
  rclcpp::Subscription<ItineraryClear>::SharedPtr itinerary_clear_sub;

  using InconsistencyMsg = rmf_traffic_msgs::msg::ScheduleInconsistency;
  rclcpp::Publisher<InconsistencyMsg>::SharedPtr inconsistency_pub;
  void publish_inconsistencies(rmf_traffic::schedule::ParticipantId id);

  void wakeup_mirrors();

  // TODO(MXG): Consider using libguarded instead of a database_mutex
  std::mutex database_mutex;
  rmf_traffic::schedule::Database database;

  using QueryMap =
    std::unordered_map<uint64_t, rmf_traffic::schedule::Query>;
  // TODO(MXG): Have a way to make query registrations expire after they have
  // not been used for some set amount of time (e.g. 24 hours? 48 hours?).
  std::size_t last_query_id = 0;
  QueryMap registered_queries;

  // TODO(MXG): Make this a separate node
  std::thread conflict_check_thread;
  std::condition_variable conflict_check_cv;
  std::atomic_bool conflict_check_quit;

  using ConflictAck = rmf_traffic_msgs::msg::ScheduleConflictAck;
  using ConflictAckSub = rclcpp::Subscription<ConflictAck>;
  ConflictAckSub::SharedPtr conflict_ack_sub;
  void receive_conclusion_ack(const ConflictAck& msg);

  using ConflictNotice = rmf_traffic_msgs::msg::ScheduleConflictNotice;
  using ConflictNoticePub = rclcpp::Publisher<ConflictNotice>;
  ConflictNoticePub::SharedPtr conflict_notice_pub;

  using ConflictProposal = rmf_traffic_msgs::msg::ScheduleConflictProposal;
  using ConflictProposalSub = rclcpp::Subscription<ConflictProposal>;
  ConflictProposalSub::SharedPtr conflict_proposal_sub;
  void receive_proposal(const ConflictProposal& msg);

  using ConflictRejection = rmf_traffic_msgs::msg::ScheduleConflictRejection;
  using ConflictRejectionSub = rclcpp::Subscription<ConflictRejection>;
  ConflictRejectionSub::SharedPtr conflict_rejection_sub;
  void receive_rejection(const ConflictRejection& msg);

  using ConflictConclusion = rmf_traffic_msgs::msg::ScheduleConflictConclusion;
  using ConflictConclusionPub = rclcpp::Publisher<ConflictConclusion>;
  ConflictConclusionPub::SharedPtr conflict_conclusion_pub;

  using Version = rmf_traffic::schedule::Version;
  using ItineraryVersion = rmf_traffic::schedule::ItineraryVersion;
  using ParticipantId = rmf_traffic::schedule::ParticipantId;
  using ConflictSet = std::unordered_set<ParticipantId>;

  using Negotiation = rmf_traffic::schedule::Negotiation;

  class ConflictRecord
  {
  public:

    using NegotiationRoom = rmf_traffic_ros2::schedule::NegotiationRoom;
    using Entry = std::pair<Version, const Negotiation*>;
    struct Wait
    {
      Version negotiation_version;
      rmf_utils::optional<ItineraryVersion> itinerary_update_version;
    };

    ConflictRecord(const rmf_traffic::schedule::Viewer& viewer)
    : _viewer(viewer)
    {
      // Do nothing
    }

    rmf_utils::optional<Entry> insert(const ConflictSet& conflicts)
    {
      ConflictSet add_to_negotiation;
      const Version* existing_negotiation = nullptr;
      for (const auto c : conflicts)
      {
        const auto wait_it = _waiting.find(c);
        if (wait_it != _waiting.end())
        {
          // Ignore conflicts with participants that we're waiting for an update
          // from. Otherwise we might announce conflicts for them faster than
          // they can respond to them.
          return rmf_utils::nullopt;
        }

        const auto it = _version.find(c);
        if (it != _version.end())
        {
          const Version* const it_conflict = &it->second;
          if (existing_negotiation && *existing_negotiation != *it_conflict)
            continue;

          existing_negotiation = it_conflict;
        }
        else
        {
          add_to_negotiation.insert(c);
        }
      }

      if (add_to_negotiation.empty())
        return rmf_utils::nullopt;

      const Version negotiation_version = existing_negotiation ?
        *existing_negotiation : _next_negotiation_version++;

      const auto insertion = _negotiations.insert(
        std::make_pair(negotiation_version, rmf_utils::nullopt));

      for (const auto p : add_to_negotiation)
        _version[p] = negotiation_version;

      auto& update_negotiation = insertion.first->second;
      if (!update_negotiation)
      {
        update_negotiation = rmf_traffic::schedule::Negotiation(
          _viewer, std::vector<ParticipantId>(
            add_to_negotiation.begin(), add_to_negotiation.end()));
      }
      else
      {
        for (const auto p : add_to_negotiation)
          update_negotiation->negotiation.add_participant(p);
      }

      return Entry{negotiation_version, &update_negotiation->negotiation};
    }

    NegotiationRoom* negotiation(const Version version)
    {
      const auto it = _negotiations.find(version);
      if (it == _negotiations.end())
        return nullptr;

      assert(it->second);

      return &(*it->second);
    }

    void conclude(const Version version)
    {
      const auto negotiation_it = _negotiations.find(version);
      if (negotiation_it == _negotiations.end())
        return;

      const auto& participants =
        negotiation_it->second->negotiation.participants();

      for (const auto p : participants)
      {
        const auto insertion =
          _waiting.insert({p, Wait{version, rmf_utils::nullopt}});

        assert(insertion.second);
        (void)(insertion);
        _version.erase(p);
      }

      _negotiations.erase(negotiation_it);
    }

    // Tell the ConflictRecord what ItineraryVersion will resolve this
    // negotiation.
    void acknowledge(
      const Version negotiation_version,
      const ParticipantId p,
      const rmf_utils::optional<ItineraryVersion> update_version)
    {
      const auto wait_it = _waiting.find(p);
      if (wait_it == _waiting.end())
      {
        assert(false);
        // TODO(MXG): We should probably output some warning here using
        // RCLCPP_WARN
        std::cout << "[ScheduleNode::ConflictRecord::acknowledge] We are NOT "
                  << "waiting for an acknowledgment from participant [" << p
                  << "]" << std::endl;
        return;
      }

      const auto expected_negotiation = wait_it->second.negotiation_version;
      if (expected_negotiation != negotiation_version)
      {
        assert(false);
        // TODO(MXG): We should probably output some warning here using
        // RCLCPP_WARN
        std::cout << "[ScheduleNode::ConflictRecord::acknowledge] We are "
                  << "waiting for an acknowledgment from participant ["
                  << p << "] regarding negotiation [" << expected_negotiation
                  << "] but received an acknowledgment for negotiation ["
                  << negotiation_version << "] instead." << std::endl;
        return;
      }

      if (update_version)
        wait_it->second.itinerary_update_version = *update_version;
      else
        _waiting.erase(wait_it);
    }

    void check(const ParticipantId p, const ItineraryVersion version)
    {
      const auto wait_it = _waiting.find(p);
      if (wait_it == _waiting.end())
        return;

      const auto expected_version = wait_it->second.itinerary_update_version;
      if (!expected_version)
        return;

      if (rmf_utils::modular(*expected_version).less_than_or_equal(version))
      {
        _waiting.erase(wait_it);
      }
    }

//  private:
    std::unordered_map<ParticipantId, Version> _version;
    std::unordered_map<Version,
      rmf_utils::optional<NegotiationRoom>> _negotiations;
    std::unordered_map<ParticipantId, Wait> _waiting;
    const rmf_traffic::schedule::Viewer& _viewer;
    Version _next_negotiation_version = 0;
  };

  ConflictRecord active_conflicts;
  std::mutex active_conflicts_mutex;
};

} // namespace rmf_traffic_schedule

#endif // SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
