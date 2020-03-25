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
  using ScheduleConflictNoticePublisher = rclcpp::Publisher<ScheduleConflictNotice>;
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

  using Version = rmf_traffic::schedule::Version;
  using ParticipantId = rmf_traffic::schedule::ParticipantId;
  struct ConflictPair
  {
    ParticipantId p1;
    ParticipantId p2;

    ParticipantId first() const
    {
      return std::min(p1, p2);
    }

    ParticipantId second() const
    {
      return std::max(p1, p2);
    }
  };

  class ConflictRecord
  {
  public:

    struct Entry
    {
      ConflictPair pair;
      rmf_traffic::schedule::Negotiation negotiation;
      bool concluded = false;
      std::unordered_set<ParticipantId> removal_request;
    };

    ConflictRecord(const rmf_traffic::schedule::Viewer& viewer)
      : _viewer(viewer)
    {
      // Do nothing
    }

    const Version* insert(const ConflictPair& pair)
    {
      auto& first = _record[pair.first()];
      const auto inserted = first.insert(pair.second());

      if (!inserted.second)
        return nullptr;

      std::vector<ParticipantId> participants;
      for (const auto p : {pair.p1, pair.p2})
      {
        if (_viewer.get_participant(p)->responsiveness()
            == rmf_traffic::schedule::ParticipantDescription::Rx::Responsive)
          participants.push_back(p);
      }

      const auto it =
          _entries.insert(
            std::make_pair(
              _next_conflict_version++,
              Entry{pair, rmf_traffic::schedule::Negotiation(
                    _viewer, std::move(participants))}));

      return &it.first->first;
    }

    rmf_traffic::schedule::Negotiation* negotiation(const Version version)
    {
      const auto it = _entries.find(version);
      if (it == _entries.end())
        return nullptr;

      if (it->second.concluded)
        return nullptr;

      return &it->second.negotiation;
    }

    void conclude(const Version version)
    {
      const auto entry_it = _entries.find(version);
      if (entry_it == _entries.end())
        return;

      entry_it->second.concluded = true;
    }

    void acknowledge(const Version version, const ParticipantId p)
    {
      const auto entry_it = _entries.find(version);
      if (entry_it == _entries.end())
        return;

      auto& entry = entry_it->second;
      assert(entry.concluded);

      // TODO(MXG): I should check to make sure that p is actually a member of
      // this conflict
      entry.removal_request.insert(p);

      if (entry.removal_request.size() == entry.negotiation.participants().size())
      {
        const auto& pair = entry.pair;
        _record[pair.first()].erase(pair.second());
        _entries.erase(entry_it);
      }
    }

  private:
    std::unordered_map<ParticipantId, std::unordered_set<ParticipantId>> _record;

    std::unordered_map<Version, Entry> _entries;
    const rmf_traffic::schedule::Viewer& _viewer;
    Version _next_conflict_version = 0;
  };

  ConflictRecord active_conflicts;
  std::mutex active_conflicts_mutex;

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
};

} // namespace rmf_traffic_schedule

#endif // SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
