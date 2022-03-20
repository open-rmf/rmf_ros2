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

#include "NegotiationRoom.hpp"

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Negotiation.hpp>

#include <rclcpp/node.hpp>

#include <rmf_traffic_msgs/msg/mirror_update.hpp>
#include <rmf_traffic_msgs/msg/participant.hpp>
#include <rmf_traffic_msgs/msg/participants.hpp>
#include <rmf_traffic_msgs/msg/schedule_queries.hpp>

#include <rmf_traffic_msgs/msg/itinerary_clear.hpp>
#include <rmf_traffic_msgs/msg/itinerary_delay.hpp>
#include <rmf_traffic_msgs/msg/itinerary_extend.hpp>
#include <rmf_traffic_msgs/msg/itinerary_set.hpp>
#include <rmf_traffic_msgs/msg/itinerary_reached.hpp>

#include <rmf_traffic_msgs/msg/negotiation_ack.hpp>
#include <rmf_traffic_msgs/msg/negotiation_repeat.hpp>
#include <rmf_traffic_msgs/msg/negotiation_notice.hpp>
#include <rmf_traffic_msgs/msg/negotiation_refusal.hpp>
#include <rmf_traffic_msgs/msg/negotiation_forfeit.hpp>
#include <rmf_traffic_msgs/msg/negotiation_proposal.hpp>
#include <rmf_traffic_msgs/msg/negotiation_rejection.hpp>
#include <rmf_traffic_msgs/msg/negotiation_conclusion.hpp>

#include <rmf_traffic_msgs/msg/schedule_inconsistency.hpp>

#include <rmf_traffic_msgs/msg/heartbeat.hpp>

#include <rmf_traffic_msgs/srv/register_query.hpp>
#include <rmf_traffic_msgs/srv/request_changes.hpp>
#include <rmf_traffic_msgs/srv/register_participant.hpp>
#include <rmf_traffic_msgs/srv/unregister_participant.hpp>

#include <rmf_traffic_msgs/msg/negotiation_notice.hpp>

#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

#include <rmf_utils/Modular.hpp>

#include <optional>
#include <set>
#include <unordered_map>
#include <utility>

namespace rmf_traffic_ros2 {
namespace schedule {

using namespace std::chrono_literals;

//==============================================================================
class ScheduleNode : public rclcpp::Node
{
public:
  using QueryMap = std::unordered_map<uint64_t, rmf_traffic::schedule::Query>;
  using VersionOpt = std::optional<rmf_traffic::schedule::Version>;

  using NodeVersion = uint64_t;
  NodeVersion node_version = 0;

  static struct NoAutomaticSetup{} no_automatic_setup;

  ScheduleNode(
    NodeVersion node_version_,
    std::shared_ptr<rmf_traffic::schedule::Database> database_,
    const rclcpp::NodeOptions& options,
    NoAutomaticSetup);

  ScheduleNode(
    NodeVersion node_version_,
    std::shared_ptr<rmf_traffic::schedule::Database> database_,
    QueryMap registered_queries_,
    const rclcpp::NodeOptions& options);

  ScheduleNode(NodeVersion node_version_, const rclcpp::NodeOptions& options);

  ScheduleNode(
    NodeVersion node_version_,
    const rclcpp::NodeOptions& options,
    NoAutomaticSetup);

  ~ScheduleNode();

  virtual void setup(const QueryMap& queries);

  std::chrono::milliseconds heartbeat_period = 1s;
  rclcpp::QoS heartbeat_qos_profile;
  using Heartbeat = rmf_traffic_msgs::msg::Heartbeat;
  using HeartbeatPub = rclcpp::Publisher<Heartbeat>;
  HeartbeatPub::SharedPtr heartbeat_pub;

  virtual void setup_redundancy();
  virtual void start_heartbeat();

  using request_id_ptr = std::shared_ptr<rmw_request_id_t>;

  using RegisterQuery = rmf_traffic_msgs::srv::RegisterQuery;
  using RegisterQueryService = rclcpp::Service<RegisterQuery>;

  virtual void register_query(
    const request_id_ptr& request_header,
    const RegisterQuery::Request::SharedPtr& request,
    const RegisterQuery::Response::SharedPtr& response);

  void register_query(
    uint64_t query_id,
    const rmf_traffic::schedule::Query& query);

  RegisterQueryService::SharedPtr register_query_service;

  // How often we should check the query topics to see if they have lost all
  // their subscribers.
  std::chrono::nanoseconds query_cleanup_period = std::chrono::minutes(5);

  // If a query has no subscribers, we will unregister it, unless it has
  // received a new registration request within this time period.
  std::chrono::nanoseconds query_grace_period = std::chrono::minutes(5);

  rclcpp::TimerBase::SharedPtr query_cleanup_timer;
  void cleanup_queries();

  rclcpp::TimerBase::SharedPtr cull_timer;
  void cull();

  virtual void setup_query_services();

  using RegisterParticipant = rmf_traffic_msgs::srv::RegisterParticipant;
  using RegisterParticipantSrv = rclcpp::Service<RegisterParticipant>;

  virtual void register_participant(
    const request_id_ptr& request_header,
    const RegisterParticipant::Request::SharedPtr& request,
    const RegisterParticipant::Response::SharedPtr& response);

  RegisterParticipantSrv::SharedPtr register_participant_service;

  using UnregisterParticipant = rmf_traffic_msgs::srv::UnregisterParticipant;
  using UnregisterParticipantSrv = rclcpp::Service<UnregisterParticipant>;

  virtual void unregister_participant(
    const request_id_ptr& request_header,
    const UnregisterParticipant::Request::SharedPtr& request,
    const UnregisterParticipant::Response::SharedPtr& response);

  UnregisterParticipantSrv::SharedPtr unregister_participant_service;

  virtual void setup_participant_services();

  using MirrorUpdate = rmf_traffic_msgs::msg::MirrorUpdate;
  using MirrorUpdateTopicPublisher = rclcpp::Publisher<MirrorUpdate>::SharedPtr;

  void add_query_topic(uint64_t query_id);
  void remove_query_topic(uint64_t query_id);

  void make_mirror_update_topics(const QueryMap& queries);

  using SingleParticipantInfo = rmf_traffic_msgs::msg::Participant;
  using ParticipantsInfo = rmf_traffic_msgs::msg::Participants;
  rclcpp::Publisher<ParticipantsInfo>::SharedPtr participants_info_pub;
  virtual void broadcast_participants();

  using ScheduleQuery = rmf_traffic_msgs::msg::ScheduleQuery;
  using ScheduleQueries = rmf_traffic_msgs::msg::ScheduleQueries;
  rclcpp::Publisher<ScheduleQueries>::SharedPtr queries_info_pub;
  virtual void broadcast_queries();

  using RequestChanges = rmf_traffic_msgs::srv::RequestChanges;
  using RequestChangesSrv = rclcpp::Service<RequestChanges>;
  void request_changes(
    const request_id_ptr& request_header,
    const RequestChanges::Request::SharedPtr& request,
    const RequestChanges::Response::SharedPtr& response);
  RequestChangesSrv::SharedPtr request_changes_service;

  virtual void setup_changes_services();

  using ItinerarySet = rmf_traffic_msgs::msg::ItinerarySet;
  void itinerary_set(const ItinerarySet& set);
  rclcpp::Subscription<ItinerarySet>::SharedPtr itinerary_set_sub;

  using ItineraryExtend = rmf_traffic_msgs::msg::ItineraryExtend;
  void itinerary_extend(const ItineraryExtend& extend);
  rclcpp::Subscription<ItineraryExtend>::SharedPtr itinerary_extend_sub;

  using ItineraryDelay = rmf_traffic_msgs::msg::ItineraryDelay;
  void itinerary_delay(const ItineraryDelay& delay);
  rclcpp::Subscription<ItineraryDelay>::SharedPtr itinerary_delay_sub;

  using ItineraryReached = rmf_traffic_msgs::msg::ItineraryReached;
  void itinerary_reached(const ItineraryReached& msg);
  rclcpp::Subscription<ItineraryReached>::SharedPtr itinerary_reached_sub;

  using ItineraryClear = rmf_traffic_msgs::msg::ItineraryClear;
  void itinerary_clear(const ItineraryClear& clear);
  rclcpp::Subscription<ItineraryClear>::SharedPtr itinerary_clear_sub;

  virtual void setup_itinerary_topics();

  using InconsistencyMsg = rmf_traffic_msgs::msg::ScheduleInconsistency;
  rclcpp::Publisher<InconsistencyMsg>::SharedPtr inconsistency_pub;
  void publish_inconsistencies(rmf_traffic::schedule::ParticipantId id);

  virtual void setup_incosistency_pub();

  rclcpp::TimerBase::SharedPtr mirror_update_timer;
  void update_mirrors();
  void update_query(
    const MirrorUpdateTopicPublisher& publisher,
    const rmf_traffic::schedule::Query& query,
    VersionOpt last_sent_version,
    bool is_remedial);

  // TODO(MXG): Consider using libguarded instead of a database_mutex
  std::mutex database_mutex;
  std::shared_ptr<rmf_traffic::schedule::Database> database;

  struct QueryInfo
  {
    rmf_traffic::schedule::Query query;
    MirrorUpdateTopicPublisher publisher;
    VersionOpt last_sent_version;
    std::chrono::steady_clock::time_point last_registration_time;
    std::unordered_set<VersionOpt> remediation_requests;
  };
  using QueryInfoMap = std::unordered_map<uint64_t, QueryInfo>;

  std::size_t last_query_id = 0;
  QueryInfoMap registered_queries;

  // TODO(MXG): Make this a separate node
  std::thread conflict_check_thread;
  std::condition_variable conflict_check_cv;
  std::atomic_bool conflict_check_quit;

  using ConflictAck = rmf_traffic_msgs::msg::NegotiationAck;
  using ConflictAckSub = rclcpp::Subscription<ConflictAck>;
  ConflictAckSub::SharedPtr conflict_ack_sub;
  void receive_conclusion_ack(const ConflictAck& msg);

  using ConflictNotice = rmf_traffic_msgs::msg::NegotiationNotice;
  using ConflictNoticePub = rclcpp::Publisher<ConflictNotice>;
  ConflictNoticePub::SharedPtr conflict_notice_pub;

  using ConflictRefusal = rmf_traffic_msgs::msg::NegotiationRefusal;
  using ConflictRefusalSub = rclcpp::Subscription<ConflictRefusal>;
  ConflictRefusalSub::SharedPtr conflict_refusal_sub;
  void receive_refusal(const ConflictRefusal& msg);
  void refuse(std::size_t conflict_version);

  using ConflictProposal = rmf_traffic_msgs::msg::NegotiationProposal;
  using ConflictProposalSub = rclcpp::Subscription<ConflictProposal>;
  ConflictProposalSub::SharedPtr conflict_proposal_sub;
  void receive_proposal(const ConflictProposal& msg);

  using ConflictRejection = rmf_traffic_msgs::msg::NegotiationRejection;
  using ConflictRejectionSub = rclcpp::Subscription<ConflictRejection>;
  ConflictRejectionSub::SharedPtr conflict_rejection_sub;
  void receive_rejection(const ConflictRejection& msg);

  using ConflictForfeit = rmf_traffic_msgs::msg::NegotiationForfeit;
  using ConflictForfeitSub = rclcpp::Subscription<ConflictForfeit>;
  ConflictForfeitSub::SharedPtr conflict_forfeit_sub;
  void receive_forfeit(const ConflictForfeit& msg);

  using ConflictConclusion = rmf_traffic_msgs::msg::NegotiationConclusion;
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
    struct OpenNegotiation
    {
      NegotiationRoom room;
      rmf_traffic::Time start_time;
    };

    struct Wait
    {
      Version negotiation_version;
      std::optional<ItineraryVersion> itinerary_update_version;
      rmf_traffic::Time conclusion_time;
    };

    ConflictRecord(
      std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer)
    : _viewer(std::move(viewer))
    {
      // Do nothing
    }

    std::optional<Entry> insert(
      const ConflictSet& conflicts,
      const rmf_traffic::Time time)
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
          return std::nullopt;
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
        return std::nullopt;

      const Version negotiation_version = existing_negotiation ?
        *existing_negotiation : _next_negotiation_version++;

      const auto insertion = _negotiations.insert(
        std::make_pair(negotiation_version, std::nullopt));

      for (const auto p : add_to_negotiation)
        _version[p] = negotiation_version;

      auto& update_negotiation = insertion.first->second;
      if (!update_negotiation)
      {
        update_negotiation = OpenNegotiation{
          *rmf_traffic::schedule::Negotiation::make(
            _viewer->snapshot(), std::vector<ParticipantId>(
              add_to_negotiation.begin(), add_to_negotiation.end())),
          time
        };
      }
      else
      {
        for (const auto p : add_to_negotiation)
        {
          update_negotiation->room.negotiation.add_participant(p);
          update_negotiation->start_time = time;
        }
      }

      return Entry{negotiation_version, &update_negotiation->room.negotiation};
    }

    NegotiationRoom* negotiation(const Version version)
    {
      const auto it = _negotiations.find(version);
      if (it == _negotiations.end())
        return nullptr;

      assert(it->second);

      return &(it->second->room);
    }

    void conclude(const Version version, rmf_traffic::Time time)
    {
      const auto negotiation_it = _negotiations.find(version);
      if (negotiation_it == _negotiations.end())
        return;

      const auto& participants =
        negotiation_it->second->room.negotiation.participants();

      for (const auto p : participants)
      {
        const auto insertion =
          _waiting.insert(
          {
            p,
            Wait{
              version,
              std::nullopt,
              time
            }
          });

        assert(insertion.second);
        (void)(insertion);
        _version.erase(p);
      }

      _negotiations.erase(negotiation_it);
    }

    void refuse(const Version version)
    {
      const auto negotiation_it = _negotiations.find(version);
      if (negotiation_it == _negotiations.end())
        return;

      const auto& participants =
        negotiation_it->second->room.negotiation.participants();

      for (const auto p : participants)
        _version.erase(p);

      _negotiations.erase(version);
    }

    // Tell the ConflictRecord what ItineraryVersion will resolve this
    // negotiation.
    void acknowledge(
      const Version negotiation_version,
      const ParticipantId p,
      const std::optional<ItineraryVersion> update_version)
    {
      const auto wait_it = _waiting.find(p);
      if (wait_it == _waiting.end())
      {
        // TODO(MXG): We should probably output some warning here using
        // RCLCPP_WARN
        std::cout << "[ScheduleNode::ConflictRecord::acknowledge] We are NOT "
                  << "waiting for an acknowledgment from participant [" << p
                  << "] for negotiation [" << negotiation_version << "]"
                  << std::endl;
        assert(false);
        return;
      }

      const auto expected_negotiation = wait_it->second.negotiation_version;
      if (expected_negotiation != negotiation_version)
      {
        // TODO(MXG): We should probably output some warning here using
        // RCLCPP_WARN
        std::cout << "[ScheduleNode::ConflictRecord::acknowledge] We are "
                  << "waiting for an acknowledgment from participant ["
                  << p << "] regarding negotiation [" << expected_negotiation
                  << "] but received an acknowledgment for negotiation ["
                  << negotiation_version << "] instead." << std::endl;
        assert(false);
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
      std::optional<OpenNegotiation>> _negotiations;
    std::unordered_map<ParticipantId, Wait> _waiting;
    std::shared_ptr<const rmf_traffic::schedule::Snappable> _viewer;
    Version _next_negotiation_version = 0;
  };

  ConflictRecord active_conflicts;
  std::mutex active_conflicts_mutex;
  std::shared_ptr<ParticipantRegistry> participant_registry;

  virtual void setup_conflict_topics_and_thread();
  void setup_cull_timer();

  // TODO(MXG): Build this into the Database/Mirror class, tracking participant
  // description versions separately from itinerary versions.
  std::size_t last_known_participants_version = 0;
  std::size_t current_participants_version = 1;
};

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
