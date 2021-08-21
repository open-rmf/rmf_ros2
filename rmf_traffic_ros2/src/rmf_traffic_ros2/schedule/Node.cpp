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

#include "internal_Node.hpp"

#include <cstring>

#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/schedule/Itinerary.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>
#include <rmf_traffic_ros2/schedule/Patch.hpp>
#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/schedule/Inconsistencies.hpp>

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/schedule/Mirror.hpp>

#include <rmf_utils/optional.hpp>

#include <unordered_map>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
std::vector<ScheduleNode::ConflictSet> get_conflicts(
  const rmf_traffic::schedule::Viewer::View& view_changes,
  const rmf_traffic::schedule::ItineraryViewer& viewer)
{
  std::vector<ScheduleNode::ConflictSet> conflicts;
  const auto& participants = viewer.participant_ids();
  for (const auto participant : participants)
  {
    const auto itinerary = *viewer.get_itinerary(participant);
    const auto description = viewer.get_participant(participant);
    if (!description)
      continue;

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
            description->profile(),
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
// This constructor will _not_ automatically call the setup() method to finalise
// construction of the ScheduleNode object. setup() must be called manually.
ScheduleNode::ScheduleNode(
  std::shared_ptr<rmf_traffic::schedule::Database> database_,
  QueryMap registered_queries_,
  const rclcpp::NodeOptions& options)
: Node("rmf_traffic_schedule_node", options),
  heartbeat_qos_profile(1),
  database(std::move(database_)),
  registered_queries(std::move(registered_queries_)),
  active_conflicts(database)
{
  // Period, in milliseconds, for sending out a heartbeat signal to the monitor
  // node in the redundant pair
  declare_parameter<int>("heartbeat_period", 1000);
  heartbeat_period = std::chrono::milliseconds(
    get_parameter("heartbeat_period").as_int());

  // Participant registry location
  declare_parameter<std::string>(
    "log_file_location", ".rmf_schedule_node.yaml");
}

//==============================================================================
// This constructor will automatically call the setup() method to finalise
// construction of the ScheduleNode object.
ScheduleNode::ScheduleNode(
  std::shared_ptr<rmf_traffic::schedule::Database> database_,
  QueryMap registered_queries_,
  QuerySubscriberCountMap registered_query_subscriber_counts,
  const rclcpp::NodeOptions& options)
: ScheduleNode(
    database_,
    registered_queries_,
    options)
{
  setup(registered_query_subscriber_counts);
}

//==============================================================================
// This constructor will automatically call the setup() method to finalise
// construction of the ScheduleNode object.
ScheduleNode::ScheduleNode(const rclcpp::NodeOptions& options)
: ScheduleNode(  // Call the version that will automatically call setup(...)
    std::make_shared<rmf_traffic::schedule::Database>(),
    QueryMap(),
    QuerySubscriberCountMap(),
    options)
{
}

//==============================================================================
// This constructor will _not_ automatically call the setup() method to finalise
// construction of the ScheduleNode object. setup() must be called manually.
ScheduleNode::ScheduleNode(
  const rclcpp::NodeOptions& options,
  NoAutomaticSetup)
: ScheduleNode(  // Call the version that does not call setup(...)
    std::make_shared<rmf_traffic::schedule::Database>(),
    QueryMap(),
    options)
{
  // No setup(...) call here; it must be called manually
}

//==============================================================================
ScheduleNode::~ScheduleNode()
{
  conflict_check_quit = true;
  if (conflict_check_thread.joinable())
    conflict_check_thread.join();
}

//==============================================================================
void ScheduleNode::setup(
  QuerySubscriberCountMap registered_query_subscriber_counts)
{
  //Attempt to load/create participant registry.
  std::string log_file_name;
  get_parameter_or<std::string>(
    "log_file_location",
    log_file_name,
    ".rmf_schedule_node.yaml");

  // Re-instantiate any query update topics based on received queries
  remake_mirror_update_topics(registered_query_subscriber_counts);

  try
  {
    auto participant_logger = std::make_unique<YamlLogger>(log_file_name);

    participant_registry =
      std::make_shared<ParticipantRegistry>(
      std::move(participant_logger),
      database);

    RCLCPP_INFO(get_logger(),
      "Successfully loaded logfile %s ",
      log_file_name.c_str());
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_FATAL(get_logger(),
      "Failed to correctly load participant registry: %s\n",
      e.what());
    throw e;
  }

  setup_redundancy();
  setup_query_services();
  setup_participant_services();
  setup_changes_services();
  setup_itinerary_topics();
  setup_incosistency_pub();
  setup_conflict_topics_and_thread();
}

//==============================================================================
void ScheduleNode::setup_query_services()
{
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
}

//==============================================================================
void ScheduleNode::setup_participant_services()
{
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
}

//==============================================================================
void ScheduleNode::setup_changes_services()
{
  request_changes_service =
    create_service<RequestChanges>(
    rmf_traffic_ros2::RequestChangesServiceName,
    [=](const request_id_ptr request_header,
    const RequestChanges::Request::SharedPtr request,
    const RequestChanges::Response::SharedPtr response)
    { this->request_changes(request_header, request, response); });
}

//==============================================================================
void ScheduleNode::setup_itinerary_topics()
{
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
}

//==============================================================================
void ScheduleNode::setup_incosistency_pub()
{
  inconsistency_pub =
    create_publisher<InconsistencyMsg>(
    rmf_traffic_ros2::ScheduleInconsistencyTopicName,
    rclcpp::SystemDefaultsQoS().reliable());
}

//==============================================================================
void ScheduleNode::setup_conflict_topics_and_thread()
{
  const auto negotiation_qos = rclcpp::ServicesQoS().reliable();
  conflict_ack_sub = create_subscription<ConflictAck>(
    rmf_traffic_ros2::NegotiationAckTopicName, negotiation_qos,
    [&](const ConflictAck::UniquePtr msg)
    {
      this->receive_conclusion_ack(*msg);
    });

  conflict_notice_pub = create_publisher<ConflictNotice>(
    rmf_traffic_ros2::NegotiationNoticeTopicName, negotiation_qos);

  conflict_refusal_sub = create_subscription<ConflictRefusal>(
    rmf_traffic_ros2::NegotiationRefusalTopicName, negotiation_qos,
    [&](const ConflictRefusal::UniquePtr msg)
    {
      this->receive_refusal(*msg);
    });

  conflict_proposal_sub = create_subscription<ConflictProposal>(
    rmf_traffic_ros2::NegotiationProposalTopicName, negotiation_qos,
    [&](const ConflictProposal::UniquePtr msg)
    {
      this->receive_proposal(*msg);
    });

  conflict_rejection_sub = create_subscription<ConflictRejection>(
    rmf_traffic_ros2::NegotiationRejectionTopicName, negotiation_qos,
    [&](const ConflictRejection::UniquePtr msg)
    {
      this->receive_rejection(*msg);
    });

  conflict_forfeit_sub = create_subscription<ConflictForfeit>(
    rmf_traffic_ros2::NegotiationForfeitTopicName, negotiation_qos,
    [&](const ConflictForfeit::UniquePtr msg)
    {
      this->receive_forfeit(*msg);
    });

  conflict_conclusion_pub = create_publisher<ConflictConclusion>(
    rmf_traffic_ros2::NegotiationConclusionTopicName, negotiation_qos);

  conflict_check_quit = false;
  conflict_check_thread = std::thread(
    [&]()
    {
      rmf_traffic::schedule::Mirror mirror;
      const auto query_all = rmf_traffic::schedule::query_all();
      Version last_checked_version = 0;

      while (rclcpp::ok(get_node_options().context()) && !conflict_check_quit)
      {
        rmf_utils::optional<rmf_traffic::schedule::Patch> next_patch;
        rmf_traffic::schedule::Viewer::View view_changes;

        // Use this scope to minimize how long we lock the database for
        {
          std::unique_lock<std::mutex> lock(database_mutex);
          conflict_check_cv.wait_for(lock, std::chrono::milliseconds(100), [&]()
          {
            return (database->latest_version() > last_checked_version)
            && !conflict_check_quit;
          });

          if ( (database->latest_version() == last_checked_version
          && last_known_participants_version == current_participants_version)
          || conflict_check_quit)
          {
            // This is a casual wakeup to check if we're supposed to quit yet
            continue;
          }

          if (last_known_participants_version != current_participants_version)
          {
            last_known_participants_version = current_participants_version;
            rmf_traffic::schedule::ParticipantDescriptionsMap participants;
            for (const auto& id: database->participant_ids())
            {
              participants.insert({id, *database->get_participant(id)});
            }

            try
            {
              mirror.update_participants_info(participants);
            }
            catch (const std::exception& e)
            {
              RCLCPP_ERROR(get_logger(), e.what());
            }
          }

          next_patch = database->changes(query_all, last_checked_version);
          // TODO(MXG): Check whether the database really needs to remain locked
          // during this update.
          try
          {
            mirror.update(*next_patch);
            view_changes = database->query(query_all, last_checked_version);
            last_checked_version = next_patch->latest_version();
          }
          catch (const std::exception& e)
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
void ScheduleNode::setup_redundancy()
{
  start_heartbeat();

  participants_info_pub =
    create_publisher<ParticipantsInfo>(
    rmf_traffic_ros2::ParticipantsInfoTopicName,
    rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local());

  queries_info_pub =
    create_publisher<ScheduleQueries>(
    rmf_traffic_ros2::QueriesInfoTopicName,
    rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local());
}

//==============================================================================
void ScheduleNode::start_heartbeat()
{
  // Set up liveliness announcements of this node, powered by DDS[tm][r][c][rgb]
  heartbeat_qos_profile
  .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
  .liveliness_lease_duration(heartbeat_period)
  .deadline(heartbeat_period);

  heartbeat_pub = create_publisher<Heartbeat>(
    rmf_traffic_ros2::HeartbeatTopicName,
    heartbeat_qos_profile);
  RCLCPP_INFO(
    get_logger(),
    "Set up heartbeat on %s with liveliness lease duration of %ld ms "
    "and deadline of %ld ms",
    heartbeat_pub->get_topic_name(),
    heartbeat_period.count(),
    heartbeat_period.count());
}

//==============================================================================
void ScheduleNode::add_query_topic(uint64_t query_id)
{
  RCLCPP_INFO(get_logger(), "Adding query topic for query %ld", query_id);
  MirrorUpdateTopicPublisher update_topic_publisher =
    create_publisher<MirrorUpdate>(
    rmf_traffic_ros2::QueryUpdateTopicNameBase + std::to_string(query_id),
    rclcpp::SystemDefaultsQoS());
  // Start the latest version sent for this query at the oldest version of the
  // database. This will cause the new participant to be updated with all
  // currently-relevant information from the database, not just the next
  // change to come in.
  MirrorUpdateTopicInfo update_topic {
    update_topic_publisher,
    std::nullopt,
    1
  };
  mirror_update_topics.insert({query_id, update_topic});
}

//==============================================================================
void ScheduleNode::remove_query_topic(uint64_t query_id)
{
  RCLCPP_INFO(get_logger(), "Removing query topic for query %ld", query_id);
  const auto& query_topic = mirror_update_topics.find(query_id);
  if (query_topic == mirror_update_topics.end())
  {
    // Missing query update topic; something has gone very wrong but it doesn't
    // matter much since we were going to remove the topic anyway
    RCLCPP_ERROR(
      get_logger(),
      "Could not find expected mirror update topic to remove for query ID %ld",
      query_id);
    return;
  }
  mirror_update_topics.erase(query_topic);
}

//==============================================================================
void ScheduleNode::add_subscriber_to_query_topic(uint64_t query_id)
{
  RCLCPP_INFO(
    get_logger(),
    "Adding subscriber to query topic for query %ld",
    query_id);
  const auto& query_topic = mirror_update_topics.find(query_id);
  if (query_topic == mirror_update_topics.end())
  {
    // Missing query update topic; something has gone very wrong.
    // TODO(Geoff): Make a new topic if an existing one can't be found? Or
    // respond with failure to the request?
    RCLCPP_ERROR(
      get_logger(),
      "Could not find expected mirror update topic for existing query ID %ld "
      "to add subscriber to",
      query_id);
    return;
  }
  auto mirror_update_topic_info = query_topic->second;
  mirror_update_topic_info.subscriber_count += 1;
  mirror_update_topics.insert_or_assign(query_id, mirror_update_topic_info);
  RCLCPP_DEBUG(get_logger(), "Query topic has %ld subscribers",
    mirror_update_topic_info.subscriber_count);
}

//==============================================================================
ScheduleNode::SubscriberRemovalResult
ScheduleNode::remove_subscriber_from_query_topic(
  uint64_t query_id)
{
  RCLCPP_INFO(
    get_logger(),
    "Removing subscriber from query topic for query %ld",
    query_id);
  const auto& query_topic = mirror_update_topics.find(query_id);
  if (query_topic == mirror_update_topics.end())
  {
    // Missing query update topic; something has gone very wrong.
    // TODO(Geoff): Make a new topic if an existing one can't be found? Or
    // respond with failure to the request?
    RCLCPP_ERROR(
      get_logger(),
      "Could not find expected mirror update topic for existing query ID %ld "
      "to remove subscriber from",
      query_id);
    return SubscriberRemovalResult::query_missing;
  }

  auto mirror_update_topic_info = query_topic->second;
  mirror_update_topic_info.subscriber_count -= 1;
  RCLCPP_DEBUG(get_logger(), "Query topic has %ld subscribers",
    mirror_update_topic_info.subscriber_count);
  if (mirror_update_topic_info.subscriber_count == 0)
  {
    remove_query_topic(query_id);
    return SubscriberRemovalResult::query_removed;
  }
  else
  {
    mirror_update_topics.insert_or_assign(query_id, mirror_update_topic_info);
    return SubscriberRemovalResult::query_in_use;
  }
}

//==============================================================================
void ScheduleNode::remake_mirror_update_topics(
  const QuerySubscriberCountMap& subscriber_counts)
{
  // Delete any existing topics, just to be sure
  mirror_update_topics.clear();

  for (const auto& subscriber_count: subscriber_counts)
  {
    auto query_id = subscriber_count.first;
    RCLCPP_INFO(
      get_logger(),
      "Remaking query topic for query ID %ld (%ld subscribers)",
      query_id,
      subscriber_count.second);
    add_query_topic(query_id);
    // Set the subscriber count manually
    const auto& query_topic = mirror_update_topics.find(query_id);
    if (query_topic == mirror_update_topics.end())
    {
      // Missing query update topic; something has gone _unbelievably_ wrong.
      // Didn't we just add this topic?
      RCLCPP_ERROR(
        get_logger(),
        "Could not find expected mirror update topic for existing query ID %ld "
        "to set subscriber count on",
        query_id);
      continue;
    }
    auto mirror_update_topic_info = query_topic->second;
    mirror_update_topic_info.subscriber_count = subscriber_count.second;
    mirror_update_topics.insert_or_assign(
      query_id,
      mirror_update_topic_info);
  }
}


//==============================================================================
void ScheduleNode::register_query(
  const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
  const RegisterQuery::Request::SharedPtr& request,
  const RegisterQuery::Response::SharedPtr& response)
{
  rmf_traffic::schedule::Query new_query =
    rmf_traffic_ros2::convert(request->query);
  bool query_exists = false;
  uint64_t query_id = last_query_id;
  // Search for an existing query with the same search parameters
  for (const auto& [existing_query_id, existing_query] : registered_queries)
  {
    if (existing_query == new_query)
    {
      query_exists = true;
      query_id = existing_query_id;
      break;
    }
  }

  if (query_exists)
  {
    // Query exists and query_id will be the existing query.

    // Reset the latest version sent for this query to the oldest version of the
    // database. This will ensure that the new participant receives all
    // currently-relevant information from the database, not just the next
    // change to come in.

    // Record an additional subscriber for this query
    add_subscriber_to_query_topic(query_id);
    RCLCPP_INFO(get_logger(), "[%ld] Added mirror to query", query_id);
  }
  else
  {
    // If the query does not exist, then query_id is still at last_query_id.
    // Find an unused query ID, store the query, and create a topic to publish
    // updates that match it.
    // Note that this search may begin at query ID 0 if this is the first time
    // it is performed on a replacement schedule node. This is because the set
    // of queries will have been filled in from the original schedule node's
    // synchronised data, but last_query_id will have been initialised to zero
    // when the replacement was constructed. This is not a problem because a
    // search for the next available query ID does not need to be performed
    // until we actually need a new query ID, so performing it in the
    // constructor in advance would be unnecessary early optimisation.
    uint64_t attempts = 0;
    do
    {
      ++query_id;
      ++attempts;
      if (attempts == std::numeric_limits<uint64_t>::max())
      {
        // I suspect a computer would run out of RAM before we reach this point,
        // but there's no harm in double-checking.
        response->error =
          "No more space for additional queries to be registered";
        RCLCPP_ERROR(
          get_logger(),
          "[ScheduleNode::register_query] %s",
          response->error.c_str());
        return;
      }
    } while (registered_queries.find(query_id) != registered_queries.end());

    last_query_id = query_id;
    registered_queries.insert(
      std::make_pair(query_id, rmf_traffic_ros2::convert(request->query)));

    // Create the topic for updating those interested in this query
    add_query_topic(query_id);
    RCLCPP_INFO(get_logger(), "[%ld] Registered query", query_id);
  }

  broadcast_queries();

  // If query does exist, query_id is already at the existing query ID and a
  // topic already exists. Return the query ID to the client without creating a
  // new topic or query.
  response->query_id = query_id;
}

//==============================================================================
void ScheduleNode::unregister_query(
  const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
  const UnregisterQuery::Request::SharedPtr& request,
  const UnregisterQuery::Response::SharedPtr& response)
{
  const auto it = registered_queries.find(request->query_id);
  if (it == registered_queries.end())
  {
    response->error = "No query found with the id ["
      + std::to_string(request->query_id) + "]";
    response->confirmation = false;

    RCLCPP_INFO(
      get_logger(),
      "[ScheduleNode::unregister_query] %s",
      response->error.c_str());
    return;
  }

  // Remove a subscriber for this query
  auto removal_result = remove_subscriber_from_query_topic(request->query_id);
  if (removal_result == SubscriberRemovalResult::query_removed)
  {
    registered_queries.erase(it);
    RCLCPP_INFO(get_logger(), "[%ld] Unregistered query", request->query_id);
  }
  broadcast_queries();
  response->confirmation = true;
}

//==============================================================================
void ScheduleNode::broadcast_queries()
{
  ScheduleQueries msg;

  for (const auto& registered_query: registered_queries)
  {
    msg.ids.push_back(registered_query.first);

    rmf_traffic::schedule::Query original =
      registered_queries.at(registered_query.first);
    ScheduleQuery query = rmf_traffic_ros2::convert(original);
    msg.queries.push_back(query);

    const auto& query_topic = mirror_update_topics.find(registered_query.first);
    msg.subscriber_counts.push_back(query_topic->second.subscriber_count);
  }
  queries_info_pub->publish(msg);
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
    const auto registration = participant_registry
      ->add_or_retrieve_participant(
      rmf_traffic_ros2::convert(request->description));

    using Response = rmf_traffic_msgs::srv::RegisterParticipant::Response;

    *response =
      rmf_traffic_msgs::build<Response>()
      .participant_id(registration.id())
      .last_itinerary_version(registration.last_itinerary_version())
      .last_route_id(registration.last_route_id())
      .error("");

    RCLCPP_INFO(
      get_logger(),
      "Registered participant [%ld] named [%s] owned by [%s]",
      response->participant_id,
      request->description.name.c_str(),
      request->description.owner.c_str());

    broadcast_participants();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to register participant [%s] owned by [%s]: %s",
      request->description.name.c_str(),
      request->description.owner.c_str(),
      e.what());
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

  const auto& p = database->get_participant(request->participant_id);
  if (!p)
  {
    response->error =
      "Failed to unregister participant ["
      + std::to_string(request->participant_id) + "] because no "
      "participant has that ID";
    response->confirmation = false;

    RCLCPP_ERROR(get_logger(), response->error.c_str());
    return;
  }

  try
  {
    // We need to copy this data before the participant is unregistered, because
    // unregistering it will invalidate the pointer p.
    const std::string name = p->name();
    const std::string owner = p->owner();

    auto version = database->itinerary_version(request->participant_id);
    database->erase(request->participant_id, version);
    response->confirmation = true;

    RCLCPP_INFO(
      get_logger(),
      "Unregistered participant [%ld] named [%s] owned by [%s]",
      request->participant_id,
      name.c_str(),
      owner.c_str());

    broadcast_participants();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to unregister participant [%ld]: %s",
      request->participant_id,
      e.what());
    response->error = e.what();
    response->confirmation = false;
  }
}

//==============================================================================
void ScheduleNode::broadcast_participants()
{
  ++current_participants_version;
  ParticipantsInfo msg;

  for (const auto& id: database->participant_ids())
  {
    SingleParticipantInfo participant;
    participant.id = id;
    participant.description = rmf_traffic_ros2::convert(
      *database->get_participant(id));
    msg.participants.push_back(participant);
  }
  participants_info_pub->publish(msg);
}

//==============================================================================
void ScheduleNode::request_changes(
  [[maybe_unused]] const request_id_ptr& request_header,
  const RequestChanges::Request::SharedPtr& request,
  const RequestChanges::Response::SharedPtr& response)
{
  const auto query_topic = mirror_update_topics.find(request->query_id);
  if (query_topic == mirror_update_topics.end())
  {
    // Missing query update topic; something has gone very wrong.
    RCLCPP_ERROR(
      get_logger(),
      "[ScheduleNode::request_changes] Could not find mirror update topic "
      "for query ID %ld",
      request->query_id);
    response->result = RequestChanges::Response::UNKNOWN_QUERY_ID;
  }
  else
  {
    auto& mirror_update_topic_info = query_topic->second;
    // Tell the next update to send the changes since the requested version by
    // resetting the last sent version number to the requested version,
    // which may be std::nullopt if a full update is requested
    if (request->full_update)
    {
      mirror_update_topic_info.last_sent_version = std::nullopt;
    }
    else
    {
      if (mirror_update_topic_info.last_sent_version.has_value() &&
        rmf_utils::modular(request->version).less_than(
          *mirror_update_topic_info.last_sent_version))
      {
        mirror_update_topic_info.last_sent_version = request->version;
      }
    }

    // Force-send the next update
    update_mirrors();

    response->result = RequestChanges::Response::REQUEST_ACCEPTED;
  }
}

//==============================================================================
void ScheduleNode::itinerary_set(const ItinerarySet& set)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  assert(!set.itinerary.empty());
  database->set(
    set.participant,
    rmf_traffic_ros2::convert(set.itinerary),
    set.itinerary_version);

  publish_inconsistencies(set.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(set.participant, set.itinerary_version);
  update_mirrors();
}

//==============================================================================
void ScheduleNode::itinerary_extend(const ItineraryExtend& extend)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  database->extend(
    extend.participant,
    rmf_traffic_ros2::convert(extend.routes),
    extend.itinerary_version);

  publish_inconsistencies(extend.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(
    extend.participant, database->itinerary_version(extend.participant));
  update_mirrors();
}

//==============================================================================
void ScheduleNode::itinerary_delay(const ItineraryDelay& delay)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  database->delay(
    delay.participant,
    rmf_traffic::Duration(delay.delay),
    delay.itinerary_version);

  publish_inconsistencies(delay.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(
    delay.participant, database->itinerary_version(delay.participant));
  update_mirrors();
}

//==============================================================================
void ScheduleNode::itinerary_erase(const ItineraryErase& erase)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  database->erase(
    erase.participant,
    std::vector<rmf_traffic::RouteId>(
      erase.routes.begin(), erase.routes.end()),
    erase.itinerary_version);

  publish_inconsistencies(erase.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(
    erase.participant, database->itinerary_version(erase.participant));
  update_mirrors();
}

//==============================================================================
void ScheduleNode::itinerary_clear(const ItineraryClear& clear)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  database->erase(clear.participant, clear.itinerary_version);

  publish_inconsistencies(clear.participant);

  std::lock_guard<std::mutex> lock2(active_conflicts_mutex);
  active_conflicts.check(
    clear.participant, database->itinerary_version(clear.participant));
  update_mirrors();
}

//==============================================================================
void ScheduleNode::publish_inconsistencies(
  rmf_traffic::schedule::ParticipantId id)
{
  // TODO(MXG): This approach is likely to send out a lot of redundant
  // inconsistency reports. We should try to be smarter about how
  // inconsistencies get reported.
  const auto it = database->inconsistencies().find(id);
  assert(it != database->inconsistencies().end());
  if (it->ranges.size() == 0)
    return;

  inconsistency_pub->publish(rmf_traffic_ros2::convert(*it));
}

//==============================================================================
void ScheduleNode::update_mirrors()
{
  rmf_traffic_msgs::msg::MirrorUpdate msg;
  msg.database_version = database->latest_version();

  for (const auto& query_it : registered_queries)
  {
    const auto query_topic = mirror_update_topics.find(query_it.first);
    if (query_topic == mirror_update_topics.end())
    {
      // Missing query update topic; not a fatal error, but still a sign that
      // something has gone wrong.
      RCLCPP_ERROR(
        get_logger(),
        "[ScheduleNode::update_mirrors] Could not find mirror update topic "
        "to remove for query ID %ld",
        query_it.first);
      continue;
    }

    msg.query_id = query_it.first;
    msg.query = rmf_traffic_ros2::convert(query_it.second);

    auto& mirror_update_topic_info = query_topic->second;
    const auto patch = database->changes(
      query_it.second,
      mirror_update_topic_info.last_sent_version);

    if (patch.size() == 0 && !patch.cull())
      continue;

    msg.patch = rmf_traffic_ros2::convert(patch);
    mirror_update_topic_info.publisher->publish(msg);

    // Update the latest version sent to this topic
    mirror_update_topic_info.last_sent_version = msg.database_version;

    RCLCPP_DEBUG(
      get_logger(),
      "[ScheduleNode::update_mirrors] Updated query %ld", query_it.first);
  }

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
    .push_back({entry.first,
        entry.second.itinerary_update_version.has_value()});
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
void ScheduleNode::receive_refusal(const ConflictRefusal& msg)
{
  std::unique_lock<std::mutex> lock(active_conflicts_mutex);
  auto* negotiation_room =
    active_conflicts.negotiation(msg.conflict_version);

  if (!negotiation_room)
    return;

  std::string output = "Refused negotiation ["
    + std::to_string(msg.conflict_version) + "]";
  RCLCPP_INFO(get_logger(), output.c_str());

  active_conflicts.refuse(msg.conflict_version);

  ConflictConclusion conclusion;
  conclusion.conflict_version = msg.conflict_version;
  conclusion.resolved = false;
  conflict_conclusion_pub->publish(conclusion);
}

//==============================================================================
void ScheduleNode::receive_proposal(const ConflictProposal& msg)
{
  std::unique_lock<std::mutex> lock(active_conflicts_mutex);
  auto* negotiation_room =
    active_conflicts.negotiation(msg.conflict_version);

  if (!negotiation_room)
    return;

  auto& negotiation = negotiation_room->negotiation;

  const auto search = negotiation.find(
    msg.for_participant, rmf_traffic_ros2::convert(msg.to_accommodate));

  if (search.deprecated())
    return;

  const auto table = search.table;
  if (!table)
  {
    std::string error = "Received proposal in negotiation ["
      + std::to_string(msg.conflict_version) + "] for participant ["
      + std::to_string(msg.for_participant) + "] on unknown table [";
    for (const auto p : msg.to_accommodate)
      error += " " + std::to_string(p.participant) + ":" + std::to_string(
        p.version) + " ";
    error += "]";

    RCLCPP_WARN(get_logger(), error.c_str());
    negotiation_room->cached_proposals.push_back(msg);
    return;
  }

  table->submit(rmf_traffic_ros2::convert(msg.itinerary), msg.proposal_version);
  negotiation_room->check_cache({});

  // TODO(MXG): This should be removed once we have a negotiation visualizer
  rmf_traffic_ros2::schedule::print_negotiation_status(msg.conflict_version,
    negotiation);

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
    conclusion.table = rmf_traffic_ros2::convert(choose->sequence());

    std::string output = "Resolved negotiation ["
      + std::to_string(msg.conflict_version) + "]:";

    for (const auto p : conclusion.table)
      output += " " + std::to_string(p.participant) + ":" + std::to_string(
        p.version);
    RCLCPP_INFO(get_logger(), output.c_str());

    conflict_conclusion_pub->publish(std::move(conclusion));
//    print_conclusion(active_conflicts._waiting);
  }
  else if (negotiation.complete())
  {
    std::string output = "Forfeited negotiation ["
      + std::to_string(msg.conflict_version) + "]";
    RCLCPP_INFO(get_logger(), output.c_str());

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
    return;

  auto& negotiation = negotiation_room->negotiation;

  const auto search = negotiation.find(rmf_traffic_ros2::convert(msg.table));
  if (search.deprecated())
    return;

  const auto table = search.table;
  if (!table)
  {
    std::string error = "Received rejection in negotiation ["
      + std::to_string(msg.conflict_version) + "] for unknown table [";
    for (const auto p : msg.table)
      error += " " + std::to_string(p.participant) + ":" + std::to_string(
        p.version) + " ";
    error += "]";

    RCLCPP_WARN(get_logger(), error.c_str());
    negotiation_room->cached_rejections.push_back(msg);
    return;
  }

  table->reject(
    msg.table.back().version,
    msg.rejected_by,
    rmf_traffic_ros2::convert(msg.alternatives));

  negotiation_room->check_cache({});


  // TODO(MXG): This should be removed once we have a negotiation visualizer
  rmf_traffic_ros2::schedule::print_negotiation_status(msg.conflict_version,
    negotiation);
}

//==============================================================================
void ScheduleNode::receive_forfeit(const ConflictForfeit& msg)
{
  std::unique_lock<std::mutex> lock(active_conflicts_mutex);
  auto* negotiation_room = active_conflicts.negotiation(msg.conflict_version);

  if (!negotiation_room)
    return;

  auto& negotiation = negotiation_room->negotiation;

  const auto search = negotiation.find(rmf_traffic_ros2::convert(msg.table));
  if (search.deprecated())
    return;

  const auto table = search.table;
  if (!table)
  {
    std::string error = "Received forfeit in negotiation ["
      + std::to_string(msg.conflict_version) + "] for unknown table [";
    for (const auto p : msg.table)
      error += " " + std::to_string(p.participant) + ":" + std::to_string(
        p.version) + " ";
    error += "]";

    RCLCPP_WARN(get_logger(), error.c_str());
    negotiation_room->cached_forfeits.push_back(msg);
    return;
  }

  table->forfeit(msg.table.back().version);
  negotiation_room->check_cache({});

  // TODO(MXG): This should be removed once we have a negotiation visualizer
  rmf_traffic_ros2::schedule::print_negotiation_status(msg.conflict_version,
    negotiation);

  if (negotiation.complete())
  {
    std::string output = "Forfeited negotiation ["
      + std::to_string(msg.conflict_version) + "]";
    RCLCPP_INFO(get_logger(), output.c_str());

    active_conflicts.conclude(msg.conflict_version);

    ConflictConclusion conclusion;
    conclusion.conflict_version = msg.conflict_version;
    conclusion.resolved = false;

    conflict_conclusion_pub->publish(conclusion);
//    print_conclusion(active_conflicts._waiting);
  }
}

std::shared_ptr<rclcpp::Node> make_node(const rclcpp::NodeOptions& options)
{
  return std::make_shared<ScheduleNode>(options);
}

} // namespace schedule
} // namespace rmf_traffic_ros2
