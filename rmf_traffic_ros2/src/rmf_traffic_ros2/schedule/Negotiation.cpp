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

#include "NegotiationRoom.hpp"

#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/schedule/Itinerary.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic_msgs/msg/negotiation_ack.hpp>
#include <rmf_traffic_msgs/msg/negotiation_repeat.hpp>
#include <rmf_traffic_msgs/msg/negotiation_notice.hpp>
#include <rmf_traffic_msgs/msg/negotiation_refusal.hpp>
#include <rmf_traffic_msgs/msg/negotiation_forfeit.hpp>
#include <rmf_traffic_msgs/msg/negotiation_proposal.hpp>
#include <rmf_traffic_msgs/msg/negotiation_rejection.hpp>
#include <rmf_traffic_msgs/msg/negotiation_conclusion.hpp>

#include <rclcpp/logging.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
std::string table_to_string(
  const std::vector<rmf_traffic::schedule::ParticipantId>& table)
{
  std::string output;
  for (const auto p : table)
    output += " " + std::to_string(p);
  return output;
}

//==============================================================================
template<typename T>
std::string ptr_to_string(T* ptr)
{
  std::stringstream str;
  str << ptr;
  return str.str();
}

//==============================================================================
class Negotiation::Implementation
{
public:

  class Responder : public rmf_traffic::schedule::Negotiator::Responder
  {
  public:

    Responder(
      Implementation* const impl_,
      const rmf_traffic::schedule::Version version_,
      rmf_traffic::schedule::Negotiation::TablePtr table_)
    : impl(impl_),
      conflict_version(version_),
      table(table_),
      table_version(table->version()),
      parent(table->parent()),
      parent_version(parent ? OptVersion(parent->version()) : OptVersion())
    {
      // Do nothing
    }

    template<typename... Args>
    static std::shared_ptr<Responder> make(
      Args&& ... args)
    {
      auto responder = std::make_shared<Responder>(std::forward<Args>(args)...);
      rclcpp::Node& node = responder->impl->node;
      responder->timer = node.create_wall_timer(
        responder->impl->timeout,
        [r = std::weak_ptr<Responder>(responder)]()
        {
          if (auto responder = r.lock())
          {
            responder->timer.reset();
            responder->timeout();
          }
        });

      return responder;
    }

    void submit(
      rmf_traffic::PlanId plan_id,
      std::vector<rmf_traffic::Route> itinerary,
      std::function<UpdateVersion()> approval_callback) const final
    {
      responded = true;
      if (table->defunct())
        return;

      if (table->submit(plan_id, itinerary, table_version+1))
      {
        impl->approvals[conflict_version][table] = {
          table->sequence(),
          std::move(approval_callback)
        };

        impl->publish_proposal(conflict_version, *table);

        if (impl->worker)
        {
          for (const auto& c : table->children())
          {
            const auto n_it = impl->negotiators->find(c->participant());
            if (n_it == impl->negotiators->end())
              continue;

            impl->worker->schedule(
              [viewer = c->viewer(),
              negotiator = n_it->second.get(),
              responder = make(impl, conflict_version, c)]()
              {
                negotiator->respond(viewer, responder);
              });
          }
        }
      }
    }

    void reject(const Alternatives& alternatives) const final
    {
      responded = true;
      if (parent && !parent->defunct())
      {
        // We will reject the parent to communicate that its proposal is not
        // feasible for us.
        if (parent->reject(*parent_version, table->participant(), alternatives))
        {
          impl->publish_rejection(
            conflict_version, *parent, table->participant(), alternatives);

          // TODO(MXG): We don't schedule a response to the rejection for
          // async negotiations, because the ROS2 subscription will do that for
          // us whether we want it to or not.
//          if (impl->worker)
//          {
//            const auto n_it = impl->negotiators->find(parent->participant());
//            if (n_it == impl->negotiators->end())
//              return;

//            impl->worker->schedule(
//                  [viewer = parent->viewer(),
//                   negotiator = n_it->second.get(),
//                   responder = make(impl, conflict_version, parent)]()
//            {
//              negotiator->respond(viewer, responder);
//            });
//          }
        }
      }
    }

    void forfeit(const std::vector<ParticipantId>& /*blockers*/) const final
    {
      responded = true;
      if (!table->defunct())
      {
        // TODO(MXG): Consider using blockers to invite more participants into the
        // negotiation
        table->forfeit(table_version);
        impl->publish_forfeit(conflict_version, *table);
      }
    }

    void timeout()
    {
      if (!responded)
        forfeit({});
    }

    ~Responder()
    {
      timeout();
    }

  private:

    Implementation* const impl;
    const rmf_traffic::schedule::Version conflict_version;

    const rmf_traffic::schedule::Negotiation::TablePtr table;
    rmf_traffic::schedule::Version table_version;

    using OptVersion = rmf_utils::optional<rmf_traffic::schedule::Version>;
    const rmf_traffic::schedule::Negotiation::TablePtr parent;
    OptVersion parent_version;

    rclcpp::TimerBase::SharedPtr timer;
    mutable bool responded = false;

  };

  rclcpp::Node& node;
  std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer;
  std::shared_ptr<Worker> worker;
  rmf_traffic::Duration timeout = std::chrono::seconds(15);

  using Repeat = rmf_traffic_msgs::msg::NegotiationRepeat;
  using RepeatSub = rclcpp::Subscription<Repeat>;
  using RepeatPub = rclcpp::Publisher<Repeat>;
  RepeatSub::SharedPtr repeat_sub;
  RepeatPub::SharedPtr repeat_pub;

  using Notice = rmf_traffic_msgs::msg::NegotiationNotice;
  using NoticeSub = rclcpp::Subscription<Notice>;
  using NoticePub = rclcpp::Publisher<Notice>;
  NoticeSub::SharedPtr notice_sub;
  NoticePub::SharedPtr notice_pub;

  using Refusal = rmf_traffic_msgs::msg::NegotiationRefusal;
  using RefusalPub = rclcpp::Publisher<Refusal>;
  RefusalPub::SharedPtr refusal_pub;

  using Proposal = rmf_traffic_msgs::msg::NegotiationProposal;
  using ProposalSub = rclcpp::Subscription<Proposal>;
  using ProposalPub = rclcpp::Publisher<Proposal>;
  ProposalSub::SharedPtr proposal_sub;
  ProposalPub::SharedPtr proposal_pub;

  using Rejection = rmf_traffic_msgs::msg::NegotiationRejection;
  using RejectionSub = rclcpp::Subscription<Rejection>;
  using RejectionPub = rclcpp::Publisher<Rejection>;
  RejectionSub::SharedPtr rejection_sub;
  RejectionPub::SharedPtr rejection_pub;

  using Forfeit = rmf_traffic_msgs::msg::NegotiationForfeit;
  using ForfeitSub = rclcpp::Subscription<Forfeit>;
  using ForfeitPub = rclcpp::Publisher<Forfeit>;
  ForfeitSub::SharedPtr forfeit_sub;
  ForfeitPub::SharedPtr forfeit_pub;

  using Conclusion = rmf_traffic_msgs::msg::NegotiationConclusion;
  using ConclusionSub = rclcpp::Subscription<Conclusion>;
  ConclusionSub::SharedPtr conclusion_sub;

  using ParticipantAck = rmf_traffic_msgs::msg::NegotiationParticipantAck;
  using Ack = rmf_traffic_msgs::msg::NegotiationAck;
  using AckPub = rclcpp::Publisher<Ack>;
  AckPub::SharedPtr ack_pub;

  using NegotiatorMapPtr = std::shared_ptr<NegotiatorMap>;
  using WeakNegotiatorMapPtr = std::weak_ptr<NegotiatorMap>;
  NegotiatorMapPtr negotiators;

  using FailureMap = std::unordered_map<ParticipantId, std::function<void()>>;
  using FailureMapPtr = std::shared_ptr<FailureMap>;
  using WeakFailureMapPtr = std::weak_ptr<FailureMap>;
  FailureMapPtr failure_callbacks;

  using Version = rmf_traffic::schedule::Version;
  using Negotiation = rmf_traffic::schedule::Negotiation;
  struct Entry
  {
    bool participating;
    NegotiationRoom room;
  };

  using NegotiationMap = std::unordered_map<Version, Entry>;

  // The negotiations that this Negotiation class is involved in
  NegotiationMap negotiations;

  using TablePtr = rmf_traffic::schedule::Negotiation::TablePtr;
  using ItineraryVersion = rmf_traffic::schedule::ItineraryVersion;
  using UpdateVersion = rmf_utils::optional<ItineraryVersion>;
  struct CallbackEntry
  {
    Negotiation::VersionedKeySequence sequence;
    std::function<UpdateVersion()> callback;
  };

  using ApprovalCallbackMap = std::unordered_map<TablePtr, CallbackEntry>;
  using Approvals = std::unordered_map<Version, ApprovalCallbackMap>;
  Approvals approvals;

  // Status update callbacks
  using TableViewPtr = rmf_traffic::schedule::Negotiation::Table::ViewerPtr;
  using StatusUpdateCallback =
    std::function<void (uint64_t conflict_version, TableViewPtr& view)>;
  StatusUpdateCallback status_callback;

  using StatusConclusionCallback =
    std::function<void (uint64_t conflict_version, bool success)>;
  StatusConclusionCallback conclusion_callback;

  uint retained_history_count = 0;
  std::map<Version, rmf_traffic::schedule::Negotiation> history;

  Implementation(
    rclcpp::Node& node_,
    std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer_,
    std::shared_ptr<Worker> worker_)
  : node(node_),
    viewer(std::move(viewer_)),
    worker(std::move(worker_)),
    negotiators(std::make_shared<NegotiatorMap>()),
    failure_callbacks(std::make_shared<FailureMap>())
  {
    // TODO(MXG): Make the QoS configurable
    const auto qos = rclcpp::ServicesQoS().reliable().keep_last(1000);

    repeat_sub = node.create_subscription<Repeat>(
      NegotiationRepeatTopicName, qos,
      [&](const Repeat::UniquePtr msg)
      {
        this->receive_repeat_request(*msg);
      });

    repeat_pub = node.create_publisher<Repeat>(
      NegotiationRepeatTopicName, qos);

    notice_sub = node.create_subscription<Notice>(
      NegotiationNoticeTopicName, qos,
      [&](const Notice::UniquePtr msg)
      {
        this->receive_notice(*msg);
      });

    notice_pub = node.create_publisher<Notice>(
      NegotiationNoticeTopicName, qos);

    refusal_pub = node.create_publisher<Refusal>(
      NegotiationRefusalTopicName, qos);

    proposal_sub = node.create_subscription<Proposal>(
      NegotiationProposalTopicName, qos,
      [&](const Proposal::UniquePtr msg)
      {
        this->receive_proposal(*msg);
      });

    proposal_pub = node.create_publisher<Proposal>(
      NegotiationProposalTopicName, qos);

    rejection_sub = node.create_subscription<Rejection>(
      NegotiationRejectionTopicName, qos,
      [&](const Rejection::UniquePtr msg)
      {
        this->receive_rejection(*msg);
      });

    rejection_pub = node.create_publisher<Rejection>(
      NegotiationRejectionTopicName, qos);

    forfeit_sub = node.create_subscription<Forfeit>(
      NegotiationForfeitTopicName, qos,
      [&](const Forfeit::UniquePtr msg)
      {
        this->receive_forfeit(*msg);
      });

    forfeit_pub = node.create_publisher<Forfeit>(
      NegotiationForfeitTopicName, qos);

    conclusion_sub = node.create_subscription<Conclusion>(
      NegotiationConclusionTopicName, qos,
      [&](const Conclusion::UniquePtr msg)
      {
        this->receive_conclusion(*msg);
      });

    ack_pub = node.create_publisher<Ack>(
      NegotiationAckTopicName, qos);
  }

  void receive_repeat_request(const Repeat& msg)
  {
    // Ignore if it's asking for a repeat of the conflict notice
    if (msg.table.empty())
      return;

    // Ignore if we haven't heard of this negotiation
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
      return;

    // Ignore if we aren't participating in this negotiation
    if (!negotiate_it->second.participating)
      return;

    // Ignore if we aren't managing the relevant negotiator
    const ParticipantId for_participant = msg.table.back();
    const auto negotiator_it = negotiators->find(for_participant);
    if (negotiator_it == negotiators->end())
      return;

    auto to_accommodate = msg.table;
    to_accommodate.pop_back();

    const auto table = negotiate_it->second.room.negotiation.table(
      for_participant, to_accommodate);
    if (!table)
    {
      std::string error =
        "[rmf_traffic_ros2::schedule::Negotiation] A repeat was requested "
        "for a table that does not exist. Negotiation ["
        + std::to_string(msg.conflict_version) + "], table ["
        + table_to_string(msg.table) + " ]";

      RCLCPP_WARN(node.get_logger(), "%s", error.c_str());

      return;
    }

    publish_proposal(msg.conflict_version, *table);
  }

  void respond_to_queue(
    std::vector<TablePtr> queue,
    Version conflict_version)
  {
    while (!queue.empty())
    {
      const auto top = queue.back();
      queue.pop_back();

      if (top->defunct())
        continue;

      if (!top->submission())
      {
        const auto n_it = negotiators->find(top->participant());
        if (n_it == negotiators->end())
          continue;

        // TODO(MXG): Make this limit configurable
        if (top->version() > 3)
        {
          // Give up on this table at this point to avoid an infinite loop
          top->forfeit(top->version());
          publish_forfeit(conflict_version, *top);
          continue;
        }

        const auto& negotiator = n_it->second;
        negotiator->respond(
          top->viewer(), Responder::make(this, conflict_version, top));
      }

      if (top->submission())
      {
        for (const auto& c : top->children())
          queue.push_back(c);
      }
      else if (const auto& parent = top->parent())
      {
        if (parent->rejected())
          queue.push_back(parent);
      }
    }
  }

  void receive_notice(const Notice& msg)
  {
    bool relevant = false;
    for (const auto p : msg.participants)
    {
      if (negotiators->find(p) != negotiators->end())
      {
        relevant = true;
        break;
      }
    }


    auto new_negotiation = Negotiation::make(
      viewer->snapshot(), msg.participants);
    if (!new_negotiation)
    {
      // TODO(MXG): This is a temporary hack to deal with situations where a
      // conflict occurs before this node is aware of other relevant schedule
      // participants. We will refuse the negotiation and then expect it to be
      // retriggered some time later. Hopefully this node will know about the
      // schedule participant by then.
      //
      // A better system will be to use ReactiveX to observe the schedule and
      // try to open the negotiation again once the participant information is
      // available.
      Refusal refusal;
      refusal.conflict_version = msg.conflict_version;
      refusal_pub->publish(refusal);

      const auto n_it = negotiations.find(msg.conflict_version);
      if (n_it != negotiations.end())
        negotiations.erase(n_it);
      return;
    }

    const auto insertion = negotiations.insert(
      {msg.conflict_version, Entry{relevant, *std::move(new_negotiation)}});

    const bool is_new = insertion.second;
    bool& participating = insertion.first->second.participating;
    auto& room = insertion.first->second.room;
    Negotiation& negotiation = room.negotiation;

    if (!is_new)
    {
      const auto& old_participants = negotiation.participants();
      for (const auto p : msg.participants)
      {
        if (old_participants.count(p) == 0)
        {
          negotiation.add_participant(p);
        }
      }
    }

    if (!relevant)
    {
      // No response needed
      return;
    }

    // TODO(MXG): Is the participating flag even relevant?
    participating = true;

    std::vector<TablePtr> queue;
    for (const auto p : negotiation.participants())
      queue.push_back(negotiation.table(p, {}));

    respond_to_queue(queue, msg.conflict_version);
  }

  void receive_proposal(const Proposal& msg)
  {
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
    {
      // This negotiation has probably been completed already
      return;
    }

    const bool participating = negotiate_it->second.participating;
    auto& room = negotiate_it->second.room;
    Negotiation& negotiation = room.negotiation;
    const auto search =
      negotiation.find(msg.for_participant, convert(msg.to_accommodate));

    if (search.deprecated())
      return;

    const auto received_table = search.table;
    if (!received_table)
    {
      std::string error =
        "[rmf_traffic_ros2::schedule::Negotiation::receive_proposal] "
        "Receieved a proposal for negotiation ["
        + std::to_string(msg.conflict_version) + "] that builds on an "
        "unknown table: [";
      for (const auto p : msg.to_accommodate)
        error += " " + std::to_string(p.participant) + ":" + std::to_string(
          p.version);
      error += " " + std::to_string(msg.for_participant) + " ]";

      RCLCPP_WARN(node.get_logger(), "%s", error.c_str());
      room.cached_proposals.push_back(msg);
      return;
    }

    // We'll keep track of these negotiations whether or not we're participating
    // in them, because one of our negotiators might get added to it in the
    // future
    const bool updated = received_table->submit(
      msg.plan_id, convert(msg.itinerary), msg.proposal_version);

    if (!updated)
      return;

    if (status_callback)
    {
      auto table_view = received_table->viewer();
      status_callback(msg.conflict_version, table_view);
    }

    std::vector<TablePtr> queue = room.check_cache(*negotiators);

    if (!participating)
      return;

    for (const auto& n : *negotiators)
    {
      if (const auto respond_to = received_table->respond(n.first))
        queue.push_back(respond_to);
    }

    respond_to_queue(queue, msg.conflict_version);
  }

  void receive_rejection(const Rejection& msg)
  {
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
    {
      // We don't need to worry about caching an unknown rejection, because it
      // is impossible for a proposal that was produced by this negotiation
      // instance to be rejected without us being aware of that proposal.
      return;
    }

    auto& room = negotiate_it->second.room;
    Negotiation& negotiation = room.negotiation;
    const auto search = negotiation.find(convert(msg.table));

    if (search.deprecated())
      return;

    const auto table = search.table;
    if (!table)
    {
      std::string error =
        "[rmf_traffic_ros2::schedule::Negotiation::receive_rejection] "
        "Receieved a rejection for negotiation ["
        + std::to_string(msg.conflict_version) + "] for an "
        "unknown table: [";
      for (const auto p : msg.table)
        error += " " + std::to_string(p.participant) + ":" + std::to_string(
          p.version);
      error += " ]";

      RCLCPP_WARN(node.get_logger(), "%s", error.c_str());

      room.cached_rejections.push_back(msg);
      return;
    }

    const bool updated = table->reject(
      msg.table.back().version, msg.rejected_by, convert(msg.alternatives));

    if (!updated)
      return;

    if (status_callback)
    {
      auto table_view = table->viewer();
      status_callback(msg.conflict_version, table_view);
    }

    std::vector<TablePtr> queue = room.check_cache(*negotiators);

    if (!negotiate_it->second.participating)
      return;

    queue.push_back(table);
    respond_to_queue(queue, msg.conflict_version);
  }

  void receive_forfeit(const Forfeit& msg)
  {
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
    {
      // We don't need to worry about caching an unknown rejection, because it
      // is impossible for a proposal that was produced by this negotiation
      // instance to be rejected without us being aware of that proposal.
      return;
    }

    auto& room = negotiate_it->second.room;
    Negotiation& negotiation = room.negotiation;
    const auto search = negotiation.find(convert(msg.table));
    if (search.deprecated())
      return;

    const auto table = search.table;
    if (!table)
    {
      room.cached_forfeits.push_back(msg);
      return;
    }

    table->forfeit(msg.table.back().version);

    if (status_callback)
    {
      auto table_view = table->viewer();
      status_callback(msg.conflict_version, table_view);
    }

    respond_to_queue(room.check_cache(*negotiators), msg.conflict_version);
  }

  void dump_conclusion_info(
    const Conclusion& msg,
    const Approvals::const_iterator& approval_callback_it,
    const Negotiation& negotiation)
  {
    const auto full_sequence = convert(msg.table);

    std::string err =
      "\n !!!!!!!!!!!! Impossible situation encountered for Negotiation ["
      + std::to_string(msg.conflict_version) + "] in node ["
      + node.get_name() + "]: No approval callbacks found?? Sequence: [";
    for (const auto s : msg.table)
      err += " " + std::to_string(s.participant) + ":" + std::to_string(
        s.version);
    err += " ] ";

    if (msg.resolved)
    {
      err += "Tables with acknowledgments for this negotiation:";

      const auto& approval_callbacks = approval_callback_it->second;
      for (const auto& cb : approval_callbacks)
      {
        const auto table = cb.first;
        err += "\n -- " + ptr_to_string(table.get()) + " |";

        for (const auto& s : table->sequence())
          err += " " + std::to_string(s.participant) + ":" + std::to_string(
            s.version);
      }

      err += "\nCurrent relevant tables in the negotiation:";
      for (std::size_t i = 1; i <= msg.table.size(); ++i)
      {
        std::vector<ParticipantId> sequence;
        for (std::size_t j = 0; j < i; ++j)
          sequence.push_back(full_sequence[j].participant);

        const auto table = negotiation.table(sequence);
        err += "\n -- " + ptr_to_string(table.get()) + " |";

        for (std::size_t j = 0; j < i; ++j)
          err += " " + std::to_string(msg.table[j].participant) + ":" +
            std::to_string(msg.table[j].version);
      }
    }
    else
    {
      err += "Negotiation participants for this node:";
      for (const auto p : negotiation.participants())
      {
        if (negotiators->count(p) != 0)
          err += " " + std::to_string(p);
      }
    }

    print_negotiation_status(msg.conflict_version, negotiation);
  }

  void receive_conclusion(const Conclusion& msg)
  {
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
    {
      // We don't need to worry about concluding unknown negotiations
      return;
    }

    const bool participating = negotiate_it->second.participating;
    auto& room = negotiate_it->second.room;
    Negotiation& negotiation = room.negotiation;
    const auto full_sequence = convert(msg.table);

    if (participating)
    {
      std::vector<ParticipantAck> acknowledgments;

      const auto approval_callback_it = approvals.find(msg.conflict_version);
      if (msg.resolved)
      {
        assert(approval_callback_it != approvals.end());
        const auto& approval_callbacks = approval_callback_it->second;

        for (std::size_t i = 1; i <= msg.table.size(); ++i)
        {
          const auto sequence = Negotiation::VersionedKeySequence(
            full_sequence.begin(), full_sequence.begin()+i);
          const auto participant = sequence.back().participant;

          const auto search = negotiation.find(sequence);
          if (search.absent())
          {
            // If the Negotiation never knew about this sequence, then we cannot
            // possibly have any approval callbacks waiting for it. This may
            // happen towards the end of a Negotiation sequence if the remaining
            // tables belong to a different node.
            break;
          }

          auto approve_it = approval_callbacks.end();
          if (search)
          {
            approve_it = approval_callbacks.find(search.table);
          }
          else
          {
            assert(search.deprecated());
            // The final table was somehow deprecated. In principle this
            // shouldn't happen if all Negotiation participants are "consistent"
            // about what they propose. However, due to race conditions and
            // implementation details, we cannot guarantee consistency.
            // Therefore this situation is possible, and we should just try our
            // best to cope with it gracefully.

            if (negotiators->find(participant) == negotiators->end())
            {
              // We do not have a negotiator for this participant, so there is
              // no point searching for an approval callback for it.
              continue;
            }

            // We will do a brute-force search through our approval callbacks
            // to dig up the relevant one.
            for (auto a_it = approval_callbacks.begin();
              a_it != approval_callbacks.end();
              ++a_it)
            {
              if (a_it->second.sequence == sequence)
              {
                approve_it = a_it;
                break;
              }
            }
          }

          if (approve_it != approval_callbacks.end())
          {
            const auto& entry = approve_it->second;
            ParticipantAck p_ack;
            p_ack.participant = entry.sequence.back().participant;
            p_ack.updating = false;
            const auto& approval_cb = entry.callback;
            if (approval_cb)
            {
              const auto update_version = approval_cb();
              if (update_version)
              {
                p_ack.updating = true;
                p_ack.itinerary_version = *update_version;
              }
            }

            acknowledgments.emplace_back(std::move(p_ack));
          }
          else
          {
            // If we couldn't find an approval callback for this table of the
            // conclusion, then that should not be a negotiator for the table.
            if (negotiators->find(participant) != negotiators->end())
            {
              dump_conclusion_info(msg, approval_callback_it, negotiation);
              assert(negotiators->find(participant) == negotiators->end());
            }
          }
        }
      }
      else
      {
        for (const auto p : room.negotiation.participants())
        {
          // If we have a failure callback for one of the participants in this
          // failed negotiation, then trigger its failure callback.
          const auto f_it = failure_callbacks->find(p);
          if (f_it != failure_callbacks->end())
          {
            const auto& callback = f_it->second;
            if (callback)
              callback();
          }
        }

        ParticipantAck p_ack;
        p_ack.updating = false;
        for (const auto p : negotiation.participants())
        {
          if (negotiators->count(p) != 0)
          {
            p_ack.participant = p;
            acknowledgments.push_back(p_ack);
          }
        }
      }

      // Acknowledge that we know about this conclusion
      Ack ack;
      ack.conflict_version = msg.conflict_version;
      ack.acknowledgments = std::move(acknowledgments);

      if (ack.acknowledgments.empty())
      {
        // If we are participating in this negotiation, then the
        // acknowledgments must not be empty, or else there is a bug somewhere.
        dump_conclusion_info(msg, approval_callback_it, negotiation);
        assert(!ack.acknowledgments.empty());
      }

      ack_pub->publish(ack);
      // TODO(MXG): Should we consider a more robust cache cleanup strategy?

      if (approval_callback_it != approvals.end())
        approvals.erase(approval_callback_it);
    }

    if (conclusion_callback)
      conclusion_callback(msg.conflict_version, msg.resolved);

    // add to retained history
    if (retained_history_count > 0)
    {
      while (history.size() >= retained_history_count)
        history.erase(history.begin());

      history.emplace(
        msg.conflict_version, std::move(negotiate_it->second.room.negotiation));
    }

    // Erase these entries because the negotiation has concluded
    negotiations.erase(negotiate_it);
  }

  void publish_proposal(
    const Version conflict_version,
    const Negotiation::Table& table)
  {
    Proposal msg;
    msg.conflict_version = conflict_version;
    msg.proposal_version = table.version();

    assert(table.submission());
    msg.itinerary = convert(*table.submission());
    msg.for_participant = table.participant();
    msg.to_accommodate = convert(table.sequence());

    // Make sure to pop the back off of msg.to_accommodate, because we don't
    // want to include this table's final sequence key. That final key is
    // provided by for_participant.
    msg.to_accommodate.pop_back();

    proposal_pub->publish(msg);
  }

  void publish_rejection(
    const Version conflict_version,
    const Negotiation::Table& table,
    const ParticipantId rejected_by,
    const Negotiation::Alternatives& alternatives)
  {
    Rejection msg;
    msg.conflict_version = conflict_version;
    msg.table = convert(table.sequence());
    msg.rejected_by = rejected_by;
    msg.alternatives = convert(alternatives);

    rejection_pub->publish(msg);
  }

  void publish_forfeit(
    const Version conflict_version,
    const Negotiation::Table& table)
  {
    Forfeit msg;
    msg.conflict_version = conflict_version;
    msg.table = convert(table.sequence());

    forfeit_pub->publish(msg);
  }

  struct Handle
  {
    Handle(
      const ParticipantId for_participant_,
      NegotiatorMapPtr negotiators,
      FailureMapPtr failure)
    : for_participant(for_participant_),
      weak_negotiator_map(negotiators),
      weak_failure_map(failure)
    {
      // Do nothing
    }

    ParticipantId for_participant;
    WeakNegotiatorMapPtr weak_negotiator_map;
    WeakFailureMapPtr weak_failure_map;

    ~Handle()
    {
      if (const auto map = weak_negotiator_map.lock())
        map->erase(for_participant);

      if (const auto map = weak_failure_map.lock())
        map->erase(for_participant);
    }
  };

  std::shared_ptr<void> register_negotiator(
    const ParticipantId for_participant,
    NegotiatorPtr negotiator,
    std::function<void()> failure_cb)
  {
    const auto insertion = negotiators->insert(
      std::make_pair(for_participant, std::move(negotiator)));

    if (!insertion.second)
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[rmf_traffic_ros2::schedule::Negotiaton] Attempt to register a "
        "duplicate negotiator for participant ["
        + std::to_string(for_participant) + "]");
      // *INDENT-ON*
    }

    if (failure_cb)
    {
      failure_callbacks->insert(
        std::make_pair(for_participant, std::move(failure_cb)));
    }

    return std::make_shared<Handle>(
      for_participant, negotiators, failure_callbacks);
  }

  void set_retained_history_count(uint count)
  {
    retained_history_count = count;
  }

  TableViewPtr table_view(
    uint64_t conflict_version,
    const std::vector<ParticipantId>& sequence) const
  {
    const auto negotiate_it = negotiations.find(conflict_version);
    rmf_traffic::schedule::Negotiation::ConstTablePtr table;

    if (negotiate_it == negotiations.end())
    {
      const auto history_it = history.find(conflict_version);
      if (history_it == history.end())
      {
        RCLCPP_WARN(
          node.get_logger(),
          "Conflict version %lu does not exist."
          "It may have been successful and wiped",
          conflict_version);
        return nullptr;
      }

      table = history_it->second.table(sequence);
    }
    else
    {
      const auto& room = negotiate_it->second.room;
      const Negotiation& negotiation = room.negotiation;
      table = negotiation.table(sequence);
    }

    if (!table)
    {
      RCLCPP_WARN(node.get_logger(), "Table not found");
      return nullptr;
    }

    return table->viewer();
  }

  void on_status_update(StatusUpdateCallback cb)
  {
    status_callback = cb;
  }

  void on_conclusion(StatusConclusionCallback cb)
  {
    conclusion_callback = cb;
  }
};

//==============================================================================
Negotiation::Negotiation(
  rclcpp::Node& node,
  std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer,
  std::shared_ptr<Worker> worker)
: _pimpl(rmf_utils::make_unique_impl<Implementation>(
      node, std::move(viewer), std::move(worker)))
{
  // Do nothing
}

//==============================================================================
void Negotiation::on_status_update(StatusUpdateCallback cb)
{
  _pimpl->on_status_update(cb);
}

//==============================================================================
void Negotiation::on_conclusion(StatusConclusionCallback cb)
{
  _pimpl->on_conclusion(cb);
}

//==============================================================================
Negotiation& Negotiation::timeout_duration(rmf_traffic::Duration duration)
{
  _pimpl->timeout = duration;
  return *this;
}

//==============================================================================
rmf_traffic::Duration Negotiation::timeout_duration() const
{
  return _pimpl->timeout;
}

//==============================================================================
Negotiation::TableViewPtr Negotiation::table_view(
  uint64_t conflict_version,
  const std::vector<ParticipantId>& sequence) const
{
  return _pimpl->table_view(conflict_version, sequence);
}

//==============================================================================
void Negotiation::set_retained_history_count(uint count)
{
  return _pimpl->set_retained_history_count(count);
}

//==============================================================================
std::shared_ptr<void> Negotiation::register_negotiator(
  rmf_traffic::schedule::ParticipantId for_participant,
  std::unique_ptr<rmf_traffic::schedule::Negotiator> negotiator)
{
  return _pimpl->register_negotiator(
    for_participant, std::move(negotiator), nullptr);
}

//==============================================================================
std::shared_ptr<void> Negotiation::register_negotiator(
  rmf_traffic::schedule::ParticipantId for_participant,
  std::unique_ptr<rmf_traffic::schedule::Negotiator> negotiator,
  std::function<void()> on_negotiation_failure)
{
  return _pimpl->register_negotiator(
    for_participant, std::move(negotiator), std::move(on_negotiation_failure));
}

//==============================================================================
class LambdaNegotiator : public rmf_traffic::schedule::Negotiator
{
public:

  using RespondFn = std::function<void(TableViewerPtr, ResponderPtr)>;

  LambdaNegotiator(RespondFn respond)
  : _respond(std::move(respond))
  {
    // Do nothing
  }

  void respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder) final
  {
    _respond(std::move(table_viewer), std::move(responder));
  }

private:
  RespondFn _respond;
};

//==============================================================================
std::shared_ptr<void> Negotiation::register_negotiator(
  rmf_traffic::schedule::ParticipantId for_participant,
  std::function<void(TableViewPtr, ResponderPtr)> respond,
  std::function<void()> on_negotiation_failure)
{
  return register_negotiator(
    for_participant,
    std::make_unique<LambdaNegotiator>(std::move(respond)),
    std::move(on_negotiation_failure));
}

} // namespace schedule
} // namespace rmf_traffic_ros2
