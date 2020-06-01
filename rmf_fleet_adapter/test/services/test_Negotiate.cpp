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

#include <services/Negotiate.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_utils/catch.hpp>

// Helper Definitions
//==============================================================================
namespace {
using VertexId = std::string;
using IsHoldingSpot = bool;
using VertexMap = std::unordered_map<VertexId, std::pair<Eigen::Vector2d,
    IsHoldingSpot>>;

using EdgeId = std::string;
using EdgeVertices = std::pair<VertexId, VertexId>;
using IsBidirectional = bool;
using EdgeMap = std::unordered_map<EdgeId, std::pair<EdgeVertices,
    IsBidirectional>>;

using VertexIdtoIdxMap = std::unordered_map<VertexId, size_t>;

using ParticipantName = std::string;
using ParticipantIndex = size_t;

struct ParticipantConfig
{
  const rmf_traffic::Profile profile;
  const rmf_traffic::agv::VehicleTraits traits;
  const rmf_traffic::schedule::ParticipantDescription description;
};

// Helper Functions
//==============================================================================
// Makes graph using text ids, and returns bookkeeping that maps ids to indices
// TODO(BH): Perhaps some sort of book keeping can be introduced in Graph itself?
inline std::pair<rmf_traffic::agv::Graph, VertexIdtoIdxMap>
generate_test_graph_data(std::string map_name, VertexMap vertices,
  EdgeMap edges)
{
  rmf_traffic::agv::Graph graph;
  // Maps vertex id to its corresponding id in the rmf_traffic::agv::Graph
  VertexIdtoIdxMap vertex_id_to_idx;
  size_t current_idx = 0; // vertex idxes are added in monotonic increments

  for (auto it = vertices.cbegin(); it != vertices.cend(); it++)
  {
    // Adding to rmf_traffic::agv::Graph
    graph.add_waypoint(map_name, it->second.first)
        .set_holding_point(it->second.second);

    // Book keeping
    vertex_id_to_idx.insert({it->first, current_idx});
    current_idx++;
  }

  for (auto it = edges.cbegin(); it != edges.cend(); it++)
  {
    auto source_vtx = vertex_id_to_idx[it->second.first.first];
    auto sink_vtx = vertex_id_to_idx[it->second.first.second];
    graph.add_lane(source_vtx, sink_vtx);
    if (it->second.second)
    {
      graph.add_lane(sink_vtx, source_vtx);
    }
  }
  return std::pair<rmf_traffic::agv::Graph, VertexIdtoIdxMap>(graph,
      vertex_id_to_idx);
}

inline rmf_utils::optional<rmf_traffic::schedule::Itinerary> get_participant_itinerary(
  rmf_traffic::schedule::Negotiation::Proposal proposal,
  rmf_traffic::schedule::ParticipantId participant_id)
{
  for (auto submission : proposal)
  {
    if (submission.participant == participant_id)
    {
      return submission.itinerary;
    }
  }
  return rmf_utils::nullopt;
}

// Preset Robot Configurations
// Agent a(i) generates participant p(i), instantiated in tests
//==============================================================================
// a0
const rmf_traffic::Profile a0_profile{
  rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0)
};

const rmf_traffic::agv::VehicleTraits a0_traits{
  {0.7, 0.3},
  {1.0, 0.45},
  a0_profile
};

const rmf_traffic::schedule::ParticipantDescription a0_description = {
  "p0",
  "test_Negotiator",
  rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
  a0_profile
};

const ParticipantConfig a0_config = {
  a0_profile, a0_traits, a0_description
};

// a1
const rmf_traffic::Profile a1_profile{
  rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0)
};

const rmf_traffic::agv::VehicleTraits a1_traits{
  {0.7, 0.3},
  {1.0, 0.45},
  a1_profile
};

const rmf_traffic::schedule::ParticipantDescription a1_description = {
  "p1",
  "test_Negotiator",
  rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
  a1_profile
};

const ParticipantConfig a1_config = {
  a1_profile, a1_traits, a1_description
};

// a2 - A slower version of a0 / a1
const rmf_traffic::Profile a2_profile{
  rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0)
};

const rmf_traffic::agv::VehicleTraits a2_traits{
  {0.3, 0.1},
  {0.5, 0.2},
  a2_profile
};

const rmf_traffic::schedule::ParticipantDescription a2_description = {
  "p2",
  "test_Negotiator",
  rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
  a2_profile
};

const ParticipantConfig a2_config = {
  a2_profile, a2_traits, a2_description
};

//==============================================================================
template<typename TableViewPtr>
std::string to_string(const TableViewPtr& table)
{
  std::string out = "[";
  for (const auto& p : table->sequence())
  {
    out += " " + std::to_string(p.participant)
        + ":" + std::to_string(p.version);
  }
  out += " ]";

  return out;
}

//==============================================================================
class TestPathNegotiator
    : public rmf_traffic::schedule::Negotiator,
      public std::enable_shared_from_this<TestPathNegotiator>
{
public:

  class ResponderWrapper : public Responder
  {
  public:

    ResponderWrapper(
        std::shared_ptr<TestPathNegotiator> negotiator,
        ResponderPtr responder)
      : _negotiator(negotiator),
        _responder(std::move(responder))
    {
      // Do nothing
    }

    template<typename... Args>
    static std::shared_ptr<ResponderWrapper> make(Args&&... args)
    {
      return std::make_shared<ResponderWrapper>(std::forward<Args>(args)...);
    }

    void set_job(std::shared_ptr<rmf_fleet_adapter::services::Negotiate> job)
    {
      _job = std::move(job);
    }

    void submit(
        std::vector<rmf_traffic::Route> itinerary,
        ApprovalCallback approval_callback = nullptr) const final
    {
      _responder->submit(std::move(itinerary), std::move(approval_callback));
      _clear_job();
    }

    void reject(const Alternatives& alternatives) const final
    {
      _responder->reject(alternatives);
      _clear_job();
    }

    void forfeit(const std::vector<ParticipantId>& blockers) const final
    {
      _responder->forfeit(blockers);
      _clear_job();
    }

  private:

    void _clear_job() const
    {
      if (const auto n = _negotiator.lock())
      {
        if (const auto job = _job.lock())
          n->_negotiate_jobs.erase(job);
      }
    }

    std::weak_ptr<TestPathNegotiator> _negotiator;
    std::weak_ptr<rmf_fleet_adapter::services::Negotiate> _job;
    ResponderPtr _responder;
  };

  TestPathNegotiator(
      std::shared_ptr<rmf_traffic::agv::Planner> planner,
      rmf_traffic::agv::Plan::StartSet starts,
      rmf_traffic::agv::Plan::Goal goal)
    : _planner(std::move(planner)),
      _starts(std::move(starts)),
      _goal(std::move(goal))
  {
    // Do nothing
  }

  TestPathNegotiator& print(bool on)
  {
    _print = on;
    return *this;
  }

  TestPathNegotiator& n(std::shared_ptr<rmf_traffic::schedule::Negotiation> negotiation)
  {
    _n = std::move(negotiation);
    return *this;
  }

  void respond(
      const TableViewerPtr& table_viewer,
      const ResponderPtr& responder,
      const bool * = nullptr) final
  {
    if (_print)
    {
      std::cout << "    Responding to " << to_string(table_viewer)
                << " (" << _n.get() << ")" << std::endl;
    }

    auto responder_wrap = ResponderWrapper::make(shared_from_this(), responder);
    auto negotiate = rmf_fleet_adapter::services::Negotiate::path(
          _planner, _starts, _goal, table_viewer, responder_wrap, nullptr);
    responder_wrap->set_job(negotiate);

    _negotiate_jobs.insert(negotiate);

    auto sub = rmf_rxcpp::make_job<rmf_fleet_adapter::services::Negotiate::Result>(
          negotiate)
        .observe_on(rxcpp::observe_on_event_loop())
        .subscribe(
          [](const auto& result)
    {
      result();
    });

    _subscriptions.emplace_back(std::move(sub));
  }

private:
  std::shared_ptr<rmf_traffic::agv::Planner> _planner;
  rmf_traffic::agv::Plan::StartSet _starts;
  rmf_traffic::agv::Plan::Goal _goal;
  std::vector<rxcpp::subscription> _subscriptions;
  bool _print = false;
  std::shared_ptr<rmf_traffic::schedule::Negotiation> _n;

  using NegotiateSet =
    std::unordered_set<
      std::shared_ptr<rmf_fleet_adapter::services::Negotiate>
    >;

  NegotiateSet _negotiate_jobs;

};

//==============================================================================
class NegotiationRoom : public std::enable_shared_from_this<NegotiationRoom>
{
public:

  using ParticipantId = rmf_traffic::schedule::ParticipantId;
  using Negotiator = TestPathNegotiator;
  using Negotiation = rmf_traffic::schedule::Negotiation;

  class Responder : public rmf_traffic::schedule::Negotiator::Responder
  {
  public:
    Responder(
        std::shared_ptr<NegotiationRoom> room,
        rmf_traffic::schedule::Negotiation::TablePtr table)
      : _room(std::move(room)),
        _table(std::move(table))
    {
      // Do nothing
    }

    template<typename... Args>
    static std::shared_ptr<Responder> make(Args&&... args)
    {
      return std::make_shared<Responder>(std::forward<Args>(args)...);
    }

    void submit(
        std::vector<rmf_traffic::Route> itinerary,
        ApprovalCallback approval_callback = nullptr) const final
    {
      if (!_table->ongoing() && _room->_print)
      {
        std::cout << "Deprecated negotiation: ";
      }

      if (_table->defunct())
      {
        std::cout << "Defunct " << to_string(_table) << " ("
                  << _room->negotiation.get() << ")" << std::endl;
        return;
      }

      rmf_traffic::schedule::SimpleResponder(_table)
          .submit(std::move(itinerary), std::move(approval_callback));

      if (_room->_print)
      {
        std::cout << "Submission given for " + to_string(_table)
                  << " (" << _room->negotiation.get() << ")" << std::endl;
      }

      if (_room->check_finished())
        return;

      for (const auto& n : _room->negotiators)
      {
        const auto participant = n.first;
        const auto respond_to = _table->respond(participant);
        if (respond_to)
        {
          if (skip(respond_to))
          {
            if (_room->_print)
            {
              std::cout << "    Skipping a response request from "
                        << to_string(respond_to) << std::endl;
            }

            respond_to->forfeit(respond_to->version());
            continue;
          }

          n.second->print(_room->_print).n(_room->negotiation).respond(
                respond_to->viewer(), make(_room, respond_to));
        }
      }
    }

    void reject(const Alternatives& alternatives) const final
    {
      if (_table->defunct())
        return;

      rmf_traffic::schedule::SimpleResponder(_table).reject(alternatives);

      if (_room->check_finished())
        return;

      const auto parent = _table->parent();
      if (parent)
      {
        if (_room->_print)
        {
          std::cout << std::to_string(_table->participant()) << " rejected "
                    << to_string(parent) << std::endl;
        }

        if (skip(parent))
        {
          parent->forfeit(parent->version());
          return;
        }

        _room->negotiators.at(parent->participant())
            ->print(_room->_print).n(_room->negotiation).respond(
              parent->viewer(), make(_room, parent));
      }
    }

    void forfeit(const std::vector<ParticipantId>& blockers) const final
    {
      if (_table->defunct())
        return;

      rmf_traffic::schedule::SimpleResponder(_table).forfeit(blockers);

      if (_room->_print)
      {
        std::cout << "Forfeit given for " << to_string(_table)
                  << " with the following blockers:";
        for (const auto p : blockers)
          std::cout << " " << p << std::endl;
      }
    }

  private:
    std::shared_ptr<NegotiationRoom> _room;
    rmf_traffic::schedule::Negotiation::TablePtr _table;
  };

  struct Intention
  {
    std::vector<rmf_traffic::agv::Planner::Start> start;
    rmf_traffic::agv::Planner::Goal goal;
    std::shared_ptr<rmf_traffic::agv::Planner> planner;

    Intention(
      std::vector<rmf_traffic::agv::Planner::Start> starts_,
      rmf_traffic::agv::Planner::Goal goal_,
      std::shared_ptr<rmf_traffic::agv::Planner> planner_)
    : start(std::move(starts_)),
      goal(std::move(goal_)),
      planner(std::move(planner_))
    {
      // Do nothing
    }

    Intention(
      rmf_traffic::agv::Planner::Start start_,
      rmf_traffic::agv::Planner::Goal goal_,
      std::shared_ptr<rmf_traffic::agv::Planner> planner_)
    : start({std::move(start_)}),
      goal(std::move(goal_)),
      planner(std::move(planner_))
    {
      // Do nothing
    }
  };

  using Intentions = std::unordered_map<ParticipantId, Intention>;

  NegotiationRoom(
    std::shared_ptr<const rmf_traffic::schedule::Viewer> viewer,
    Intentions intentions,
    const bool print = false)
  : negotiators(make_negotiators(intentions)),
    negotiation(Negotiation::make_shared(
        std::move(viewer), get_participants(intentions))),
    _print(print)
  {
    // Do nothing
  }

  static bool skip(const Negotiation::TablePtr& table)
  {
    if (table->submission() && !table->rejected())
      return true;

    // Give up we have already attempted more than 2 submissions
    if (table->version() > 2)
      return true;

    auto ancestor = table->parent();
    while (ancestor)
    {
      if (ancestor->rejected() || ancestor->forfeited())
        return true;

      ancestor = ancestor->parent();
    }

    return false;
  }

  using Proposal = rmf_traffic::schedule::Negotiation::Proposal;
  using OptProposal = rmf_utils::optional<Proposal>;

  std::future<OptProposal> solve()
  {
    if (_print)
    {
      std::cout << "Beginning negotiation for ("
                << negotiation.get() << ")" <<std::endl;
    }

    for (const auto& n : negotiators)
    {
      const auto participant = n.first;
      const auto table = negotiation->table(participant, {});
      if (!table)
        continue;

      const auto& negotiator = n.second;
      negotiator->print(_print).n(negotiation).respond(
            table->viewer(),
            Responder::make(shared_from_this(), table));
    }

    return _solution.get_future();
  }

  static std::unordered_map<ParticipantId, std::shared_ptr<Negotiator>>
  make_negotiators(
    const std::unordered_map<ParticipantId, Intention>& intentions)
  {
    std::unordered_map<ParticipantId, std::shared_ptr<Negotiator>> negotiators;
    for (const auto& entry : intentions)
    {
      const auto participant = entry.first;
      const auto& intention = entry.second;
      negotiators.insert(
        std::make_pair(
          participant,
          std::make_shared<Negotiator>(
                intention.planner, intention.start, intention.goal)));
    }

    return negotiators;
  }

  static std::vector<ParticipantId> get_participants(
    const std::unordered_map<ParticipantId, Intention>& intentions)
  {
    std::vector<ParticipantId> participants;
    participants.reserve(intentions.size());
    for (const auto& entry : intentions)
      participants.push_back(entry.first);

    return participants;
  }

  bool check_finished()
  {
    if (!negotiation->ready() && !negotiation->complete())
      return false;

    if (!_finished)
    {
      _finished = true;
      if (negotiation->ready())
      {
        _solution.set_value(
              negotiation->evaluate(
                rmf_traffic::schedule::QuickestFinishEvaluator())->proposal());
      }
      else
      {
        _solution.set_value(rmf_utils::nullopt);
      }
    }

    return true;
  }

  std::unordered_map<ParticipantId, std::shared_ptr<Negotiator>> negotiators;
  std::shared_ptr<rmf_traffic::schedule::Negotiation> negotiation;

  bool _finished = false;

  std::promise<OptProposal> _solution;


  NegotiationRoom& print()
  {
    _print = true;
    return *this;
  }

  bool _print = false;
};
} // anonymous namespace

SCENARIO("fan-in-fan-out bottleneck")
{
  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_fan_in_fan_out";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *        fan_in_fan_out (All lengths are 3)
   *
   *        A       B(H)    C      D(H)    E
   *        ^       ^       ^      ^       ^
   *        |       |       |      |       |
   *        v       v       v      v       v
   *        A'<---> B'<---> C'<--->D'<---> E'
   *                        ^
   *                        |
   *                        v
   *                        F
   *                        ^
   *                        |
   *                        v
   *        V'<---> W'<---> X'<--->Y'<---> Z'
   *        ^       ^       ^      ^       ^
   *        |       |       |      |       |
   *        v       v       v      v       v
   *        V       W(H)    X      Y(H)    Z
   *
   *
   *
   */

  // Vertices
  vertices.insert({"A", {{-6.0, 6.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{-3.0, 6.0}, IsHoldingSpot(true)}});
  vertices.insert({"C", {{0.0, 6.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{3.0, 6.0}, IsHoldingSpot(true)}});
  vertices.insert({"E", {{6.0, 6.0}, IsHoldingSpot(false)}});

  vertices.insert({"A'", {{-6.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"B'", {{-3.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"C'", {{0.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"D'", {{3.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"E'", {{6.0, 3.0}, IsHoldingSpot(false)}});

  vertices.insert({"F", {{0.0, 0.0}, IsHoldingSpot(false)}});

  vertices.insert({"V'", {{-6.0, -3.0}, IsHoldingSpot(false)}});
  vertices.insert({"W'", {{-3.0, -3.0}, IsHoldingSpot(false)}});
  vertices.insert({"X'", {{0.0, -3.0}, IsHoldingSpot(false)}});
  vertices.insert({"Y'", {{3.0, -3.0}, IsHoldingSpot(false)}});
  vertices.insert({"Z'", {{6.0, -3.0}, IsHoldingSpot(false)}});

  vertices.insert({"V", {{-6.0, -6.0}, IsHoldingSpot(false)}});
  vertices.insert({"W", {{-3.0, -6.0}, IsHoldingSpot(true)}});
  vertices.insert({"X", {{0.0, -6.0}, IsHoldingSpot(false)}});
  vertices.insert({"Y", {{3.0, -6.0}, IsHoldingSpot(true)}});
  vertices.insert({"Z", {{6.0, -6.0}, IsHoldingSpot(false)}});

  // Edges
  edges.insert({"AA'", {{"A", "A'"}, IsBidirectional(true)}});
  edges.insert({"BB'", {{"B", "B'"}, IsBidirectional(true)}});
  edges.insert({"CC'", {{"C", "C'"}, IsBidirectional(true)}});
  edges.insert({"DD'", {{"D", "D'"}, IsBidirectional(true)}});
  edges.insert({"EE'", {{"E", "E'"}, IsBidirectional(true)}});

  edges.insert({"A'B'", {{"A'", "B'"}, IsBidirectional(true)}});
  edges.insert({"B'C'", {{"B'", "C'"}, IsBidirectional(true)}});
  edges.insert({"C'D'", {{"C'", "D'"}, IsBidirectional(true)}});
  edges.insert({"D'E'", {{"D'", "E'"}, IsBidirectional(true)}});

  edges.insert({"VV'", {{"V", "V'"}, IsBidirectional(true)}});
  edges.insert({"WW'", {{"W", "W'"}, IsBidirectional(true)}});
  edges.insert({"XX'", {{"X", "X'"}, IsBidirectional(true)}});
  edges.insert({"YY'", {{"Y", "Y'"}, IsBidirectional(true)}});
  edges.insert({"ZZ'", {{"Z", "Z'"}, IsBidirectional(true)}});

  edges.insert({"V'W'", {{"V'", "W'"}, IsBidirectional(true)}});
  edges.insert({"W'X'", {{"W'", "X'"}, IsBidirectional(true)}});
  edges.insert({"X'Y'", {{"X'", "Y'"}, IsBidirectional(true)}});
  edges.insert({"Y'Z'", {{"Y'", "Z'"}, IsBidirectional(true)}});

  edges.insert({"C'F", {{"C'", "F"}, IsBidirectional(true)}});
  edges.insert({"X'F", {{"X'", "F"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  auto set_passthrough_point = [&](const std::string& wp)
  {
    graph.get_waypoint(vertex_id_to_idx[wp]).set_passthrough_point(true);
  };
  set_passthrough_point("A'");
  set_passthrough_point("B'");
  set_passthrough_point("C'");
  set_passthrough_point("D'");
  set_passthrough_point("E'");
  set_passthrough_point("F");
  set_passthrough_point("V'");
  set_passthrough_point("W'");
  set_passthrough_point("X'");
  set_passthrough_point("Y'");
  set_passthrough_point("Z'");

  const auto a0_planner = std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Plan::Configuration(
          graph, a0_config.traits),
        rmf_traffic::agv::Plan::Options(nullptr));

  const auto a1_planner = std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Plan::Configuration(
          graph, a1_config.traits),
        rmf_traffic::agv::Plan::Options(nullptr));

  const auto a2_planner = std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Plan::Configuration(
          graph, a2_config.traits),
        rmf_traffic::agv::Plan::Options(nullptr));

  const auto job_cooldown = 0s;

//  GIVEN("1 Participant")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        *database);

//    WHEN("Schedule:[], Negotiation:[p0(A->Z)]")
    {
      const auto time = std::chrono::steady_clock::now();
      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

//      THEN("Valid Proposal is found")
      {
        std::cout << " ---------------- " << __LINE__ << " --------------------" << std::endl;
        const auto room = std::make_shared<NegotiationRoom>(database, intentions);
        auto future_proposal = room->print().solve();
        const auto status = future_proposal.wait_for(20min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary = get_participant_itinerary(proposal, p0.id());
        REQUIRE(p0_itinerary);
        REQUIRE(p0_itinerary->back()->trajectory().back().position()
                .segment(0, 2) == vertices["Z"].first);

        std::cout << "... Waiting for jobs to finish..." << std::endl;
        // Wait for jobs to finish
        std::this_thread::sleep_for(job_cooldown);
      }
    }

//    WHEN("Schedule:[], Negotiation:[p0(X->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["X"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

//      THEN("Valid Proposal is found")
      {
        std::cout << " ---------------- " << __LINE__ << " --------------------" << std::endl;
        const auto room = std::make_shared<NegotiationRoom>(database, intentions);
        auto future_proposal = room->print().solve();
        const auto status = future_proposal.wait_for(20min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary = get_participant_itinerary(proposal, p0.id());
        REQUIRE(p0_itinerary);
        REQUIRE(p0_itinerary->back()->trajectory().back().position()
                .segment(0, 2) == vertices["C"].first);

        std::cout << "... Waiting for jobs to finish..." << std::endl;
        // Wait for jobs to finish
        std::this_thread::sleep_for(job_cooldown);
      }
    }
  }

//  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        *database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        *database);

//    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(E->V)]")
    {
      const auto time = std::chrono::steady_clock::now();

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["V"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

//      THEN("Valid Proposal is found")
      {
        std::cout << " ---------------- " << __LINE__ << " --------------------" << std::endl;
        const auto room = std::make_shared<NegotiationRoom>(database, intentions);
        auto future_proposal = room->print().solve();
        const auto status = future_proposal.wait_for(20min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary = get_participant_itinerary(proposal, p0.id());
        REQUIRE(p0_itinerary);

        auto p1_itinerary = get_participant_itinerary(proposal, p1.id());
        REQUIRE(p1_itinerary);

        REQUIRE(p0_itinerary->back()->trajectory().back().position()
                .segment(0, 2) == vertices["Z"].first);
        REQUIRE(p1_itinerary->back()->trajectory().back().position()
                .segment(0, 2) == vertices["V"].first);

        std::cout << "... Waiting for jobs to finish..." << std::endl;
        // Wait for jobs to finish
        std::this_thread::sleep_for(job_cooldown);
      }
    }

//    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(V->E)]")
    {
      const auto time = std::chrono::steady_clock::now();

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["V"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["E"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

//      THEN("Valid Proposal is found")
      {
        std::cout << " ---------------- " << __LINE__ << " --------------------" << std::endl;
        const auto room = std::make_shared<NegotiationRoom>(database, intentions);
        auto future_proposal = room->print().solve();
        const auto status = future_proposal.wait_for(20min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["E"].first);

        std::cout << "... Waiting for jobs to finish..." << std::endl;
        // Wait for jobs to finish
        std::this_thread::sleep_for(job_cooldown);
      }
    }

//    WHEN("Schedule:[], Negotiation:[p0(A->X), p1(V->Z)]")
    {
      const auto time = std::chrono::steady_clock::now();

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["X"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["V"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

//      THEN("Valid Proposal is found")
      {
        std::cout << " ---------------- " << __LINE__ << " --------------------" << std::endl;
        const auto room = std::make_shared<NegotiationRoom>(database, intentions);
        auto future_proposal = room->print().solve();
        const auto status = future_proposal.wait_for(20min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["X"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);

        std::cout << "... Waiting for jobs to finish..." << std::endl;
        // Wait for jobs to finish
        std::this_thread::sleep_for(job_cooldown);
      }
    }

//    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(X->C)")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["X"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

//      THEN("Valid Proposal is found")
      {
        std::cout << " ---------------- " << __LINE__ << " --------------------" << std::endl;
        const auto room = std::make_shared<NegotiationRoom>(database, intentions);
        auto future_proposal = room->print().solve();
        const auto status = future_proposal.wait_for(20min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);

        std::cout << "... Waiting for jobs to finish..." << std::endl;
        // Wait for jobs to finish
        std::this_thread::sleep_for(job_cooldown);
      }
    }
  }

//  GIVEN("3 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        *database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        *database);
    auto p2 = rmf_traffic::schedule::make_participant(a2_config.description,
        *database);

//    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(E->V), p2(C->X)]")
    {
      const auto time = std::chrono::steady_clock::now();

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["V"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p2.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["C"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["X"], // Goal Vertex
            a2_planner // Planner Configuration ( Preset )
          }
        });

      // We don't run this test in debug mode because it takes a long time
//      THEN("Valid Proposal is found")
      {
        std::cout << " ---------------- " << __LINE__ << " --------------------" << std::endl;
        const auto room = std::make_shared<NegotiationRoom>(database, intentions);
        auto future_proposal = room->print().solve();
        const auto status = future_proposal.wait_for(20min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        auto p2_itinerary =
          get_participant_itinerary(proposal, p2.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["V"].first);
        REQUIRE(p2_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["X"].first);

        std::cout << "... Waiting for jobs to finish..." << std::endl;
        // Wait for jobs to finish
        std::this_thread::sleep_for(job_cooldown);
      }
    }
//    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(E->V), p2(X->C)]")
//    {
//      const auto time = std::chrono::steady_clock::now();

//      NegotiationRoom::Intentions intentions;
//      intentions.insert({
//          p0.id(),
//          NegotiationRoom::Intention{
//            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
//            vertex_id_to_idx["Z"], // Goal Vertex
//            a0_planner // Planner Configuration ( Preset )
//          }
//        });

//      intentions.insert({
//          p1.id(),
//          NegotiationRoom::Intention{
//            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
//            vertex_id_to_idx["V"], // Goal Vertex
//            a1_planner // Planner Configuration ( Preset )
//          }
//        });

//      intentions.insert({
//          p2.id(),
//          NegotiationRoom::Intention{
//            {time, vertex_id_to_idx["X"], 0.0},  // Time, Start Vertex, Initial Orientation
//            vertex_id_to_idx["C"], // Goal Vertex
//            a2_planner // Planner Configuration ( Preset )
//          }
//        });

      // We don't run this test in debug mode because it takes a long time
//      THEN("Valid Proposal is found")
//      {
//        std::cout << " ---------------- " << __LINE__ << " --------------------" << std::endl;
//        const auto room = std::make_shared<NegotiationRoom>(database, intentions);
//        auto future_success = room->print().solve();
//        const auto status = future_success.wait_for(20min);
//        REQUIRE(status == std::future_status::ready);
//        REQUIRE(future_success.get());

//        const auto proposed_table = room->negotiation->evaluate(
//              rmf_traffic::schedule::QuickestFinishEvaluator());
//        REQUIRE(proposed_table);
//        const auto proposal = proposed_table->proposal();

//        auto p0_itinerary =
//          get_participant_itinerary(proposal, p0.id()).value();
//        auto p1_itinerary =
//          get_participant_itinerary(proposal, p1.id()).value();
//        auto p2_itinerary =
//          get_participant_itinerary(proposal, p2.id()).value();
//        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
//          2) ==
//          vertices["Z"].first);
//        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
//          2) ==
//          vertices["V"].first);
//        REQUIRE(p2_itinerary.back()->trajectory().back().position().segment(0,
//          2) ==
//          vertices["C"].first);
//      }

//      std::cout << "... Waiting for jobs to finish..." << std::endl;
//      // Wait for jobs to finish
//      std::this_thread::sleep_for(job_cooldown);
//    }
  }

  std::this_thread::sleep_for(10s);
}
