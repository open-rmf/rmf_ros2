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
#include <rmf_traffic/DetectConflict.hpp>

#include <rmf_utils/catch.hpp>

#include "../thread_cooldown.hpp"

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

//==============================================================================
inline rmf_utils::optional<rmf_traffic::schedule::Itinerary>
get_participant_itinerary(
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

//==============================================================================
bool no_conflicts(
  const rmf_traffic::Profile& p0,
  const rmf_traffic::schedule::Itinerary& i0,
  const rmf_traffic::Profile& p1,
  const rmf_traffic::schedule::Itinerary& i1)
{
  for (const auto& r0 : i0)
  {
    for (const auto& r1 : i1)
    {
      if (r0.map() != r1.map())
        continue;

      if (rmf_traffic::DetectConflict::between(
          p0, r0.trajectory(), nullptr,
          p1, r1.trajectory(), nullptr))
        return false;
    }
  }

  return true;
}

//==============================================================================
bool no_conflicts(
  const rmf_traffic::schedule::Participant& p0,
  const rmf_traffic::schedule::Itinerary& i0,
  const rmf_traffic::schedule::Participant& p1,
  const rmf_traffic::schedule::Itinerary& i1)
{
  return no_conflicts(
    p0.description().profile(), i0,
    p1.description().profile(), i1);
}

//==============================================================================
inline rmf_traffic::Time print_start(const rmf_traffic::Route& route)
{
  assert(route.trajectory().size() > 0);
  std::cout << "(start) --> ";
  std::cout << "(" << 0.0 << "; "
            << route.trajectory().front().position().transpose()
            << ") --> ";

  return *route.trajectory().start_time();
}

//==============================================================================
inline void print_route(
  const rmf_traffic::Route& route,
  const rmf_traffic::Time start_time)
{
  assert(route.trajectory().size() > 0);
  for (auto it = ++route.trajectory().begin(); it
    != route.trajectory().end(); ++it)
  {
    const auto& wp = *it;
    if (wp.velocity().norm() > 1e-3)
      continue;

    const auto rel_time = wp.time() - start_time;
    std::cout << "(" << rmf_traffic::time::to_seconds(rel_time) << "; "
              << wp.position().transpose() << ") --> ";
  }
}

//==============================================================================
inline void print_itinerary(const std::vector<rmf_traffic::Route>& itinerary)
{
  if (itinerary.empty())
  {
    std::cout << "No plan needed!" << std::endl;
  }
  else
  {
    auto start_time = print_start(itinerary.front());
    for (const auto& r : itinerary)
      print_route(r, start_time);

    std::cout << "(end)" << std::endl;
  }
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

  using ParticipantId = rmf_traffic::schedule::ParticipantId;
  using Intentions = std::unordered_map<ParticipantId, Intention>;

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

  TestPathNegotiator& n(
    std::shared_ptr<rmf_traffic::schedule::Negotiation> negotiation)
  {
    _n = std::move(negotiation);
    return *this;
  }

  TestPathNegotiator& w(const rxcpp::schedulers::worker& worker)
  {
    _worker = worker;
    return *this;
  }

  void respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder) final
  {
    if (_print)
    {
      std::cout << "    Responding to " << to_string(table_viewer)
                << " (" << _n.lock().get() << ")" << std::endl;
    }

    rmf_fleet_adapter::services::ProgressEvaluator evaluator;
    if (table_viewer->parent_id())
    {
      const auto& s = table_viewer->sequence();
      assert(s.size() >= 2);
      evaluator.compliant_leeway_base *= s[s.size()-2].version + 1;
    }

    auto negotiate = rmf_fleet_adapter::services::Negotiate::path(
      0, _planner, _starts, _goal, {}, table_viewer, responder, nullptr,
      evaluator);

    auto sub = rmf_rxcpp::make_job<
      rmf_fleet_adapter::services::Negotiate::Result>(negotiate)
      .observe_on(rxcpp::identity_same_worker(_worker))
      .subscribe([w = weak_from_this()](const auto& result)
        {
          result.respond();
          if (const auto self = w.lock())
            self->_services.erase(result.service);
        });

    _subscriptions.emplace_back(std::move(sub));
    _services.insert(std::move(negotiate));
  }

private:
  std::shared_ptr<rmf_traffic::agv::Planner> _planner;
  rmf_traffic::agv::Plan::StartSet _starts;
  rmf_traffic::agv::Plan::Goal _goal;
  std::vector<rmf_rxcpp::subscription_guard> _subscriptions;
  std::unordered_set<std::shared_ptr<rmf_fleet_adapter::services::Negotiate>>
  _services;
  bool _print = false;
  std::weak_ptr<rmf_traffic::schedule::Negotiation> _n;
  rxcpp::schedulers::worker _worker;
};

//==============================================================================
std::unordered_map<
  rmf_traffic::schedule::ParticipantId,
  std::shared_ptr<TestPathNegotiator>
>
make_negotiators(const TestPathNegotiator::Intentions& intentions)
{
  std::unordered_map<
    rmf_traffic::schedule::ParticipantId,
    std::shared_ptr<TestPathNegotiator>
  > negotiators;

  for (const auto& entry : intentions)
  {
    const auto participant = entry.first;
    const auto& intention = entry.second;
    negotiators.insert(
      std::make_pair(
        participant,
        std::make_shared<TestPathNegotiator>(
          intention.planner, intention.start, intention.goal)));
  }

  return negotiators;
}

//==============================================================================
class TestEmergencyNegotiator
  : public rmf_traffic::schedule::Negotiator,
  public std::enable_shared_from_this<TestEmergencyNegotiator>
{
public:

  struct Intention
  {
    std::vector<rmf_traffic::agv::Planner::Start> start;
    std::shared_ptr<rmf_traffic::agv::Planner> planner;

    Intention(
      std::vector<rmf_traffic::agv::Planner::Start> starts_,
      std::shared_ptr<rmf_traffic::agv::Planner> planner_)
    : start(std::move(starts_)),
      planner(std::move(planner_))
    {
      // Do nothing
    }

    Intention(
      rmf_traffic::agv::Planner::Start start_,
      std::shared_ptr<rmf_traffic::agv::Planner> planner_)
    : start({std::move(start_)}),
      planner(std::move(planner_))
    {
      // Do nothing
    }
  };

  using ParticipantId = rmf_traffic::schedule::ParticipantId;
  using Intentions = std::unordered_map<ParticipantId, Intention>;

  TestEmergencyNegotiator(
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    rmf_traffic::agv::Plan::StartSet starts)
  : _planner(std::move(planner)),
    _starts(std::move(starts))
  {
    // Do nothing
  }

  TestEmergencyNegotiator& print(bool on)
  {
    _print = on;
    return *this;
  }

  TestEmergencyNegotiator& n(
    std::shared_ptr<rmf_traffic::schedule::Negotiation> negotiation)
  {
    _n = std::move(negotiation);
    return *this;
  }

  TestEmergencyNegotiator& w(const rxcpp::schedulers::worker& worker)
  {
    _worker = worker;
    return *this;
  }

  void respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder) final
  {
    if (_print)
    {
      std::cout << "    Responding to " << to_string(table_viewer)
                << " (" << _n.lock().get() << ")" << std::endl;
    }

    rmf_fleet_adapter::services::ProgressEvaluator evaluator;
    if (table_viewer->parent_id())
    {
      const auto& s = table_viewer->sequence();
      assert(s.size() >= 2);
      evaluator.compliant_leeway_base *= s[s.size()-2].version + 1;
    }

    auto negotiate = rmf_fleet_adapter::services::Negotiate::emergency_pullover(
      0, _planner, _starts, table_viewer, responder, nullptr, evaluator);

    auto sub = rmf_rxcpp::make_job<
      rmf_fleet_adapter::services::Negotiate::Result>(std::move(negotiate))
      .observe_on(rxcpp::identity_same_worker(_worker))
      .subscribe([w = weak_from_this()](const auto& result)
        {
          result.respond();
          if (const auto self = w.lock())
            self->_services.erase(result.service);
        });

    _subscriptions.emplace_back(std::move(sub));
    _services.insert(std::move(negotiate));
  }

private:
  std::shared_ptr<rmf_traffic::agv::Planner> _planner;
  rmf_traffic::agv::Plan::StartSet _starts;
  std::vector<rmf_rxcpp::subscription_guard> _subscriptions;
  std::unordered_set<std::shared_ptr<rmf_fleet_adapter::services::Negotiate>>
  _services;
  bool _print = false;
  std::weak_ptr<rmf_traffic::schedule::Negotiation> _n;
  rxcpp::schedulers::worker _worker;
};

//==============================================================================
std::unordered_map<
  rmf_traffic::schedule::ParticipantId,
  std::shared_ptr<TestEmergencyNegotiator>
>
make_negotiators(const TestEmergencyNegotiator::Intentions& intentions)
{
  std::unordered_map<
    rmf_traffic::schedule::ParticipantId,
    std::shared_ptr<TestEmergencyNegotiator>
  > negotiators;

  for (const auto& entry : intentions)
  {
    const auto participant = entry.first;
    const auto& intention = entry.second;
    negotiators.insert(
      {
        participant,
        std::make_shared<TestEmergencyNegotiator>(
          intention.planner, intention.start)
      });
  }

  return negotiators;
}

//==============================================================================
template<typename NegotiatorT>
class NegotiationRoom
  : public std::enable_shared_from_this<NegotiationRoom<NegotiatorT>>
{
public:

  using ParticipantId = rmf_traffic::schedule::ParticipantId;
  using Negotiator = NegotiatorT;
  using Negotiation = rmf_traffic::schedule::Negotiation;

  class Responder : public rmf_traffic::schedule::Negotiator::Responder
  {
  public:
    Responder(
      std::shared_ptr<NegotiationRoom<Negotiator>> room,
      rmf_traffic::schedule::Negotiation::TablePtr table)
    : _room(std::move(room)),
      _table(std::move(table))
    {
      // Do nothing
    }

    template<typename... Args>
    static std::shared_ptr<Responder> make(Args&& ... args)
    {
      return std::make_shared<Responder>(std::forward<Args>(args)...);
    }

    void submit(
      rmf_traffic::PlanId plan,
      std::vector<rmf_traffic::Route> itinerary,
      ApprovalCallback approval_callback = nullptr) const final
    {
      const auto room = _room.lock();
      if (!room)
        return;

      if (!_table->ongoing())
      {
        if (room->_print)
        {
          std::cout << "Deprecated negotiation (" << room->negotiation.get()
                    << "): " << to_string(_table);
        }

        return;
      }

      if (_table->defunct())
      {
        if (room->_print)
        {
          std::cout << "Defunct " << to_string(_table) << " ("
                    << room->negotiation.get() << "): " << to_string(_table)
                    << std::endl;
        }

        return;
      }

      rmf_traffic::schedule::SimpleResponder(_table)
      .submit(plan, std::move(itinerary), std::move(approval_callback));

      if (room->_print)
      {
        std::cout << "Submission given for " + to_string(_table)
                  << " (" << room->negotiation.get() << "):\n";
        print_itinerary(*_table->submission());
      }

      if (room->check_finished())
        return;

      for (const auto& n : room->negotiators)
      {
        const auto participant = n.first;
        const auto respond_to = _table->respond(participant);
        if (respond_to)
        {
          if (skip(respond_to))
          {
            if (room->_print)
            {
              std::cout << "    Skipping a response request from "
                        << to_string(respond_to) << std::endl;
            }

            respond_to->forfeit(respond_to->version());
            continue;
          }

          n.second->print(room->_print).n(room->negotiation).w(room->worker)
          .respond(respond_to->viewer(), make(room, respond_to));
        }
      }
    }

    void reject(const Alternatives& alternatives) const final
    {
      const auto room = _room.lock();
      if (!room)
        return;

      if (_table->defunct())
        return;

      rmf_traffic::schedule::SimpleResponder(_table).reject(alternatives);

      if (room->check_finished())
        return;

      const auto parent = _table->parent();
      if (parent)
      {
        if (room->_print)
        {
          std::cout << "[ "<< std::to_string(_table->participant())
                    << " ] rejected " << to_string(parent) << " ("
                    << room->negotiation.get() << ") with ["
                    << alternatives.size() << "] alternatives" << std::endl;
        }

        if (skip(parent))
        {
          std::cout << "Forfeit given for " << to_string(parent)
                    << " after too many rejections";
          parent->forfeit(parent->version());
          room->check_finished();
          return;
        }

        room->negotiators.at(parent->participant())
        ->print(room->_print).n(room->negotiation).w(room->worker)
        .respond(parent->viewer(), make(room, parent));
      }
    }

    void forfeit(const std::vector<ParticipantId>& blockers) const final
    {
      const auto room = _room.lock();
      if (!room)
        return;

      if (_table->defunct())
        return;

      rmf_traffic::schedule::SimpleResponder(_table).forfeit(blockers);

      if (room->_print)
      {
        std::cout << "Forfeit given for " << to_string(_table)
                  << " with the following blockers:";
        for (const auto p : blockers)
          std::cout << " " << p << std::endl;
      }

      room->check_finished();
    }

  private:
    std::weak_ptr<NegotiationRoom> _room;
    rmf_traffic::schedule::Negotiation::TablePtr _table;
  };

  using Intention = typename Negotiator::Intention;
  using Intentions = typename Negotiator::Intentions;

  NegotiationRoom(
    std::shared_ptr<const rmf_traffic::schedule::Viewer> viewer,
    Intentions intentions,
    const bool print = false)
  : negotiators(make_negotiators(intentions)),
    negotiation(Negotiation::make_shared(
        std::move(viewer), get_participants(intentions))),
    worker(rxcpp::schedulers::make_event_loop().create_worker()),
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
      negotiator->print(_print).n(negotiation).w(worker).respond(
        table->viewer(),
        Responder::make(this->shared_from_this(), table));
    }

    return _solution.get_future();
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
    if (!negotiation)
      return true;

    if (!negotiation->ready() && !negotiation->complete())
      return false;

    if (negotiation->ready())
    {
      const auto winner = negotiation->evaluate(
        rmf_traffic::schedule::QuickestFinishEvaluator());

      if (_print)
      {
        std::cout << "Successfully finished negotiation ("
                  << negotiation.get() << ") with " << to_string(winner)
                  << std::endl;
      }

      if (!_promise_fulfilled)
      {
        _promise_fulfilled = true;
        _solution.set_value(winner->proposal());
      }
    }
    else
    {
      std::cout << "Failed finish for negotiation (" << negotiation.get()
                << ")" << std::endl;

      if (!_promise_fulfilled)
      {
        _promise_fulfilled = true;
        _solution.set_value(rmf_utils::nullopt);
      }
    }

    negotiation.reset();
    return true;
  }

  std::unordered_map<ParticipantId, std::shared_ptr<Negotiator>> negotiators;
  std::shared_ptr<rmf_traffic::schedule::Negotiation> negotiation;
  rxcpp::schedulers::worker worker;

  std::promise<OptProposal> _solution;
  bool _promise_fulfilled = false;


  NegotiationRoom& print()
  {
    _print = true;
    return *this;
  }

  bool _print = false;
};

using TestPathNegotiationRoom = NegotiationRoom<TestPathNegotiator>;
using TestEmergencyNegotiationRoom = NegotiationRoom<TestEmergencyNegotiator>;

} // anonymous namespace

// TODO(MXG): The GitHub Actions CI seems to struggle with this test, possibly
// because it's running single-threaded in debug mode. We'll turn this test off
// for debug mode for now, but we should try to get it active again by making
// the negotiation process perform better for this edge case or by having the CI
// compile in Release mode.
//
// Simultaneously or alternatively, we could run tests on our own servers,
// perhaps nightly, that run these tests with more resource allocaation.

//==============================================================================
SCENARIO("Test Plan Negotiation Between Two Participants", "[.high_cpu]")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  using namespace std::chrono_literals;

  auto database = std::make_shared<rmf_traffic::schedule::Database>();

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  auto p1 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 1",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  auto p2 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 2",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  auto p3 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 3",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      profile
    },
    database);

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                  10
   *                   |
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6------7
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(1, 5);
  add_bidir_lane(2, 6);
  add_bidir_lane(3, 4);
  add_bidir_lane(4, 5);
  add_bidir_lane(5, 6);
  add_bidir_lane(6, 7);
  add_bidir_lane(5, 8);
  add_bidir_lane(6, 9);
  add_bidir_lane(8, 9);
  add_bidir_lane(8, 10);

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  const rmf_traffic::Duration wait_time = 1s;

  rmf_traffic::agv::Planner::Configuration configuration{graph, traits};
  const auto planner = std::make_shared<rmf_traffic::agv::Planner>(
    configuration,
    rmf_traffic::agv::Planner::Options{nullptr, wait_time}
  );

  auto start_time = std::chrono::steady_clock::now();

  const auto start_1 = rmf_traffic::agv::Plan::Start(start_time, 3, 0.0);
  const auto goal_1 = rmf_traffic::agv::Plan::Goal(7);

  const auto plan_1 = planner->plan(start_1, goal_1);
  REQUIRE(plan_1);
  p1.set(p1.assign_plan_id(), plan_1->get_itinerary());

  const auto start_2 =
    rmf_traffic::agv::Plan::Start(start_time, 7, 0.0*M_PI/180.0);
  const auto goal_2 = rmf_traffic::agv::Plan::Goal(3);

  const auto plan_2 = planner->plan(start_2, goal_2);
  REQUIRE(plan_2);
  p2.set(p2.assign_plan_id(), plan_2->get_itinerary());

  CHECK_FALSE(
    no_conflicts(p1, p1.itinerary(), p2, p2.itinerary()));

  WHEN("Participants Crossing Paths")
  {
    GIVEN("No third participant")
    {
      // Intentionally do nothing
    }

    GIVEN("A third participant")
    {
      const auto plan_3 = planner->plan(
        rmf_traffic::agv::Plan::Start(start_time, 0, 90.0*M_PI/180.0),
        rmf_traffic::agv::Plan::Goal(10));
      REQUIRE(plan_3);

      p3.set(p3.assign_plan_id(), plan_3->get_itinerary());
    }

    TestPathNegotiator::Intentions intentions;
    intentions.insert({p1.id(), { start_1, goal_1, planner }});
    intentions.insert({p2.id(), { start_2, goal_2, planner }});

    const auto room = std::make_shared<TestPathNegotiationRoom>(
      database->snapshot(), intentions);
    auto future_proposal = room->solve();
    const auto status = future_proposal.wait_for(2min);
    REQUIRE(status == std::future_status::ready);

    const auto proposal_opt = future_proposal.get();
    REQUIRE(proposal_opt);
    const auto& proposal = *proposal_opt;

    auto p1_itinerary = get_participant_itinerary(proposal, p1.id()).value();
    auto p2_itinerary = get_participant_itinerary(proposal, p2.id()).value();
    CHECK(p1_itinerary.back().trajectory().back().position().segment(0, 2)
      == graph.get_waypoint(goal_1.waypoint()).get_location());
    CHECK(p2_itinerary.back().trajectory().back().position().segment(0, 2)
      == graph.get_waypoint(goal_2.waypoint()).get_location());

    CHECK(no_conflicts(p1, p1_itinerary, p2, p2_itinerary));
  }
}


//==============================================================================
SCENARIO("Multi-participant negotiation", "[.high_cpu]")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  using namespace std::chrono_literals;

  auto database = std::make_shared<rmf_traffic::schedule::Database>();

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  auto p1 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 1",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  auto p2 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 2",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  auto p3 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 3",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, { 0.0, -5.0}); // 0
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 1
  graph.add_waypoint(test_map_name, { 0.0, 0.0}); // 2
  graph.add_waypoint(test_map_name, { 5.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, { 0.0, 5.0}); // 4

  /*
   *         4
   *         |
   *         |
   *   1-----2-----3
   *         |
   *         |
   *         0
   */

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 2);
  add_bidir_lane(1, 2);
  add_bidir_lane(3, 2);
  add_bidir_lane(4, 2);

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  const auto time = std::chrono::steady_clock::now();

  rmf_traffic::agv::Planner::Configuration configuration{graph, traits};
  const auto planner = std::make_shared<rmf_traffic::agv::Planner>(
    configuration, rmf_traffic::agv::Plan::Options(nullptr));

  const std::size_t goal_0 = 3;
  const std::size_t goal_1 = 4;
  const std::size_t goal_2 = 1;

  TestPathNegotiator::Intentions intentions;
  intentions.insert({0, { {time, 1, 0.0}, goal_0, planner}});
  intentions.insert({1, { {time, 0, M_PI/2.0}, goal_1, planner}});
  intentions.insert({2, { {time, 3, 0.0}, goal_2, planner}});

  const auto room = std::make_shared<TestPathNegotiationRoom>(
    database->snapshot(), intentions);
  auto future_proposal = room->solve();
  const auto status = future_proposal.wait_for(2min);
  REQUIRE(status == std::future_status::ready);

  const auto proposal_opt = future_proposal.get();
  REQUIRE(proposal_opt);
  const auto& proposal = *proposal_opt;

  auto p0_itinerary = get_participant_itinerary(proposal, 0).value();
  auto p1_itinerary = get_participant_itinerary(proposal, 1).value();
  auto p2_itinerary = get_participant_itinerary(proposal, 2).value();
  CHECK(p0_itinerary.back().trajectory().back().position()
    .segment(0, 2) == graph.get_waypoint(goal_0).get_location());
  CHECK(p1_itinerary.back().trajectory().back().position()
    .segment(0, 2) == graph.get_waypoint(goal_1).get_location());
  CHECK(p2_itinerary.back().trajectory().back().position()
    .segment(0, 2) == graph.get_waypoint(goal_2).get_location());

  CHECK(no_conflicts(profile, p0_itinerary, profile, p1_itinerary));
  CHECK(no_conflicts(profile, p0_itinerary, profile, p2_itinerary));
  CHECK(no_conflicts(profile, p1_itinerary, profile, p2_itinerary));
}

SCENARIO("A single lane with an alcove holding space", "[.high_cpu]")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_single_lane_with_alcove";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *    single_lane_with_alcove
   *
   *             E(H)
   *             ^
   *             | 3
   *             |
   *        3    v    3           3
   *   A <-----> B <------> C <------> D
   *
   */

  vertices.insert({"A", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"E", {{0.0, 3.0}, IsHoldingSpot(true)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});
  edges.insert({"BE", {{"B", "E"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  const auto a0_planner = std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Plan::Configuration(
      graph, a0_config.traits),
    rmf_traffic::agv::Plan::Options(nullptr));

  const auto a1_planner = std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Plan::Configuration(
      graph, a1_config.traits),
    rmf_traffic::agv::Plan::Options(nullptr));

  const auto now = std::chrono::steady_clock::now();

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(
      a0_config.description, database);
    auto p1 = rmf_traffic::schedule::make_participant(
      a1_config.description, database);

    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(D->A)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["D"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["D"], 0.0},
            vertex_id_to_idx["A"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["A"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(B->D), p1(D->A)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["B"], 0.0},
            vertex_id_to_idx["D"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["D"], 0.0},
            vertex_id_to_idx["A"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        REQUIRE(room);

        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["A"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(B->A)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["D"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["B"], 0.0},
            vertex_id_to_idx["A"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        REQUIRE(room);

        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["A"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(C->A)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["D"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["C"], 0.0},
            vertex_id_to_idx["A"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        REQUIRE(room);

        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();

        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["A"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }
  }
}

SCENARIO("A single lane with a alternate one way path", "[.high_cpu]")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name =
    "test_single_lane_with_alternate one-way path";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *       test_single_lane_with_alternative_path
   *
   *                E <------+ F
   *                +          ^
   *                | 3        | 3
   *                |          |
   *           3    v    3     +    3
   *      A <-----> B <------> C <------> D
   */

  vertices.insert({"A", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"E", {{0.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"F", {{3.0, 3.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});
  edges.insert({"CF", {{"C", "F"}, IsBidirectional(false)}});
  edges.insert({"FE", {{"F", "E"}, IsBidirectional(false)}});
  edges.insert({"EB", {{"E", "B"}, IsBidirectional(false)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  const auto now = std::chrono::steady_clock::now();
  rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
    a0_config.traits};
  rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
    a1_config.traits};

  const auto a0_planner =
    std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Planner::Configuration(graph, a0_config.traits),
    rmf_traffic::agv::Planner::Options(nullptr));

  const auto a1_planner =
    std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Planner::Configuration(graph, a1_config.traits),
    rmf_traffic::agv::Planner::Options(nullptr));

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(
      a0_config.description, database);
    auto p1 = rmf_traffic::schedule::make_participant(
      a1_config.description, database);
    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(C->A)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["C"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["A"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(D->A)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();

        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["A"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }
  }
}

SCENARIO("A single lane with a alternate two way path", "[.high_cpu]")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name =
    "test_single_lane_with_alternate_two_way_path";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *       test_single_lane_with_alternative_two_way_path
   *
   *                E <------> F
   *                ^          ^
   *                | 3        | 3
   *                |          |
   *           3    v    3     v    3
   *      A <-----> B <------> C <------> D
   */

  vertices.insert({"A", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"E", {{0.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"F", {{3.0, 3.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});
  edges.insert({"CF", {{"C", "F"}, IsBidirectional(true)}});
  edges.insert({"FE", {{"F", "E"}, IsBidirectional(true)}});
  edges.insert({"EB", {{"E", "B"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

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

  const auto now = std::chrono::steady_clock::now();

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(
      a0_config.description, database);
    auto p1 = rmf_traffic::schedule::make_participant(
      a1_config.description, database);
    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(C->A)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["C"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();

        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();

        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["A"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(D->A)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();

        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["A"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }
  }
}

SCENARIO("A single loop with alcoves at each vertex", "[.high_cpu]")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_single_loop";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *    single_loop_with_alcoves
   *
   *      A'               B'
   *      ^ 3      6     3 ^
   *      +-->A <----> B<--+
   *          ^        ^
   *        6 |        | 6
   *          v        v
   *      +-->D<------>C<--+
   *      v 3      6     3 v
   *      D'               C'
   */

  vertices.insert({"A", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{6.0, -6.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{-0.0, -6.0}, IsHoldingSpot(false)}});

  vertices.insert({"A'", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B'", {{9.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C'", {{9.0, -6.0}, IsHoldingSpot(false)}});
  vertices.insert({"D'", {{-3.0, -6.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});
  edges.insert({"AD", {{"A", "D"}, IsBidirectional(true)}});

  edges.insert({"AA'", {{"A", "A'"}, IsBidirectional(true)}});
  edges.insert({"BB'", {{"B", "B'"}, IsBidirectional(true)}});
  edges.insert({"CC'", {{"C", "C'"}, IsBidirectional(true)}});
  edges.insert({"DD'", {{"D", "D'"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

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

  const auto now = std::chrono::steady_clock::now();

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(
      a0_config.description, database);
    auto p1 = rmf_traffic::schedule::make_participant(
      a1_config.description, database);

    WHEN("Schedule:[], Negotiation:[pO(A'->C'), p1(B'->D')]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C'"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["B'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D'"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["C'"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D'"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[pO(A'->C'), p1(D'->B')]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C'"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["D'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B'"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["C'"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["B'"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[pO(B'->D'), p1(D'->B')]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["B'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D'"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["D'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B'"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D'"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["B'"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

  }

  GIVEN("3 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(
      a0_config.description, database);
    auto p1 = rmf_traffic::schedule::make_participant(
      a1_config.description, database);
    auto p2 = rmf_traffic::schedule::make_participant(
      a2_config.description, database);

    WHEN("Schedule:[], Negotiation:[pO(A'->C'), p1(B'->D'), p2(D'->B')]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C'"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["B'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D'"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p2.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["D'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B'"], // Goal Vertex
            a2_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
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
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["C'"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["D'"].first);
        CHECK(p2_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["B'"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
        CHECK(no_conflicts(p0, p0_itinerary, p2, p2_itinerary));
        CHECK(no_conflicts(p1, p1_itinerary, p2, p2_itinerary));
      }
    }
  }
}

//==============================================================================
SCENARIO("fan-in-fan-out bottleneck", "[.high_cpu]")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_fan_in_fan_out";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *        fan_in_fan_out (All lengths are 3)
   *
   *        A(P)    B(H)    C(P)   D(H)    E(P)
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
   *        V(P)    W(H)    X(P)   Y(H)    Z(P)
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

  auto set_parking_spot = [&](const std::string& wp)
    {
      graph.get_waypoint(vertex_id_to_idx[wp]).set_parking_spot(true);
    };
  set_parking_spot("A");
  set_parking_spot("C");
  set_parking_spot("E");
  set_parking_spot("V");
  set_parking_spot("X");
  set_parking_spot("Z");

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

  const auto now = std::chrono::steady_clock::now();

  GIVEN("1 Participant")
  {
    auto p0 = rmf_traffic::schedule::make_participant(
      a0_config.description, database);

    WHEN("Schedule:[], Negotiation:[p0(A->Z)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary = get_participant_itinerary(proposal, p0.id());
        CHECK(p0_itinerary);
        CHECK(p0_itinerary->back().trajectory().back().position()
          .segment(0, 2) == vertices["Z"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(X->C)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["X"], 0.0},
            vertex_id_to_idx["C"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary = get_participant_itinerary(proposal, p0.id());
        CHECK(p0_itinerary);
        CHECK(p0_itinerary->back().trajectory().back().position()
          .segment(0, 2) == vertices["C"].first);
      }
    }
  }

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(
      a0_config.description, database);
    auto p1 = rmf_traffic::schedule::make_participant(
      a1_config.description, database);

    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(E->V)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["E"], 0.0},
            vertex_id_to_idx["V"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();

        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();

        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["Z"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["V"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(V->E)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["V"], 0.0},
            vertex_id_to_idx["E"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["Z"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["E"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->X), p1(V->Z)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["X"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["V"], 0.0},
            vertex_id_to_idx["Z"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["X"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["Z"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(X->C)")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["X"], 0.0},
            vertex_id_to_idx["C"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["Z"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["C"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
      }
    }
  }

  GIVEN("3 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(
      a0_config.description, database);
    auto p1 = rmf_traffic::schedule::make_participant(
      a1_config.description, database);
    auto p2 = rmf_traffic::schedule::make_participant(
      a2_config.description, database);

    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(E->V), p2(C->X)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["E"], 0.0},
            vertex_id_to_idx["V"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p2.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["C"], 0.0},
            vertex_id_to_idx["X"], // Goal Vertex
            a2_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
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
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["Z"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["V"].first);
        CHECK(p2_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["X"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
        CHECK(no_conflicts(p0, p0_itinerary, p2, p2_itinerary));
        CHECK(no_conflicts(p1, p1_itinerary, p2, p2_itinerary));
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(E->V), p2(X->C)]")
    {
      TestPathNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["A"], 0.0},
            vertex_id_to_idx["Z"], // Goal Vertex
            a0_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["E"], 0.0},
            vertex_id_to_idx["V"], // Goal Vertex
            a1_planner // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p2.id(),
          TestPathNegotiator::Intention{
            {now, vertex_id_to_idx["X"], 0.0},
            vertex_id_to_idx["C"], // Goal Vertex
            a2_planner // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        const auto room = std::make_shared<TestPathNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
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
        CHECK(p0_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["Z"].first);
        CHECK(p1_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["V"].first);
        CHECK(p2_itinerary.back().trajectory().back().position()
          .segment(0, 2) == vertices["C"].first);

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
        CHECK(no_conflicts(p0, p0_itinerary, p2, p2_itinerary));
        CHECK(no_conflicts(p1, p1_itinerary, p2, p2_itinerary));
      }
    }

    WHEN("Schedule:[], Emergency Negotiation:[p0(C'), p1(F), p2(X')]")
    {
      TestEmergencyNegotiator::Intentions intentions;
      intentions.insert({
          p0.id(),
          TestEmergencyNegotiator::Intention{
            {now, vertex_id_to_idx["C'"], 0.0},
            a0_planner
          }
        });

      intentions.insert({
          p1.id(),
          TestEmergencyNegotiator::Intention{
            {now, vertex_id_to_idx["F"], 0.0},
            a1_planner
          }
        });

      intentions.insert({
          p2.id(),
          TestEmergencyNegotiator::Intention{
            {now, vertex_id_to_idx["X'"], 0.0},
            a2_planner
          }
        });

      THEN("Valid proposal is found")
      {
        const auto room = std::make_shared<TestEmergencyNegotiationRoom>(
          database->snapshot(), intentions);
        auto future_proposal = room->solve();
        const auto status = future_proposal.wait_for(2min);
        REQUIRE(status == std::future_status::ready);

        const auto proposal_opt = future_proposal.get();
        REQUIRE(proposal_opt);
        const auto& proposal = *proposal_opt;

        auto at_parking_spot = [&](const rmf_traffic::schedule::Itinerary& it)
          -> bool
          {
            for (std::size_t i = 0; i < graph.num_waypoints(); ++i)
            {
              const auto& wp = graph.get_waypoint(i);
              if (!wp.is_parking_spot())
                continue;

              const Eigen::Vector2d p =
                it.back().trajectory().back().position().block<2, 1>(0, 0);

              if ( (p - wp.get_location()).norm() < 1e-3)
                return true;
            }

            return false;
          };

        auto p0_itinerary =
          get_participant_itinerary(proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(proposal, p1.id()).value();
        auto p2_itinerary =
          get_participant_itinerary(proposal, p2.id()).value();

        CHECK(at_parking_spot(p0_itinerary));
        CHECK(at_parking_spot(p1_itinerary));
        CHECK(at_parking_spot(p2_itinerary));

        CHECK(no_conflicts(p0, p0_itinerary, p1, p1_itinerary));
        CHECK(no_conflicts(p0, p0_itinerary, p2, p2_itinerary));
        CHECK(no_conflicts(p1, p1_itinerary, p2, p2_itinerary));
      }
    }
  }
}

