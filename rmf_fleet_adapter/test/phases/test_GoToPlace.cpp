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

#include "MockAdapterFixture.hpp"

#include <phases/GoToPlace.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

using NegotiatorMap =
  std::unordered_map<
    rmf_traffic::schedule::ParticipantId,
    std::shared_ptr<rmf_traffic::schedule::Negotiator>
  >;
using NegotiatorMapPtr = std::shared_ptr<NegotiatorMap>;

class Responder : public rmf_traffic::schedule::Negotiator::Responder
{
public:

  Responder(
      rmf_traffic::schedule::Negotiation::TablePtr table,
      NegotiatorMapPtr negotiators,
      std::shared_ptr<std::condition_variable> cv,
      std::shared_ptr<std::mutex> mutex)
    : _table(std::move(table)),
      _table_version(_table->version()),
      _negotiators(std::move(negotiators)),
      _cv(std::move(cv)),
      _mutex(std::move(mutex))
  {
    if (const auto parent = _table->parent())
      _parent_version = parent->version();
  }

  void submit(
      std::vector<rmf_traffic::Route> itinerary,
      ApprovalCallback = nullptr) const final
  {
    std::lock_guard<std::mutex> lock(*_mutex);
    _table->submit(itinerary, _table_version+1);

    for (const auto& c : _table->children())
    {
      const auto n_it = _negotiators->at(c->participant());
      n_it->respond(
            c->viewer(),
            std::make_shared<Responder>(c, _negotiators, _cv, _mutex));
    }

    _cv->notify_all();
  }

  void reject(const Alternatives& alternatives) const final
  {
    std::lock_guard<std::mutex> lock(*_mutex);
    REQUIRE(_parent_version);
    const auto parent = _table->parent();
    REQUIRE(parent);
    if (parent->defunct())
      return;

    parent->reject(*_parent_version, _table->participant(), alternatives);

    _negotiators->at(parent->participant())->respond(
          parent->viewer(),
          std::make_shared<Responder>(parent, _negotiators, _cv, _mutex));

    _cv->notify_all();
  }

  void forfeit(const std::vector<ParticipantId>&) const final
  {
    std::lock_guard<std::mutex> lock(*_mutex);
    _table->forfeit(_table_version);
    _cv->notify_all();
  }

private:
  rmf_traffic::schedule::Negotiation::TablePtr _table;
  std::size_t _table_version;
  rmf_utils::optional<std::size_t> _parent_version;
  NegotiatorMapPtr _negotiators;
  std::shared_ptr<std::condition_variable> _cv;
  std::shared_ptr<std::mutex> _mutex;

};

SCENARIO_METHOD(MockAdapterFixture, "go to place negotiation", "[phases]")
{
  const auto info_0 = add_robot("test_robot_0");
  info_0.command->pause(true);
  const auto context_0 = info_0.context;

  const auto now = info_0.context->now();
  const auto start_0 = rmf_traffic::agv::Plan::Start(now, 0, M_PI/2.0);
  const auto start_1 = rmf_traffic::agv::Plan::Start(now, 7, 0.0);

  const auto goal_0 = rmf_traffic::agv::Plan::Goal(1);
  const auto goal_1 = rmf_traffic::agv::Plan::Goal(3);

  const auto info_1 = add_robot("test_robot_1");
  info_1.command->pause(true);
  info_1.command->updater->update_position(7, 0.0);
  const auto context_1 = info_1.context;

  const auto pending_0 = phases::GoToPlace::make(context_0, {start_0}, goal_0);
  const auto active_0 = pending_0->begin();

  const auto pending_1 = phases::GoToPlace::make(context_1, {start_1}, goal_1);
  const auto active_1 = pending_1->begin();

  const auto negotiators = std::make_shared<NegotiatorMap>();

  negotiators->insert({context_0->itinerary().id(), context_0});
  negotiators->insert({context_1->itinerary().id(), context_1});

  auto cv = std::make_shared<std::condition_variable>();
  auto mutex = std::make_shared<std::mutex>();
  auto negotiation = rmf_traffic::schedule::Negotiation::make(
        context_0->schedule()->snapshot(),
        {context_0->itinerary().id(), context_1->itinerary().id()});

  const auto table_0 = negotiation->table(context_0->itinerary().id(), {});
  context_0->respond(
        table_0->viewer(),
        std::make_shared<Responder>(table_0, negotiators, cv, mutex));

  const auto table_1 = negotiation->table(context_1->itinerary().id(), {});
  context_1->respond(
        table_1->viewer(),
        std::make_shared<Responder>(table_1, negotiators, cv, mutex));

  using namespace std::chrono_literals;
  const auto giveup = std::chrono::steady_clock::now() + 30s;

  std::unique_lock<std::mutex> lock(*mutex);
  cv->wait(lock, [&]()
  {
    return negotiation->ready() || negotiation->complete()
        || giveup < std::chrono::steady_clock::now();
  });

  CHECK(negotiation->ready());
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter
