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

#include "../thread_cooldown.hpp"

#include <thread>

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
    std::shared_ptr<std::mutex> mutex,
    rxcpp::schedulers::worker worker,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<const bool> interrupt)
  : _table(std::move(table)),
    _table_version(_table->version()),
    _negotiators(std::move(negotiators)),
    _cv(std::move(cv)),
    _mutex(std::move(mutex)),
    _worker(std::move(worker)),
    _node(std::move(node)),
    _interrupt(std::move(interrupt))
  {
    if (const auto parent = _table->parent())
      _parent_version = parent->version();
  }

  void submit(
    std::vector<rmf_traffic::Route> itinerary,
    ApprovalCallback = nullptr) const final
  {
    const auto lock = _lock();
    if (*_interrupt)
      return;

    _table->submit(itinerary, _table_version+1);

    for (const auto& c : _table->children())
    {
      auto n_it = _negotiators->at(c->participant());
      auto responder = std::make_shared<Responder>(
        c, _negotiators, _cv, _mutex, _worker, _node, _interrupt);

      _worker.schedule(
        [
          negotiator = std::move(n_it),
          viewer = c->viewer(),
          responder = std::move(responder)
        ](const auto&)
        {
          if (*responder->_interrupt)
            return;

          negotiator->respond(viewer, responder);
        });
    }

    _cv->notify_all();
  }

  void reject(const Alternatives& alternatives) const final
  {
    const auto lock = _lock();
    if (*_interrupt)
      return;

    REQUIRE(_parent_version);
    const auto parent = _table->parent();
    REQUIRE(parent);
    if (parent->defunct())
      return;

    parent->reject(*_parent_version, _table->participant(), alternatives);

    auto n_it = _negotiators->at(parent->participant());
    auto responder = std::make_shared<Responder>(
      parent, _negotiators, _cv, _mutex, _worker, _node, _interrupt);

    _worker.schedule(
      [
        negotiator = std::move(n_it),
        responder = std::move(responder),
        viewer = parent->viewer()
      ](const auto&)
      {
        if (*responder->_interrupt)
          return;

        negotiator->respond(viewer, responder);
      });

    _cv->notify_all();
  }

  void forfeit(const std::vector<ParticipantId>&) const final
  {
    const auto lock = _lock();
    if (*_interrupt)
      return;

    _table->forfeit(_table_version);
    _cv->notify_all();
  }

private:

  std::unique_lock<std::mutex> _lock() const
  {
    std::unique_lock<std::mutex> lock(*_mutex, std::defer_lock);
    while (!lock.try_lock())
    {
      // Intentionally busy wait
    }

    return lock;
  }

  rmf_traffic::schedule::Negotiation::TablePtr _table;
  std::size_t _table_version;
  rmf_utils::optional<std::size_t> _parent_version;
  NegotiatorMapPtr _negotiators;
  std::shared_ptr<std::condition_variable> _cv;
  std::shared_ptr<std::mutex> _mutex;
  rxcpp::schedulers::worker _worker;

  // We store a reference to the node so that the MockAdapterFixture remains
  // does not allow the executable to exit while this object is still alive.
  rclcpp::Node::SharedPtr _node;

  // When this contains a true value, we no longer need to respond anymore.
  std::shared_ptr<const bool> _interrupt;

};

SCENARIO_METHOD(MockAdapterFixture, "go to place negotiation", "[phases]")
{
  rmf_fleet_adapter_test::thread_cooldown = true;
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
  const auto interrupt = std::make_shared<bool>(false);

  auto negotiation = rmf_traffic::schedule::Negotiation::make(
    context_0->schedule()->snapshot(),
    {context_0->itinerary().id(), context_1->itinerary().id()});

  const auto table_0 = negotiation->table(context_0->itinerary().id(), {});
  context_0->respond(
    table_0->viewer(),
    std::make_shared<Responder>(
      table_0, negotiators, cv, mutex,
      context_0->worker(), data->node, interrupt));

  const auto table_1 = negotiation->table(context_1->itinerary().id(), {});
  context_1->respond(
    table_1->viewer(),
    std::make_shared<Responder>(
      table_1, negotiators, cv, mutex,
      context_0->worker(), data->node, interrupt));

  using namespace std::chrono_literals;
  const auto giveup = std::chrono::steady_clock::now() + 30s;

  {
    // We need to scope this block or else we can get permanent lock-ups during
    // destructions with the other loose threads that are still running.
    std::unique_lock<std::mutex> lock(*mutex);
    cv->wait(lock, [&]()
      {
        return negotiation->ready() || negotiation->complete()
        || giveup < std::chrono::steady_clock::now();
      });

    *interrupt = true;
  }

  active_0->cancel();
  active_1->cancel();
  CHECK(negotiation->ready());
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter
