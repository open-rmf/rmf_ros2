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

#include <rmf_utils/catch.hpp>

#include <LegacyTask.hpp>

#include "thread_cooldown.hpp"

using namespace std::chrono_literals;

class MockPhase
{
public:

  class Job
  {
  public:

    Job(std::shared_ptr<std::size_t> count,
      std::size_t count_length,
      std::chrono::nanoseconds period)
    : count(count),
      count_length(count_length),
      period(period)
    {
      // Do nothing
    }

    using Result = std::size_t;

    template<typename Subscriber, typename Worker>
    void operator()(const Subscriber& s, const Worker& w)
    {
      std::this_thread::sleep_for(period);
      ++ *count;
      s.on_next(*count);
      if (count_length <= *count)
      {
        s.on_completed();
        return;
      }

      w.schedule([this, s, w](const auto&)
        {
          (*this)(s, w);
        });
    }

    std::shared_ptr<std::size_t> count;
    std::size_t count_length;
    std::chrono::nanoseconds period;

  };

  class Active : public rmf_fleet_adapter::LegacyTask::ActivePhase
  {
  public:

    Active(
      std::string name,
      std::shared_ptr<std::size_t> count,
      std::size_t count_length,
      std::chrono::nanoseconds period)
    {
      _desc = "counting for " + name;

      _job = std::make_shared<Job>(count, count_length, period);

      _obs = rmf_rxcpp::make_job<std::size_t>(_job)
        .map([name = std::move(name)](const std::size_t v)
          {
            StatusMsg msg;
            msg.state = v;
            msg.status = name;
            return msg;
          });
    }

    const rxcpp::observable<StatusMsg>& observe() const final
    {
      return _obs;
    }

    rmf_traffic::Duration estimate_remaining_time() const final
    {
      return (_job->count_length - *_job->count) * _job->period;
    }

    void emergency_alarm(bool) final
    {
      // do nothing
    }

    void cancel() final
    {
      // do nothing
    }

    const std::string& description() const final
    {
      return _desc;
    }

  private:

    std::shared_ptr<Job> _job;
    rxcpp::observable<StatusMsg> _obs;
    std::string _desc;
  };


  class Pending : public rmf_fleet_adapter::LegacyTask::PendingPhase
  {
  public:

    Pending(
      std::string name,
      std::shared_ptr<std::size_t> count,
      std::size_t count_length,
      std::chrono::nanoseconds period)
    : _count(count),
      _count_length(count_length),
      _period(period),
      _desc(name)
    {
      // Do nothing
    }

    std::shared_ptr<rmf_fleet_adapter::LegacyTask::ActivePhase> begin() final
    {
      return std::make_shared<Active>(_desc, _count, _count_length, _period);
    }

    rmf_traffic::Duration estimate_phase_duration() const final
    {
      return (_count_length - *_count) * _period;
    }

    const std::string& description() const final
    {
      return _desc;
    }

  private:
    std::shared_ptr<std::size_t> _count;
    std::size_t _count_length;
    std::chrono::nanoseconds _period;
    std::string _desc;
  };
};

class MockSubtaskPhase
{
public:

  using PendingPhases = rmf_fleet_adapter::LegacyTask::PendingPhases;

  class Active
    : public rmf_fleet_adapter::LegacyTask::ActivePhase,
    public std::enable_shared_from_this<Active>
  {
  public:

    Active(PendingPhases phases)
    {
      rmf_task::State state;
      state.load_basic(
        {std::chrono::steady_clock::now(), 0, 0.0}, 0, 1.0);
      _subtasks =
        rmf_fleet_adapter::LegacyTask::make(
        "subtasks", std::move(phases),
        rxcpp::schedulers::make_event_loop().create_worker(),
        std::chrono::steady_clock::now(),
        state,
        nullptr);
      _desc = "subtasks";
      _status_obs = _status_publisher.get_observable();
    }

    void begin()
    {
      _subscription = _subtasks->observe()
        .observe_on(rxcpp::observe_on_event_loop())
        .subscribe(
        [weak = weak_from_this()](
          const StatusMsg& msg)
        {
          if (const auto phase = weak.lock())
            phase->_status_publisher.get_subscriber().on_next(msg);
        },
        [weak = weak_from_this()]()
        {
          if (const auto phase = weak.lock())
            phase->_status_publisher.get_subscriber().on_completed();
        });

      _subtasks->begin();
    }

    const rxcpp::observable<StatusMsg>& observe() const final
    {
      return _status_obs;
    }

    rmf_traffic::Duration estimate_remaining_time() const final
    {
      // TODO(MXG): We're not really testing this right now
      return rmf_traffic::Duration(0);
    }

    void emergency_alarm(bool) final
    {
      // do nothing
    }

    void cancel() final
    {
      // do nothing
    }

    const std::string& description() const final
    {
      return _desc;
    }

  private:

    std::shared_ptr<rmf_fleet_adapter::LegacyTask> _subtasks;
    rxcpp::subscription _subscription;
    rxcpp::subjects::subject<StatusMsg> _status_publisher;
    rxcpp::observable<StatusMsg> _status_obs;
    std::string _desc;

  };


  class Pending : public rmf_fleet_adapter::LegacyTask::PendingPhase
  {
  public:

    Pending(PendingPhases phases)
    : _phases(std::move(phases))
    {
      _desc = "subtasks";
    }

    std::shared_ptr<rmf_fleet_adapter::LegacyTask::ActivePhase> begin() final
    {
      auto active = std::make_shared<Active>(std::move(_phases));
      active->begin();
      return active;
    }

    rmf_traffic::Duration estimate_phase_duration() const final
    {
      // TODO(MXG): We're not really testing this right now
      return rmf_traffic::Duration(0);
    }

    const std::string& description() const final
    {
      return _desc;
    }

  private:
    PendingPhases _phases;
    std::string _desc;
  };

};

SCENARIO("Test simple task")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  std::shared_ptr<std::size_t> count = std::make_shared<std::size_t>(0);
  const auto dt = 10ms;

  rmf_fleet_adapter::LegacyTask::PendingPhases phases;
  phases.push_back(std::make_unique<MockPhase::Pending>("A", count, 5, dt));
  phases.push_back(std::make_unique<MockPhase::Pending>("B", count, 15, dt));
  phases.push_back(std::make_unique<MockPhase::Pending>("C", count, 18, dt));

  // Dummy parameters
  rmf_traffic::Time deployment_time;
  rmf_task::State finish_state;
  finish_state.load_basic({deployment_time, 0, 0.0}, 0, 1.0);

  std::shared_ptr<rmf_fleet_adapter::LegacyTask> task =
    rmf_fleet_adapter::LegacyTask::make(
    "test_Task", std::move(phases),
    rxcpp::schedulers::make_event_loop().create_worker(), deployment_time,
    finish_state, nullptr);

  std::promise<bool> completed_promise;
  auto completed_future = completed_promise.get_future();
  auto status_sub = task->observe()
    .subscribe(
    [](const rmf_fleet_adapter::LegacyTask::StatusMsg& msg)
    {
      if (msg.status.find("A") != std::string::npos)
      {
        CHECK(msg.state <= 5);
      }
      else if (msg.status.find("B") != std::string::npos)
      {
        CHECK(5 < msg.state);
        CHECK(msg.state <= 15);
      }
      else if (msg.status.find("C") != std::string::npos)
      {
        CHECK(15 < msg.state);
        CHECK(msg.state <= 18);
      }
    },
    [&completed_promise]()
    {
      completed_promise.set_value(true);
    });

  task->begin();

  // Wait 100x as long as what we're expecting in order to deal with any
  // possible overhead
  const auto status = completed_future.wait_for(18*dt * 1000);
  REQUIRE(status == std::future_status::ready);
  REQUIRE(completed_future.get());

  CHECK(*count == 18);
}

SCENARIO("Test nested task")
{
  rmf_fleet_adapter_test::thread_cooldown = true;

  using PendingPhases = rmf_fleet_adapter::LegacyTask::PendingPhases;

  std::shared_ptr<std::size_t> count = std::make_shared<std::size_t>(0);
  const auto dt = 10ms;

  std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>
  count_limits;

  count_limits["A"] = {0, 5};
  count_limits["B1"] = {5, 10};
  count_limits["B1"] = {0, 10};
  count_limits["B2"] = {10, 11};
  count_limits["B3"] = {11, 13};
  count_limits["C1"] = {13, 14};
  count_limits["C2"] = {14, 15};
  count_limits["C3"] = {15, 16};

  PendingPhases phases;

  phases.push_back(
    std::make_unique<MockPhase::Pending>("A", count, count_limits["A"].second,
    dt));

  PendingPhases b_phases;
  b_phases.push_back(
    std::make_unique<MockPhase::Pending>("B1", count, count_limits["B1"].second,
    dt));
  b_phases.push_back(
    std::make_unique<MockPhase::Pending>("B2", count, count_limits["B2"].second,
    dt));
  b_phases.push_back(
    std::make_unique<MockPhase::Pending>("B3", count, count_limits["B3"].second,
    dt));
  phases.push_back(
    std::make_unique<MockSubtaskPhase::Pending>(std::move(b_phases)));

  PendingPhases c_phases;
  c_phases.push_back(
    std::make_unique<MockPhase::Pending>("C1", count, count_limits["C1"].second,
    dt));
  c_phases.push_back(
    std::make_unique<MockPhase::Pending>("C2", count, count_limits["C2"].second,
    dt));
  c_phases.push_back(
    std::make_unique<MockPhase::Pending>("C3", count, count_limits["C3"].second,
    dt));
  phases.push_back(
    std::make_unique<MockSubtaskPhase::Pending>(std::move(c_phases)));

  // Dummy parameters
  rmf_traffic::Time deployment_time;
  rmf_task::State finish_state;
  finish_state.load_basic({deployment_time, 0, 0.0}, 0, 1.0);

  const auto task = rmf_fleet_adapter::LegacyTask::make(
    "test_NestedTask", std::move(phases),
    rxcpp::schedulers::make_event_loop().create_worker(), deployment_time,
    finish_state, nullptr);

  std::promise<bool> completed_promise;
  auto completed_future = completed_promise.get_future();
  auto status_sub = task->observe()
    .subscribe(
    [&count_limits](const rmf_fleet_adapter::LegacyTask::StatusMsg& msg)
    {
      std::pair<std::size_t, std::size_t> limits = {0, 0};
      for (const auto& element : count_limits)
      {
        if (msg.status.find(element.first) != std::string::npos)
        {
          limits = element.second;
          break;
        }
      }

      CHECK(limits.first < msg.state);
      CHECK(msg.state <= limits.second);
    },
    [&completed_promise]()
    {
      completed_promise.set_value(true);
    });

  task->begin();

  const auto status = completed_future.wait_for(16*dt * 1000);
  REQUIRE(status == std::future_status::ready);
  REQUIRE(completed_future.get());

  CHECK(*count == 16);
}
