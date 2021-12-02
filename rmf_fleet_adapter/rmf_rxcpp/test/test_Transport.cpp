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

#include <rmf_rxcpp/Transport.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/contexts/default_context.hpp>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

std::size_t node_counter = 0;
std::size_t topic_counter = 0;

//==============================================================================
class TestAction : public std::enable_shared_from_this<TestAction>
{
public:

  TestAction(
    rmf_rxcpp::Transport::Bridge<std_msgs::msg::String> bridge,
    std::shared_ptr<bool> received)
  : _bridge(std::move(bridge)),
    _received(std::move(received))
  {
    // Do nothing
  }

  template<typename Subscriber, typename Worker>
  void operator()(const Subscriber& s, const Worker&)
  {
    _bridge->observe().subscribe(
      [s, received = _received](const auto& msg)
      {
        REQUIRE(msg);
        CHECK(msg->data == "hello");
        *received = true;
        s.on_completed();
      });
  }

  rmf_rxcpp::Transport::Bridge<std_msgs::msg::String> _bridge;
  std::shared_ptr<bool> _received;
};

//==============================================================================
class Empty {};

//==============================================================================
TEST_CASE("publish subscribe loopback", "[Transport]")
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  auto transport = std::make_shared<rmf_rxcpp::Transport>(
    rxcpp::schedulers::make_event_loop().create_worker(),
    "test_transport_" + std::to_string(node_counter++),
    rclcpp::NodeOptions().context(context));

  transport->start();

  const std::string topic_name = "test_topic_" +
    std::to_string(topic_counter++);
  auto publisher = transport->create_publisher<std_msgs::msg::String>(
    topic_name, 10);

  auto obs =
    transport->create_observable<std_msgs::msg::String>(topic_name, 10);

  SECTION("can receive subscription")
  {
    std_msgs::msg::String msg;
    msg.data = "hello";
    auto timer = transport->create_wall_timer(
      std::chrono::milliseconds(100),
      [publisher, msg]()
      {
        publisher->publish(msg);
      });

    auto received = std::make_shared<bool>(false);
    auto action = std::make_shared<TestAction>(obs, received);
    auto job = rmf_rxcpp::make_job<Empty>(action);
    job.as_blocking().subscribe();

    REQUIRE(received);
    CHECK(*received);
    CHECK(transport->count_subscribers(topic_name) == 1);
  }

  SECTION("multiple subscriptions are multiplexed")
  {
    std_msgs::msg::String msg;
    msg.data = "hello";
  }

  SECTION("multiple subscriptions are multiplexed")
  {
    rxcpp::composite_subscription subscription{};
    obs->observe().subscribe(subscription);
    obs->observe().subscribe(subscription);
    // TODO(geoff): This test is flaky due to a race condition between the subscription creation
    // above and the test condition check for the subscriber count below (it will randomly fail
    // with a subscriber count of 0). Running it in a resource-constrained environment, where it
    // only has access to a single CPU core, makes the test fail stably. The loop is used to give
    // the subscription a chance to go through prior to the test condition check. A more proper
    // solution will find out why subscriptions take time to go through and find or add a condition
    // we can wait on.
    int loop_count = 10;
    while (transport->count_subscribers(topic_name) == 0 && loop_count > 0)
    {
      std::this_thread::sleep_for(100ms);
      --loop_count;
    }
    REQUIRE(transport->count_subscribers(topic_name) == 1);
    subscription.unsubscribe();
  }
}
