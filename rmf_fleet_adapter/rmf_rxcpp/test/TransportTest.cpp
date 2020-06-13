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

std::size_t node_counter = 0;
std::size_t topic_counter = 0;

TEST_CASE("publish subscribe loopback", "[Transport]")
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  auto transport = std::make_shared<rmf_rxcpp::Transport>(
        rxcpp::schedulers::make_event_loop().create_worker(),
        "test_transport_" + std::to_string(node_counter++),
        rclcpp::NodeOptions().context(context));

  transport->start();

  const std::string topic_name = "test_topic_" + std::to_string(topic_counter++);
  auto publisher = transport->create_publisher<std_msgs::msg::String>(topic_name, 10);
  auto obs = transport->create_observable<std_msgs::msg::String>(topic_name, 10);

  SECTION("can receive subscription")
  {
    std_msgs::msg::String msg{};
    msg.data = "hello";
    auto timer = transport->create_wall_timer(std::chrono::milliseconds(100), [&publisher, &msg]()
    {
      publisher->publish(msg);
    });

    bool received = false;
    auto j = rmf_rxcpp::make_leaky_job<std_msgs::msg::String>([&obs, &received](const auto& s)
    {
      obs.subscribe([s, &received](const auto&)
      {
        received = true;
        s.on_completed();
      });
    });
    j.as_blocking().subscribe();
    REQUIRE(received);
    REQUIRE(transport->count_subscribers(topic_name) == 1);
  }

  SECTION("multiple subscriptions are multiplexed")
  {
    rxcpp::composite_subscription subscription{};
    obs.subscribe(subscription);
    obs.subscribe(subscription);
    REQUIRE(transport->count_subscribers(topic_name) == 1);
    subscription.unsubscribe();
  }

  transport->stop();
  rclcpp::shutdown(context);
}
