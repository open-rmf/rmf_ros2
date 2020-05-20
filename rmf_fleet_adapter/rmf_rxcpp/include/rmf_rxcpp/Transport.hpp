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

#ifndef RMF_RXCPP__TRANSPORT_HPP
#define RMF_RXCPP__TRANSPORT_HPP

#include <rmf_rxcpp/detail/TransportDetail.hpp>
#include <rmf_rxcpp/RxJobs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rxcpp/rx.hpp>
#include <utility>

namespace rmf_rxcpp {

class Transport : public rclcpp::Node
{
public:

  explicit Transport(
      const std::string& node_name,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node{node_name, options}
  {
    // no op
  }

  /**
   * Not threadsafe
   */
  void start();

  void stop();

  /**
   * Creates a sharable observable that is bridged to a rclcpp subscription and is observed on an
   * event loop. When there are multiple subscribers, it multiplexes the message onto each
   * subscriber.
   * @tparam Message
   * @param topic_name
   * @param qos
   * @return
   */
  template<typename Message>
  auto create_observable(const std::string& topic_name, const rclcpp::QoS& qos)
  {
    auto wrapper = std::make_shared<detail::SubscriptionWrapper<Message>>(
      shared_from_this(), topic_name, qos);
    return rxcpp::observable<>::create<typename Message::SharedPtr>([wrapper](const auto& s)
    {
      (*wrapper)(s);
    }).publish().ref_count().observe_on(rxcpp::observe_on_event_loop());
  }

private:

  std::thread _spin_thread;
  bool _stopping = true;

  void _do_spin();
};

} // namespace rmf_rxcpp

#endif //RMF_RXCPP__TRANSPORT_HPP
