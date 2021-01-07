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
#include <optional>

namespace rmf_rxcpp {

// TODO(MXG): We define all the member functions of this class inline so that we
// don't need to export/install rmf_rxcpp as its own shared library (linking to
// it as a static library results in linking errors related to symbols not being
// relocatable). This is a quick and dirty solution to the linking problem, so
// we may want to consider replacing it with something cleaner.
class Transport : public rclcpp::Node
{
public:

  explicit Transport(
      rxcpp::schedulers::worker worker,
      const std::string& node_name,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
      const std::optional<std::chrono::nanoseconds>& wait_time = std::nullopt)
    : rclcpp::Node{node_name, options},
      _worker{std::move(worker)},
      _executor(_make_exec_args(options)),
      _wait_time(wait_time)
  {
    // Do nothing
  }

  /**
   * Not threadsafe
   */
  void start()
  {
    if (!_stopping)
      return;

    if (!_node_added)
      _executor.add_node(shared_from_this());

    const auto sleep_param = "transport_sleep";
    declare_parameter<double>(sleep_param, 0.0);
    if (has_parameter(sleep_param))
    {
      auto param = get_parameter(sleep_param);
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        RCLCPP_WARN(get_logger(), "Expected parameter %s to be double", sleep_param);
      }
      else
      {
        auto sleep_time = param.as_double();
        if(sleep_time > 0)
        {
          _wait_time = {std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double, std::ratio<1,1000>>(sleep_time))};
        }
        RCLCPP_WARN(get_logger(), "Sleeping for %f", sleep_time);
      }
    }
    
    _stopping = false;
    _schedule_spin();
  }

  void stop()
  {
    _stopping = true;
  }

  std::condition_variable& spin_cv()
  {
    return _spin_cv;
  }

  bool still_spinning() const
  {
    return !_stopping && rclcpp::ok(get_node_options().context());
  }

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
  rxcpp::schedulers::worker _worker;
  rclcpp::executors::SingleThreadedExecutor _executor;
  bool _node_added = false;
  std::condition_variable _spin_cv;
  std::optional<std::chrono::nanoseconds> _wait_time;

  static rclcpp::ExecutorOptions _make_exec_args(
      const rclcpp::NodeOptions& options)
  {
    rclcpp::ExecutorOptions exec_args;
    exec_args.context = options.context();
    return exec_args;
  }

  void _do_spin()
  {
    _executor.spin_some();
    if (_wait_time)
    {
      rclcpp::sleep_for(*_wait_time);
    }
    if (still_spinning())
      _schedule_spin();
  }

  void _schedule_spin()
  {
    _worker.schedule(
          [w = weak_from_this()](const auto&)
    {
      if (const auto node = std::static_pointer_cast<Transport>(w.lock()))
        node->_do_spin();
    });
  }
};

} // namespace rmf_rxcpp

#endif //RMF_RXCPP__TRANSPORT_HPP
