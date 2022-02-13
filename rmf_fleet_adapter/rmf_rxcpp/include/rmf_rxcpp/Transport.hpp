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

class RxCppExecutor :
  public rclcpp::Executor,
  public std::enable_shared_from_this<RxCppExecutor>
{
public:
  RxCppExecutor(
    rxcpp::schedulers::worker worker,
    const rclcpp::ExecutorOptions& options = rclcpp::ExecutorOptions())
  : rclcpp::Executor{options},
    _worker{std::move(worker)},
    _started{false},
    _stopping{false},
    _work_scheduled{false}
  {
    // Do nothing
  }

  void spin() override
  {
    {
      std::unique_lock<std::mutex> lock(_starting_mutex);
      _stopping = false;
      _started = true;
    }

    _started_cv.notify_all();

    const auto keep_spinning = [&]()
      {
        return !_stopping && rclcpp::ok(context_);
      };

    while (keep_spinning())
    {
      _work_scheduled = true;
      _worker.schedule([w = weak_from_this()](const auto&)
        {
          if (const auto& self = w.lock())
          {
            self->spin_some();

            {
              std::lock_guard<std::mutex> lock(self->_mutex);
              self->_work_scheduled = false;
            }

            self->_cv.notify_all();
          }
        });

      {
        std::unique_lock<std::mutex> lock(_mutex);
        while (_work_scheduled && keep_spinning())
        {
          // If work is already scheduled to be done, wait until there is no
          // longer any work scheduled (or if we're no longer supposed to keep
          // spinning).
          _cv.wait_for(lock, std::chrono::milliseconds(50), [&]()
            {
              return !_work_scheduled || !keep_spinning();
            });
        }
      }

      // TODO(MXG): It would be better if we could check whether work is really
      // available before we keep looping. Or if we could interrupt an
      // indefinite wait when a stop is requested (maybe by creating a no-op
      // timer callback?).
      if (keep_spinning())
        wait_for_work(std::chrono::milliseconds(50));
    }

    _started = false;
    _stopping = false;
  }

  void stop()
  {
    _stopping = true;
    _cv.notify_all();
  }

  void wait_until_started()
  {
    while (!_started)
    {
      std::unique_lock<std::mutex> lock(_starting_mutex);
      _started_cv.wait(lock, [&]() { return _started.load(); });
    }
  }

private:
  rxcpp::schedulers::worker _worker;

  std::atomic_bool _started;
  std::mutex _starting_mutex;
  std::condition_variable _started_cv;

  std::atomic_bool _stopping;

  bool _work_scheduled;
  std::mutex _mutex;
  std::condition_variable _cv;
};

template<typename Message>
class SubscriptionBridge
{
public:

  SubscriptionBridge(
    rclcpp::Node::SharedPtr node,
    const std::string& topic_name,
    const rclcpp::QoS& qos)
  {
    _subscription = node->create_subscription<Message>(
      topic_name, qos,
      [publisher = _publisher](
        typename Message::SharedPtr msg)
      {
        publisher.get_subscriber().on_next(msg);
      });

    _observable = _publisher.get_observable();
  }

  const rxcpp::observable<typename Message::SharedPtr>& observe() const
  {
    return _observable;
  }

  ~SubscriptionBridge()
  {
    _publisher.get_subscriber().on_completed();
  }

private:
  rxcpp::subjects::subject<typename Message::SharedPtr> _publisher;
  rxcpp::observable<typename Message::SharedPtr> _observable;
  typename rclcpp::Subscription<Message>::SharedPtr _subscription;

};

// TODO(MXG): We define all the member functions of this class inline so that we
// don't need to export/install rmf_rxcpp as its own shared library (linking to
// it as a static library results in linking errors related to symbols not being
// relocatable). This is a quick and dirty solution to the linking problem, so
// we may want to consider replacing it with something cleaner.
class Transport : public rclcpp::Node
{
public:

  template<typename Message>
  using Bridge = std::shared_ptr<SubscriptionBridge<Message>>;

  explicit Transport(
    rxcpp::schedulers::worker worker,
    const std::string& node_name,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node{node_name, options},
    _executor{std::make_shared<RxCppExecutor>(
        worker, _make_exec_args(options))}
  {
    // Do nothing
  }

  void add_node(rclcpp::Node::SharedPtr node)
  {
    _executor->add_node(node);
  }

  void start()
  {
    std::unique_lock<std::mutex> lock(_stopping_mutex);
    if (!_stopped)
      return;

    if (!_node_added)
    {
      _executor->add_node(shared_from_this());
      _node_added = true;
    }

    _stopped = false;

    _spin_thread = std::thread([&]()
        {
          _executor->spin();
        });

    _executor->wait_until_started();
  }

  void stop()
  {
    {
      std::unique_lock<std::mutex> lock(_stopping_mutex);
      if (_stopped)
        return;

      _executor->stop();
      if (_spin_thread.joinable())
        _spin_thread.join();

      _stopped = true;
    }

    _stopped_cv.notify_all();
  }

  std::condition_variable& spin_cv()
  {
    return _stopped_cv;
  }

  bool still_spinning() const
  {
    return !_stopped && rclcpp::ok(get_node_options().context());
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
  Bridge<Message> create_observable(
    const std::string& topic_name, const rclcpp::QoS& qos)
  {
    return std::make_shared<
      SubscriptionBridge<Message>>(shared_from_this(), topic_name, qos);
  }

  ~Transport()
  {
    stop();
  }

private:
  std::mutex _stopping_mutex;
  bool _stopped = true;
  std::condition_variable _stopped_cv;

  std::shared_ptr<RxCppExecutor> _executor;
  bool _node_added = false;
  std::thread _spin_thread;

  static rclcpp::ExecutorOptions _make_exec_args(
    const rclcpp::NodeOptions& options)
  {
    rclcpp::ExecutorOptions exec_args;
    exec_args.context = options.context();
    return exec_args;
  }
};

} // namespace rmf_rxcpp

#endif //RMF_RXCPP__TRANSPORT_HPP
