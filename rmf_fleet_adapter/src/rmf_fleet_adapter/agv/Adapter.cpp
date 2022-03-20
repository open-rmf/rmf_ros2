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

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include "Node.hpp"
#include "internal_FleetUpdateHandle.hpp"

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/blockade/Writer.hpp>

#include "internal_EasyTrafficLight.hpp"

#include "../load_param.hpp"

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class WorkerWrapper : public rmf_traffic_ros2::schedule::Negotiation::Worker
{
public:

  WorkerWrapper(rxcpp::schedulers::worker worker)
  : _worker(std::move(worker))
  {
    // Do nothing
  }

  void schedule(std::function<void()> job) final
  {
    _worker.schedule([job = std::move(job)](const auto&) { job(); });
  }

private:
  rxcpp::schedulers::worker _worker;
};

//==============================================================================
class Adapter::Implementation
{
public:

  rxcpp::schedulers::worker worker;
  std::shared_ptr<Node> node;
  std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation;
  std::shared_ptr<ParticipantFactory> schedule_writer;
  std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer;
  rmf_traffic_ros2::schedule::MirrorManager mirror_manager;

  std::vector<std::shared_ptr<FleetUpdateHandle>> fleets = {};

  // TODO(MXG): This mutex probably isn't needed
  std::mutex _mutex;
  std::unique_lock<std::mutex> lock_mutex()
  {
    std::unique_lock<std::mutex> lock(_mutex, std::defer_lock);
    while (!lock.try_lock())
    {
      // Intentionally busy wait
    }

    return lock;
  }

  std::unordered_set<std::string> received_tasks;
  std::map<rmf_traffic::Time, std::string> task_times;
  rclcpp::TimerBase::SharedPtr task_purge_timer;

  // This mutex protects the initialization of traffic lights
  std::mutex _traffic_light_init_mutex;

  Implementation(
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<Node> node_,
    std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation_,
    std::shared_ptr<ParticipantFactory> writer_,
    rmf_traffic_ros2::schedule::MirrorManager mirror_manager_)
  : worker{std::move(worker_)},
    node{std::move(node_)},
    negotiation{std::move(negotiation_)},
    schedule_writer{std::move(writer_)},
    blockade_writer{rmf_traffic_ros2::blockade::Writer::make(*node)},
    mirror_manager{std::move(mirror_manager_)}
  {
    // Do nothing
  }

  static rmf_utils::unique_impl_ptr<Implementation> make(
    const std::string& node_name,
    const rclcpp::NodeOptions& node_options,
    rmf_utils::optional<rmf_traffic::Duration> discovery_timeout)
  {
    if (!rclcpp::ok(node_options.context()))
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "rclcpp must be initialized before creating an Adapter! Use "
        "rclcpp::init(int argc, char* argv[]) or "
        "rclcpp::Context::init(int argc, char* argv[]) before calling "
        "rmf_fleet_adapter::agv::Adapter::make(~)");
      // *INDENT-ON*
    }

    const auto worker = rxcpp::schedulers::make_event_loop().create_worker();
    auto node = Node::make(worker, node_name, node_options);

    if (!discovery_timeout)
    {
      discovery_timeout =
        get_parameter_or_default_time(*node, "discovery_timeout", 60.0);
    }

    auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
      node, rmf_traffic::schedule::query_all());

    auto writer = rmf_traffic_ros2::schedule::Writer::make(node);

    using namespace std::chrono_literals;

    const auto stop_time =
      std::chrono::steady_clock::now() + *discovery_timeout;

    rclcpp::ExecutorOptions options;
    options.context = node_options.context();
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(node);

    while (rclcpp::ok(node_options.context())
      && std::chrono::steady_clock::now() < stop_time)
    {
      executor.spin_some();

      bool ready = true;
      ready &= writer->ready();
      ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

      if (ready)
      {
        auto mirror_manager = mirror_future.get();

        auto negotiation =
          std::make_shared<rmf_traffic_ros2::schedule::Negotiation>(
          *node, mirror_manager.view(),
          std::make_shared<WorkerWrapper>(worker));

        return rmf_utils::make_unique_impl<Implementation>(
          worker,
          std::move(node),
          std::move(negotiation),
          std::make_shared<ParticipantFactoryRos2>(std::move(writer)),
          std::move(mirror_manager));
      }
    }

    return nullptr;
  }
};

//==============================================================================
std::shared_ptr<Adapter> Adapter::init_and_make(
  const std::string& node_name,
  rmf_utils::optional<rmf_traffic::Duration> discovery_timeout)
{
  rclcpp::NodeOptions options;
  options.context(std::make_shared<rclcpp::Context>());
  options.context()->init(0, nullptr);
  return make(node_name, options, discovery_timeout);
}

//==============================================================================
std::shared_ptr<Adapter> Adapter::make(
  const std::string& node_name,
  const rclcpp::NodeOptions& node_options,
  const std::optional<rmf_traffic::Duration> discovery_timeout)
{
  auto pimpl = Implementation::make(node_name, node_options, discovery_timeout);

  if (pimpl)
  {
    auto adapter = std::shared_ptr<Adapter>(new Adapter);
    adapter->_pimpl = std::move(pimpl);
    return adapter;
  }

  return nullptr;
}

//==============================================================================
std::shared_ptr<FleetUpdateHandle> Adapter::add_fleet(
  const std::string& fleet_name,
  rmf_traffic::agv::VehicleTraits traits,
  rmf_traffic::agv::Graph navigation_graph,
  std::optional<std::string> server_uri)
{
  auto planner =
    std::make_shared<std::shared_ptr<const rmf_traffic::agv::Planner>>(
    std::make_shared<rmf_traffic::agv::Planner>(
      rmf_traffic::agv::Planner::Configuration(
        std::move(navigation_graph),
        std::move(traits)),
      rmf_traffic::agv::Planner::Options(nullptr)));

  auto fleet = FleetUpdateHandle::Implementation::make(
    fleet_name, std::move(planner), _pimpl->node, _pimpl->worker,
    _pimpl->schedule_writer, _pimpl->mirror_manager.view(),
    _pimpl->negotiation, server_uri);

  _pimpl->fleets.push_back(fleet);
  return fleet;
}

//==============================================================================
void Adapter::add_easy_traffic_light(
  std::function<void(EasyTrafficLightPtr)> handle_callback,
  const std::string& fleet_name,
  const std::string& robot_name,
  rmf_traffic::agv::VehicleTraits traits,
  std::function<void()> pause_callback,
  std::function<void()> resume_callback,
  std::function<void(Blockers)> blocker_callback)
{
  if (!handle_callback)
  {
    RCLCPP_ERROR(
      _pimpl->node->get_logger(),
      "Adapter::add_easy_traffic_light(~) was not provided a callback to "
      "receive the TrafficLight::UpdateHandle for the robot [%s] owned by "
      "[%s]. This means the traffic light controller will not be able to work "
      "since you cannot provide information about where the robot is going. We "
      "will not create the requested traffic light controller.",
      robot_name.c_str(), fleet_name.c_str());

    return;
  }

  if (!pause_callback)
  {
    RCLCPP_ERROR(
      _pimpl->node->get_logger(),
      "Adapter::add_easy_traffic_light(~) was not provided a pause_callback "
      "value for the robot [%s] owned by [%s]. This means the easy traffic "
      "light controller will not be able to work correctly since we cannot "
      "command on-demand pauses. We will not create the requested easy traffic "
      "light controller.",
      robot_name.c_str(), fleet_name.c_str());
    return;
  }

  if (!resume_callback)
  {
    RCLCPP_ERROR(
      _pimpl->node->get_logger(),
      "Adapter::add_easy_traffic_light(~) was not provided a resume_callback "
      "value for the robot [%s] owned by [%s]. This means the easy traffic "
      "light controller will not be able to work correctly since we cannot "
      "command on-demand resuming. We will not create the requested easy "
      "traffic light controller.",
      robot_name.c_str(), fleet_name.c_str());
    return;
  }

  rmf_traffic::schedule::ParticipantDescription description(
    robot_name,
    fleet_name,
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    traits.profile());

  _pimpl->schedule_writer->async_make_participant(
    std::move(description),
    [mutex = &_pimpl->_traffic_light_init_mutex,
    traits = std::move(traits),
    pause_callback = std::move(pause_callback),
    resume_callback = std::move(resume_callback),
    handle_callback = std::move(handle_callback),
    blocker_callback = std::move(blocker_callback),
    blockade_writer = _pimpl->blockade_writer,
    schedule = _pimpl->mirror_manager.view(),
    worker = _pimpl->worker,
    handle_cb = std::move(handle_callback),
    negotiation = _pimpl->negotiation,
    node = _pimpl->node](
      rmf_traffic::schedule::Participant participant)
    {
      std::unique_lock<std::mutex> lock(*mutex, std::defer_lock);
      while (!lock.try_lock())
      {
        // Intententionally busy wait
      }

      RCLCPP_INFO(
        node->get_logger(),
        "Added a traffic light controller for [%s] with participant ID [%ld]",
        participant.description().name().c_str(),
        participant.id());

      EasyTrafficLightPtr easy_handle = EasyTrafficLight::Implementation::make(
        std::move(pause_callback),
        std::move(resume_callback),
        std::move(blocker_callback),
        schedule,
        worker,
        node,
        std::move(traits),
        std::move(participant),
        blockade_writer,
        negotiation.get());

      worker.schedule(
        [handle_callback = std::move(handle_callback),
        easy_handle = std::move(easy_handle)](const auto&)
        {
          handle_callback(std::move(easy_handle));
        });
    });
}

//==============================================================================
std::shared_ptr<rclcpp::Node> Adapter::node()
{
  return _pimpl->node;
}

//==============================================================================
std::shared_ptr<const rclcpp::Node> Adapter::node() const
{
  return _pimpl->node;
}

//==============================================================================
Adapter& Adapter::start()
{
  _pimpl->node->start();
  return *this;
}

//==============================================================================
Adapter& Adapter::stop()
{
  _pimpl->node->stop();
  return *this;
}

//==============================================================================
Adapter& Adapter::wait()
{
  std::mutex temp;
  std::unique_lock<std::mutex> lock(temp);
  _pimpl->node->spin_cv().wait(
    lock, [&]() { return !_pimpl->node->still_spinning(); });

  return *this;
}

//==============================================================================
Adapter& Adapter::wait_for(std::chrono::nanoseconds max_wait)
{
  const auto wait_until_time = std::chrono::steady_clock::now() + max_wait;
  std::mutex temp;
  std::unique_lock<std::mutex> lock(temp);
  _pimpl->node->spin_cv().wait_until(
    lock, wait_until_time, [&]()
    {
      return !_pimpl->node->still_spinning()
      && std::chrono::steady_clock::now() < wait_until_time;
    });

  return *this;
}

//==============================================================================
Adapter::Adapter()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
