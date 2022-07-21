/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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


#include <rmf_obstacle_ros2/ObstacleManager.hpp>
#include <rmf_obstacle_ros2/StandardNames.hpp>


#include <thread>
#include <iostream>
#include <mutex>
#include <condition_variable>

namespace rmf_obstacle_ros2 {

//==============================================================================
class ObstacleManager::Implementation
{
public:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<Obstacles>::SharedPtr obs_pub;
  ConstResponderPtr responder;
  std::thread spin_thread;
};

//==============================================================================
auto ObstacleManager::make(
  const std::string& name,
  ConstResponderPtr responder,
  const rclcpp::NodeOptions& node_options) -> std::shared_ptr<ObstacleManager>
{
  if (!rclcpp::ok(node_options.context()))
  {
    std::cout << "rclcpp must be initialized before creating an "
              << "ObstacleManager! Use rclcpp::init(int argc, char* argv[]) "
              << "or rclcpp::Context::init(int argc, char* argv[]) before  "
              << "calling rmf_obstacle_ros2::ObstacleManager::make(~)"
              << std::endl;
    return nullptr;
  }

  auto manager = std::shared_ptr<ObstacleManager>(new ObstacleManager);
  manager->_pimpl = rmf_utils::make_unique_impl<Implementation>();

  manager->_pimpl->node = std::make_shared<rclcpp::Node>(name, node_options);
  manager->_pimpl->obs_pub =
    manager->_pimpl->node->create_publisher<Obstacles>(
      ObstacleTopicName,
      rclcpp::QoS(10).reliable()
    );
  manager->_pimpl->responder = responder;
  return manager;

}

//==============================================================================
ObstacleManager::ObstacleManager()
{
  // Do nothing
}

//==============================================================================
ObstacleManager::~ObstacleManager()
{
  if (_pimpl->spin_thread.joinable())
    _pimpl->spin_thread.join();
}

//==============================================================================
std::shared_ptr<rclcpp::Node> ObstacleManager::node()
{
  return _pimpl->node;
}

//==============================================================================
std::shared_ptr<const rclcpp::Node> ObstacleManager::node() const
{
  return _pimpl->node;
}

//==============================================================================
ObstacleManager& ObstacleManager::start()
{
  _pimpl->spin_thread = std::thread(
    [n = _pimpl->node]()
    {
      while (rclcpp::ok())
        rclcpp::spin_some(n);
    });

  return *this;
}

//==============================================================================
ObstacleManager& ObstacleManager::wait()
{
  std::mutex temp;
  std::condition_variable cv;
  std::unique_lock<std::mutex> lock(temp);
  cv.wait(
    lock, [&]() { return !rclcpp::ok(); });

  return *this;
}

//==============================================================================
void ObstacleManager::process(const Obstacles& detections)
{
  // Simple implementation for now
  auto msg = std::make_unique<Obstacles>(detections);
  _pimpl->obs_pub->publish(std::move(msg));
  RCLCPP_DEBUG(
    _pimpl->node->get_logger(),
    "Published detection results with %ld obstacles.",
    detections.obstacles.size()
  );
  if (_pimpl->responder != nullptr)
  {
    _pimpl->responder->respond(detections);
    RCLCPP_DEBUG(
      _pimpl->node->get_logger(),
      "Passed detection results to responder %s",
      _pimpl->responder->name().c_str()
    );
  }
}

} // namespace rmf_obstacle_ros2
