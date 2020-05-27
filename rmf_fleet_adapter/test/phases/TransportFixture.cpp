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

#include "TransportFixture.hpp"

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

std::size_t TransportFixture::_node_counter = 0;
std::size_t TransportFixture::_topic_counter = 0;

TransportFixture::TransportFixture()
{
  _context = std::make_shared<rclcpp::Context>();
  _context->init(0, nullptr);

  transport = std::make_shared<rmf_rxcpp::Transport>(
    "test_transport_" + std::to_string(_node_counter++),
    rclcpp::NodeOptions().context(_context));

  transport->start();
}

TransportFixture::~TransportFixture()
{
  transport->stop();
  rclcpp::shutdown(_context);
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter