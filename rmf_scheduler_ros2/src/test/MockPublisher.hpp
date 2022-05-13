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

#pragma once

#include <rmf_scheduler_msgs/msg/payload.hpp>

namespace rmf::scheduler::test {

class MockPublisher
{
public:
  using PayloadData = decltype(rmf_scheduler_msgs::msg::Payload::data);

  std::vector<PayloadData> publishes;

  void publish(uint8_t, const PayloadData& data)
  {
    this->publishes.push_back(data);
  }
};

class MockPublisherFactory
{
public:
  using Publisher = MockPublisher&;

  MockPublisher& publisher;

  MockPublisherFactory(MockPublisher& publisher) : publisher(publisher) {}

  MockPublisher& operator()(const std::string&, const std::string&)
  {
    return this->publisher;
  }
};

}
