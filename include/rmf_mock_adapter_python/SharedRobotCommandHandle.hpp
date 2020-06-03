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

#ifndef SHAREDROBOTCOMMANDHANDLE_HPP
#define SHAREDROBOTCOMMANDHANDLE_HPP

#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include <rmf_mock_adapter/adapter.hpp>

// Intermediate RobotCommandHandle wrapper to implement shared_pointer getter
class SharedRobotCommandHandle :
  public rmf_mock_adapter::RobotCommandHandle,
  public std::enable_shared_from_this<SharedRobotCommandHandle>
{
public:
  // Constructor
  SharedRobotCommandHandle() {}

  std::shared_ptr<SharedRobotCommandHandle> get_ptr()
  {
      return shared_from_this();
  }
};

#endif // SHAREDROBOTCOMMANDHANDLE_HPP
