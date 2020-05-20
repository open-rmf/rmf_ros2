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

#ifndef SRC__RMF_FLEET_ADAPTER__TASK_HPP
#define SRC__RMF_FLEET_ADAPTER__TASK_HPP

#include <string>

namespace rmf_fleet_adapter {

//==============================================================================
class Task
{
public:

  class ActivePhase
  {
  public:

    std::string current_status() const;



  };

  class PendingPhase
  {
  public:



  };

};

}

#endif // SRC__RMF_FLEET_ADAPTER__TASK_HPP
