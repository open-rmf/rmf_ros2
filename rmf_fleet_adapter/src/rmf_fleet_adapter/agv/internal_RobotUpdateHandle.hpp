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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ROBOTUPDATEHANDLE_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ROBOTUPDATEHANDLE_HPP

#include "RobotContext.hpp"
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class RobotUpdateHandle::Implementation
{
public:

  std::weak_ptr<RobotContext> context;
  std::string name;
  RobotUpdateHandle::Unstable unstable = RobotUpdateHandle::Unstable();
  bool reported_loss = false;

  static std::shared_ptr<RobotUpdateHandle> make(RobotContextPtr context)
  {
    std::string name = context->description().name();

    RobotUpdateHandle handle;
    handle._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::move(context), std::move(name)});
    handle._pimpl->unstable._pimpl =
      &RobotUpdateHandle::Implementation::get(handle);
    return std::make_shared<RobotUpdateHandle>(std::move(handle));
  }

  static Implementation& get(RobotUpdateHandle& handle)
  {
    return *handle._pimpl;
  }

  static const Implementation& get(const RobotUpdateHandle& handle)
  {
    return *handle._pimpl;
  }

  std::shared_ptr<RobotContext> get_context();

  std::shared_ptr<const RobotContext> get_context() const;

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ROBOTUPDATEHANDLE_HPP
