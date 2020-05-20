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

#include <rmf_rxcpp/Transport.hpp>

namespace rmf_rxcpp {

//==============================================================================
void Transport::start()
{
  if (!_stopping)
    return;

  _stopping = false;
  _spin_thread = std::thread{[this]() { _do_spin(); }};
}

//==============================================================================
void Transport::stop()
{
  _stopping = true;
  if (_spin_thread.joinable())
    _spin_thread.join();
}

//==============================================================================
void Transport::_do_spin()
{
  rclcpp::executor::ExecutorArgs exec_args;
  exec_args.context = this->get_node_options().context();
  rclcpp::executors::SingleThreadedExecutor executor(exec_args);
  executor.add_node(shared_from_this());

  while (!_stopping)
    executor.spin_some();
}

} // namespace rmf_rxcpp
