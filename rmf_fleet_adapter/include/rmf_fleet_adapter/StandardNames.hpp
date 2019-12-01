/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef RMF_FLEET_ADAPTER__STANDARDNAMES_HPP
#define RMF_FLEET_ADAPTER__STANDARDNAMES_HPP

#include <string>

namespace rmf_fleet_adapter {

const std::string FleetStateTopicName = "fleet_states";
const std::string DestinationRequestTopicName = "destination_requests";
const std::string ModeRequestTopicName = "mode_requests";
const std::string PathRequestTopicName = "path_requests";

const std::string DoorRequestTopicName = "adapter_door_requests";
const std::string DoorStateTopicName = "door_states";

const std::string LiftRequestTopicName = "adapter_lift_requests";
const std::string LiftStateTopicName = "lift_states";

const std::string DispenserRequestTopicName = "dispenser_requests";
const std::string DispenserResultTopicName = "dispenser_requests";
const std::string DispenserStateTopicName = "dispenser_states";

const std::string DeliveryTopicName = "delivery_requests";
const std::string LoopRequestTopicName = "loop_requests";
const std::string TaskSummaryTopicName = "task_summaries";

} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__STANDARDNAMES_HPP
