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

#include <cstdint>
#include <string>

namespace rmf_fleet_adapter {

const std::string FleetStateTopicName = "/fleet_states";
const std::string DestinationRequestTopicName = "destination_requests";
const std::string ModeRequestTopicName = "robot_mode_requests";
const std::string PathRequestTopicName = "robot_path_requests";
const std::string PauseRequestTopicName = "robot_pause_requests";
const std::string FleetStateUpdateTopicName = "fleet_state_update";
const std::string FleetLogUpdateTopicName = "fleet_log_update";

const std::string FinalDoorRequestTopicName = "door_requests";
const std::string AdapterDoorRequestTopicName = "adapter_door_requests";
const std::string DoorStateTopicName = "door_states";
const std::string DoorSupervisorHeartbeatTopicName =
  "door_supervisor_heartbeat";

const std::string FinalLiftRequestTopicName = "lift_requests";
const std::string AdapterLiftRequestTopicName = "adapter_lift_requests";
const std::string LiftStateTopicName = "lift_states";

const std::string DispenserRequestTopicName = "dispenser_requests";
const std::string DispenserResultTopicName = "dispenser_results";
const std::string DispenserStateTopicName = "dispenser_states";

const std::string IngestorRequestTopicName = "ingestor_requests";
const std::string IngestorResultTopicName = "ingestor_results";
const std::string IngestorStateTopicName = "ingestor_states";

const std::string DeliveryTopicName = "delivery_requests";
const std::string LoopRequestTopicName = "loop_requests";
const std::string TaskSummaryTopicName = "task_summaries";

const std::string BidNoticeTopicName = "rmf_task/bid_notice";
const std::string BidProposalTopicName = "rmf_task/bid_proposal";
const std::string DispatchRequestTopicName = "rmf_task/dispatch_request";
const std::string DispatchAckTopicName = "rmf_task/dispatch_ack";

const std::string DockSummaryTopicName = "dock_summary";

const std::string NavGraphTopicName = "nav_graphs";
const std::string LaneClosureRequestTopicName = "lane_closure_requests";
const std::string ClosedLaneTopicName = "closed_lanes";
const std::string SpeedLimitRequestTopicName = "speed_limit_requests";
const std::string LaneStatesTopicName = "lane_states";

const std::string InterruptRequestTopicName = "robot_interrupt_request";

const std::string TaskApiRequests = "task_api_requests";
const std::string TaskApiResponses = "task_api_responses";
const std::string TaskStateUpdateTopicName = "task_state_update";
const std::string TaskLogUpdateTopicName = "task_log_update";

const std::string ChargingAssignmentsTopicName = "charging_assignments";

const std::string MutexGroupRequestTopicName = "mutex_group_request";
const std::string MutexGroupStatesTopicName = "mutex_group_states";
const std::string MutexGroupManualReleaseTopicName =
  "mutex_group_manual_release";

const std::string ReservationRequestTopicName = "rmf/reservations/request";
const std::string ReservationResponseTopicName = "rmf/reservations/tickets";
const std::string ReservationClaimTopicName = "rmf/reservations/claim";
const std::string ReservationAllocationTopicName =
  "rmf/reservations/allocation";
const std::string ReservationReleaseTopicName = "rmf/reservations/release";

const std::string DynamicEventBeginTopicBase = "rmf/dynamic_event/begin";
const std::string DynamicEventStatusTopicBase = "rmf/dynamic_event/status";
const std::string DynamicEventActionName = "rmf/dynamic_event/command";

const uint64_t Unclaimed = (uint64_t)(-1);

} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__STANDARDNAMES_HPP
