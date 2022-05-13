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

#include "RosPublisher.hpp"
#include "Scheduler.hpp"
#include "SqliteDataSource.hpp"
#include "SystemTimeExecutor.hpp"

#include <rmf_scheduler_msgs/msg/schedule.hpp>
#include <rmf_scheduler_msgs/msg/schedule_state.hpp>
#include <rmf_scheduler_msgs/msg/trigger.hpp>
#include <rmf_scheduler_msgs/msg/trigger_state.hpp>
#include <rmf_scheduler_msgs/srv/cancel_all.hpp>
#include <rmf_scheduler_msgs/srv/cancel_schedule.hpp>
#include <rmf_scheduler_msgs/srv/cancel_trigger.hpp>
#include <rmf_scheduler_msgs/srv/create_schedule.hpp>
#include <rmf_scheduler_msgs/srv/create_trigger.hpp>
#include <rmf_scheduler_msgs/srv/list_schedules.hpp>
#include <rmf_scheduler_msgs/srv/list_schedule_states.hpp>
#include <rmf_scheduler_msgs/srv/list_triggers.hpp>
#include <rmf_scheduler_msgs/srv/list_trigger_states.hpp>

#include <rclcpp/rclcpp.hpp>

namespace rmf::scheduler {

class SqliteDataSource;

class SchedulerNode : public rclcpp::Node
{
public:
  SystemTimeExecutor executor;

  SchedulerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

  void spin_tasks();

private:
  using Scheduler = rmf::scheduler::Scheduler<SystemTimeExecutor, RosPublisherFactory>;

  SqliteDataSource _store;
  Scheduler _scheduler;

  rclcpp::Publisher<rmf_scheduler_msgs::msg::Trigger>::SharedPtr
    _new_trigger_pub;
  rclcpp::Publisher<rmf_scheduler_msgs::msg::Schedule>::SharedPtr
    _new_schedule_pub;
  rclcpp::Publisher<rmf_scheduler_msgs::msg::TriggerState>::SharedPtr
    _trigger_update_pub;
  rclcpp::Publisher<rmf_scheduler_msgs::msg::ScheduleState>::SharedPtr
    _schedule_update_pub;
  rclcpp::Service<rmf_scheduler_msgs::srv::CreateTrigger>::SharedPtr
    _create_trigger_srv;
  rclcpp::Service<rmf_scheduler_msgs::srv::CreateSchedule>::SharedPtr
    _create_schedule_srv;
  rclcpp::Service<rmf_scheduler_msgs::srv::CancelTrigger>::SharedPtr
    _cancel_trigger_srv;
  rclcpp::Service<rmf_scheduler_msgs::srv::CancelSchedule>::SharedPtr
    _cancel_schedule_srv;
  rclcpp::Service<rmf_scheduler_msgs::srv::CancelAll>::SharedPtr
    _cancel_all_srv;
  rclcpp::Service<rmf_scheduler_msgs::srv::ListTriggers>::SharedPtr
    _list_triggers_srv;
  rclcpp::Service<rmf_scheduler_msgs::srv::ListTriggerStates>::SharedPtr
    _list_trigger_states_srv;
  rclcpp::Service<rmf_scheduler_msgs::srv::ListSchedules>::SharedPtr
    _list_schedules_srv;
  rclcpp::Service<rmf_scheduler_msgs::srv::ListScheduleStates>::SharedPtr
    _list_schedule_states_srv;

  SqliteDataSource _create_store();

  void _create_trigger(
    rmf_scheduler_msgs::srv::CreateTrigger::Request::SharedPtr req,
    rmf_scheduler_msgs::srv::CreateTrigger::Response::SharedPtr resp);

  void _create_schedule(
    rmf_scheduler_msgs::srv::CreateSchedule::Request::SharedPtr req,
    rmf_scheduler_msgs::srv::CreateSchedule::Response::SharedPtr resp);

  void _cancel_trigger(
    rmf_scheduler_msgs::srv::CancelTrigger::Request::SharedPtr req,
    rmf_scheduler_msgs::srv::CancelTrigger::Response::SharedPtr resp);

  void _cancel_schedule(
    rmf_scheduler_msgs::srv::CancelSchedule::Request::SharedPtr req,
    rmf_scheduler_msgs::srv::CancelSchedule::Response::SharedPtr resp);

  void _cancel_all(
    rmf_scheduler_msgs::srv::CancelAll::Request::SharedPtr req,
    rmf_scheduler_msgs::srv::CancelAll::Response::SharedPtr resp);

  void _list_triggers(
    rmf_scheduler_msgs::srv::ListTriggers::Request::SharedPtr req,
    rmf_scheduler_msgs::srv::ListTriggers::Response::SharedPtr resp);

  void _list_trigger_states(
    rmf_scheduler_msgs::srv::ListTriggerStates::Request::SharedPtr req,
    rmf_scheduler_msgs::srv::ListTriggerStates::Response::SharedPtr resp);

  void _list_schedules(
    rmf_scheduler_msgs::srv::ListSchedules::Request::SharedPtr req,
    rmf_scheduler_msgs::srv::ListSchedules::Response::SharedPtr resp);

  void _list_schedule_states(
    rmf_scheduler_msgs::srv::ListScheduleStates::Request::SharedPtr req,
    rmf_scheduler_msgs::srv::ListScheduleStates::Response::SharedPtr resp);
};

}
