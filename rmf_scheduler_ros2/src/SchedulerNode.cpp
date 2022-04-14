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

#include "SchedulerNode.hpp"

#include "SqliteDataSource.hpp"

namespace rmf::scheduler {

SchedulerNode::SchedulerNode(const rclcpp::NodeOptions& options)
: rclcpp::Node{"rmf_scheduler", options},
  _store{this->_create_store()},
  _scheduler{this->executor, this->_store, this->_publisher}
{
  this->_new_trigger_pub =
    this->create_publisher<rmf_scheduler_msgs::msg::Trigger>("new_trigger",
      rclcpp::SystemDefaultsQoS{});

  this->_new_schedule_pub =
    this->create_publisher<rmf_scheduler_msgs::msg::Schedule>("new_schedule",
      rclcpp::SystemDefaultsQoS{});

  this->_trigger_update_pub =
    this->create_publisher<rmf_scheduler_msgs::msg::TriggerState>(
    "trigger_update", rclcpp::SystemDefaultsQoS{});

  this->_schedule_update_pub =
    this->create_publisher<rmf_scheduler_msgs::msg::ScheduleState>(
    "schedule_update", rclcpp::SystemDefaultsQoS{});

  this->_create_trigger_srv =
    this->create_service<rmf_scheduler_msgs::srv::CreateTrigger>(
    "create_trigger",
    [this](
      rmf_scheduler_msgs::srv::CreateTrigger::Request::SharedPtr req,
      rmf_scheduler_msgs::srv::CreateTrigger::Response::SharedPtr resp)
    {
      this->_create_trigger(req, resp);
    });

  this->_create_schedule_srv =
    this->create_service<rmf_scheduler_msgs::srv::CreateSchedule>(
    "create_schedule",
    [this](
      rmf_scheduler_msgs::srv::CreateSchedule::Request::SharedPtr req,
      rmf_scheduler_msgs::srv::CreateSchedule::Response::SharedPtr resp)
    {
      this->_create_schedule(req, resp);
    });

  this->_cancel_trigger_srv =
    this->create_service<rmf_scheduler_msgs::srv::CancelTrigger>(
    "cancel_trigger",
    [this](
      rmf_scheduler_msgs::srv::CancelTrigger::Request::SharedPtr req,
      rmf_scheduler_msgs::srv::CancelTrigger::Response::SharedPtr resp)
    {
      this->_cancel_trigger(req, resp);
    });

  this->_cancel_schedule_srv =
    this->create_service<rmf_scheduler_msgs::srv::CancelSchedule>(
    "cancel_schedule",
    [this](
      rmf_scheduler_msgs::srv::CancelSchedule::Request::SharedPtr req,
      rmf_scheduler_msgs::srv::CancelSchedule::Response::SharedPtr resp)
    {
      this->_cancel_schedule(req, resp);
    });

  // this->_list_triggers_srv =
  //   this->create_service<rmf_scheduler_msgs::srv::ListTriggers>(
  //   "list_triggers",
  //   [this](
  //     rmf_scheduler_msgs::srv::ListTriggers::Request::SharedPtr req,
  //     rmf_scheduler_msgs::srv::ListTriggers::Response::SharedPtr resp)
  //   {
  //     this->_list_triggers(req, resp);
  //   });

  // this->_list_trigger_states_srv =
  //   this->create_service<rmf_scheduler_msgs::srv::ListTriggerStates>(
  //   "list_trigger_states",
  //   [this](
  //     rmf_scheduler_msgs::srv::ListTriggerStates::Request::SharedPtr req,
  //     rmf_scheduler_msgs::srv::ListTriggerStates::Response::SharedPtr resp)
  //   {
  //     this->_list_trigger_states(req, resp);
  //   });

  // this->_list_schedules_srv =
  //   this->create_service<rmf_scheduler_msgs::srv::ListSchedules>(
  //   "list_schedules",
  //   [this](
  //     rmf_scheduler_msgs::srv::ListSchedules::Request::SharedPtr req,
  //     rmf_scheduler_msgs::srv::ListSchedules::Response::SharedPtr resp)
  //   {
  //     this->_list_schedules(req, resp);
  //   });

  // this->_list_schedule_states_srv =
  //   this->create_service<rmf_scheduler_msgs::srv::ListScheduleStates>(
  //   "list_schedule_states",
  //   [this](
  //     rmf_scheduler_msgs::srv::ListScheduleStates::Request::SharedPtr req,
  //     rmf_scheduler_msgs::srv::ListScheduleStates::Response::SharedPtr resp)
  //   {
  //     this->_list_schedule_states(req, resp);
  //   });

  this->_scheduler.on_trigger_update = [this](const auto& state)
    {
      this->_trigger_update_pub->publish(state);
    };

  this->_scheduler.on_schedule_update = [this](const auto& state)
    {
      this->_schedule_update_pub->publish(state);
    };
}

SqliteDataSource SchedulerNode::_create_store()
{
  this->declare_parameter("db_file");
  auto db_file = this->get_parameter("db_file").as_string();
  return SqliteDataSource{db_file};
}

void SchedulerNode::_create_trigger(
  rmf_scheduler_msgs::srv::CreateTrigger::Request::SharedPtr req,
  rmf_scheduler_msgs::srv::CreateTrigger::Response::SharedPtr resp)
{
  try
  {
    this->_scheduler.create_trigger(req->trigger);
    this->_new_trigger_pub->publish(req->trigger);
    resp->success = true;
  }
  catch (const std::exception& e)
  {
    resp->success = false;
    resp->message = e.what();
  }
}

void SchedulerNode::_create_schedule(
  rmf_scheduler_msgs::srv::CreateSchedule::Request::SharedPtr req,
  rmf_scheduler_msgs::srv::CreateSchedule::Response::SharedPtr resp)
{
  try
  {
    this->_scheduler.create_schedule(req->schedule);
    this->_new_schedule_pub->publish(req->schedule);
    resp->success = true;
  }
  catch (const std::exception& e)
  {
    resp->success = false;
    resp->message = e.what();
  }
}

void SchedulerNode::_cancel_trigger(
  rmf_scheduler_msgs::srv::CancelTrigger::Request::SharedPtr req,
  rmf_scheduler_msgs::srv::CancelTrigger::Response::SharedPtr resp)
{
  try
  {
    this->_scheduler.cancel_trigger(req->name);
    resp->success = true;
  }
  catch (const std::exception& e)
  {
    resp->success = false;
    resp->message = e.what();
  }
}

void SchedulerNode::_cancel_schedule(
  rmf_scheduler_msgs::srv::CancelSchedule::Request::SharedPtr req,
  rmf_scheduler_msgs::srv::CancelSchedule::Response::SharedPtr resp)
{
  try
  {
    this->_scheduler.cancel_schedule(req->name);
    resp->success = true;
  }
  catch (const std::exception& e)
  {
    resp->success = false;
    resp->message = e.what();
  }
}

void SchedulerNode::_list_triggers(
  rmf_scheduler_msgs::srv::ListTriggers::Request::SharedPtr req,
  rmf_scheduler_msgs::srv::ListTriggers::Response::SharedPtr resp)
{
  resp->success = false;
  resp->message = "not implemented";
}

void SchedulerNode::_list_trigger_states(
  rmf_scheduler_msgs::srv::ListTriggerStates::Request::SharedPtr req,
  rmf_scheduler_msgs::srv::ListTriggerStates::Response::SharedPtr resp)
{
  resp->success = false;
  resp->message = "not implemented";
}

void SchedulerNode::_list_schedules(
  rmf_scheduler_msgs::srv::ListSchedules::Request::SharedPtr req,
  rmf_scheduler_msgs::srv::ListSchedules::Response::SharedPtr resp)
{
  resp->success = false;
  resp->message = "not implemented";
}

void SchedulerNode::_list_schedule_states(
  rmf_scheduler_msgs::srv::ListScheduleStates::Request::SharedPtr req,
  rmf_scheduler_msgs::srv::ListScheduleStates::Response::SharedPtr resp)
{
  resp->success = false;
  resp->message = "not implemented";
}

}