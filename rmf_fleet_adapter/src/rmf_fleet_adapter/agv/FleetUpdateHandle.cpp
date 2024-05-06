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

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>
#include <rmf_fleet_msgs/msg/speed_limited_lane.hpp>

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/agv/Graph.hpp>

#include "internal_FleetUpdateHandle.hpp"
#include "internal_RobotUpdateHandle.hpp"
#include "RobotContext.hpp"

#include "../log_to_json.hpp"
#include "../tasks/Delivery.hpp"
#include "../tasks/Patrol.hpp"
#include "../tasks/Clean.hpp"
#include "../tasks/ChargeBattery.hpp"
#include "../tasks/Compose.hpp"
#include "../events/GoToPlace.hpp"
#include "../events/ResponsiveWait.hpp"
#include "../events/PerformAction.hpp"

#include <rmf_task/Constraints.hpp>
#include <rmf_task/Parameters.hpp>
#include <rmf_task/requests/Clean.hpp>
#include <rmf_task/requests/Delivery.hpp>
#include <rmf_task/requests/Loop.hpp>

#include <rmf_task_sequence/phases/SimplePhase.hpp>

#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>

#include <rmf_fleet_adapter/schemas/place.hpp>
#include <rmf_api_msgs/schemas/task_request.hpp>

namespace rmf_fleet_adapter {
namespace agv {

namespace {
//==============================================================================
class LiaisonNegotiator : public rmf_traffic::schedule::Negotiator
{
public:

  LiaisonNegotiator(
    std::shared_ptr<rmf_traffic::schedule::Negotiator> negotiator)
  : w_negotiator(negotiator)
  {
    // Do nothing
  }

  std::weak_ptr<rmf_traffic::schedule::Negotiator> w_negotiator;

  void respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder) final
  {
    const auto negotiator = w_negotiator.lock();
    if (!negotiator)
    {
      // If we no longer have access to the upstream negotiator, then we simply
      // forfeit.
      //
      // TODO(MXG): Consider issuing a warning here
      return responder->forfeit({});
    }

    negotiator->respond(table_viewer, responder);
  }

};
} // anonymous namespace

//==============================================================================
void TaskDeserialization::add_schema(const nlohmann::json& schema)
{
  _schema_dictionary->insert_or_assign(
    nlohmann::json_uri(schema["$id"]).url(), schema);
}

//==============================================================================
nlohmann::json_schema::json_validator TaskDeserialization::make_validator(
  nlohmann::json schema) const
{
  return nlohmann::json_schema::json_validator(std::move(schema), _loader);
}

//==============================================================================
std::shared_ptr<nlohmann::json_schema::json_validator>
TaskDeserialization::make_validator_shared(nlohmann::json schema) const
{
  return std::make_shared<nlohmann::json_schema::json_validator>(
    make_validator(std::move(schema)));
}

//==============================================================================
TaskDeserialization::TaskDeserialization()
{
  task = std::make_shared<DeserializeJSON<DeserializedTask>>();
  phase = std::make_shared<DeserializeJSON<DeserializedPhase>>();
  event = std::make_shared<DeserializeJSON<DeserializedEvent>>();
  consider_actions =
    std::make_shared<std::unordered_map<
        std::string, FleetUpdateHandle::ConsiderRequest>>();

  _schema_dictionary = std::make_shared<SchemaDictionary>();
  _loader = [dict = _schema_dictionary](
    const nlohmann::json_uri& id,
    nlohmann::json& value)
    {
      const auto it = dict->find(id.url());
      if (it == dict->end())
        return;

      value = it->second;
    };
}


//==============================================================================
void FleetUpdateHandle::Implementation::publish_nav_graph() const
{
  if (nav_graph_pub == nullptr)
    return;

  auto msg = rmf_traffic_ros2::convert(
    (*planner)->get_configuration().graph(),
    name);

  if (msg != nullptr)
    nav_graph_pub->publish(std::move(msg));
}

//==============================================================================
void FleetUpdateHandle::Implementation::dock_summary_cb(
  const DockSummary::SharedPtr& msg)
{
  for (const auto& dock : msg->docks)
  {
    if (dock.fleet_name == name)
    {
      dock_param_map->clear();
      for (const auto& param : dock.params)
        dock_param_map->insert({param.start, param});
      break;
    }
  }

  return;
}

//==============================================================================
std::shared_ptr<rmf_task::Request> FleetUpdateHandle::Implementation::convert(
  const std::string& task_id,
  const nlohmann::json& request_msg,
  std::vector<std::string>& errors) const
{
  const auto& category = request_msg["category"].get<std::string>();

  const auto task_deser_it = deserialization.task->handlers.find(category);
  if (task_deser_it == deserialization.task->handlers.end())
  {
    errors.push_back(make_error_str(
        4, "Unsupported type",
        "Fleet [" + name + "] does not support task category ["
        + category + "]"));
    return nullptr;
  }

  const auto& description_msg = request_msg["description"];
  const auto& task_deser_handler = task_deser_it->second;

  try
  {
    task_deser_handler.validator->validate(description_msg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Received a request description for [%s] with an invalid format. "
      "Error: %s\nRequest:\n%s",
      category.c_str(),
      e.what(),
      description_msg.dump(2, ' ').c_str());

    errors.push_back(make_error_str(5, "Invalid request format", e.what()));
    return nullptr;
  }

  const auto deserialized_task =
    task_deser_handler.deserializer(description_msg);

  if (!deserialized_task.description)
  {
    errors = deserialized_task.errors;
    return nullptr;
  }

  rmf_traffic::Time earliest_start_time = rmf_traffic_ros2::convert(
    node->get_clock()->now());
  const auto t_it = request_msg.find("unix_millis_earliest_start_time");
  if (t_it != request_msg.end())
  {
    earliest_start_time =
      rmf_traffic::Time(std::chrono::milliseconds(t_it->get<int64_t>()));
  }

  rmf_traffic::Time request_time = rmf_traffic_ros2::convert(
    node->get_clock()->now());
  const auto r_it = request_msg.find("unix_millis_request_time");
  if (r_it != request_msg.end())
  {
    request_time =
      rmf_traffic::Time(std::chrono::milliseconds(r_it->get<int64_t>()));
  }

  // Note: make_low_priority() actually returns a nullptr.
  rmf_task::ConstPriorityPtr priority =
    rmf_task::BinaryPriorityScheme::make_low_priority();
  const auto p_it = request_msg.find("priority");
  if (p_it != request_msg.end())
  {
    // Assume the schema is not valid until we have successfully parsed it.
    bool valid_schema = false;
    // TODO(YV): Validate with priority_description_Binary.json
    if (p_it->contains("type") && p_it->contains("value"))
    {
      const auto& p_type = (*p_it)["type"];
      if (p_type.is_string() && p_type.get<std::string>() == "binary")
      {
        const auto& p_value = (*p_it)["value"];
        if (p_value.is_number_integer())
        {
          // The message matches the expected schema, so now we can mark it as
          // valid.
          valid_schema = true;

          // If we have an integer greater than 0, we assign a high priority.
          // Else the priority will default to low.
          if (p_value.get<uint64_t>() > 0)
          {
            priority = rmf_task::BinaryPriorityScheme::make_high_priority();
          }
        }
      }
    }

    if (!valid_schema)
    {
      errors.push_back(
        make_error_str(
          4, "Unsupported type",
          "Fleet [" + name + "] does not support priority request: "
          + p_it->dump() + "\nDefaulting to low binary priority."));
    }
  }

  std::optional<std::string> requester = std::nullopt;
  const auto i_it = request_msg.find("requester");
  if (i_it != request_msg.end())
  {
    requester = i_it->get<std::string>();
  }

  std::vector<std::string> labels = {};
  const auto labels_it = request_msg.find("labels");
  if (labels_it != request_msg.end())
    labels = labels_it->get<std::vector<std::string>>();

  rmf_task::Task::ConstBookingPtr booking = requester.has_value() ?
    std::make_shared<const rmf_task::Task::Booking>(
    task_id,
    earliest_start_time,
    priority,
    requester.value(),
    request_time,
    false,
    labels) :
    std::make_shared<const rmf_task::Task::Booking>(
    task_id,
    earliest_start_time,
    priority,
    false,
    labels);
  const auto new_request = std::make_shared<rmf_task::Request>(
    std::move(booking),
    deserialized_task.description);

  return new_request;
}

//==============================================================================
class AllocateTasks : public std::enable_shared_from_this<AllocateTasks>
{
public:

  template<typename Subscriber>
  void operator()(const Subscriber& s)
  {
    std::vector<std::string> errors;
    auto assignments = run(errors);
    s.on_next(Result{std::move(assignments), std::move(errors)});
    s.on_completed();
  }

  std::optional<TaskAssignments> run(std::vector<std::string>& errors)
  {
    std::string id = "";

    if (new_request)
    {
      expect.pending_requests.push_back(new_request);
      id = new_request->booking()->id();
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Planning for [%ld] robot(s) and [%ld] request(s)",
      expect.states.size(),
      expect.pending_requests.size());

    std::unordered_map<std::size_t, RobotContextPtr> robot_indexes;
    std::vector<rmf_task::State> states;
    for (const auto& [context, state] : expect.states)
    {
      robot_indexes[states.size()] = context;
      states.push_back(state);
    }

    // Generate new task assignments
    const auto result = task_planner.plan(
      rmf_traffic_ros2::convert(node->now()),
      states,
      expect.pending_requests);

    auto assignments_ptr = std::get_if<
      rmf_task::TaskPlanner::Assignments>(&result);

    if (!assignments_ptr)
    {
      auto error = std::get_if<
        rmf_task::TaskPlanner::TaskPlannerError>(&result);

      if (*error == rmf_task::TaskPlanner::TaskPlannerError::low_battery)
      {
        std::string error_str =
          "[TaskPlanner] Failed to compute assignments for task_id [" + id
          +
          "] due to insufficient initial battery charge for all robots in this "
          "fleet.";

        RCLCPP_ERROR(node->get_logger(), "%s", error_str.c_str());
        errors.push_back(
          make_error_str(9, "Not feasible", std::move(error_str)));
      }

      else if (*error ==
        rmf_task::TaskPlanner::TaskPlannerError::limited_capacity)
      {
        std::string error_str =
          "[TaskPlanner] Failed to compute assignments for task_id [" + id
          + "] due to insufficient battery capacity to accommodate one or more "
          "requests by any of the robots in this fleet.";

        RCLCPP_ERROR(node->get_logger(), "%s", error_str.c_str());
        errors.push_back(
          make_error_str(9, "Not feasible", std::move(error_str)));
      }

      else
      {
        std::string error_str =
          "[TaskPlanner] Failed to compute assignments for task_id [" + id +
          "]";

        RCLCPP_ERROR(node->get_logger(), "%s", error_str.c_str());
        errors.push_back(
          make_error_str(9, "Not feasible", std::move(error_str)));
      }

      return std::nullopt;
    }

    TaskAssignments assignments;
    for (std::size_t i = 0; i < assignments_ptr->size(); ++i)
    {
      const auto r_it = robot_indexes.find(i);
      if (r_it == robot_indexes.end())
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "[AllocateTasks] Unable to find a robot associated with index [%d]",
          i);
        continue;
      }

      const auto context = r_it->second;
      assignments[context] = (*assignments_ptr)[i];
    }

    if (assignments.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "[TaskPlanner] Failed to compute assignments for task_id [%s]",
        id.c_str());

      return std::nullopt;
    }

    return assignments;
  }

  struct Result
  {
    std::optional<TaskAssignments> assignments;
    std::vector<std::string> errors;
  };

  AllocateTasks(
    rmf_task::ConstRequestPtr new_request_,
    Expectations expect_,
    rmf_task::TaskPlanner task_planner_,
    std::shared_ptr<Node> node_)
  : new_request(std::move(new_request_)),
    expect(std::move(expect_)),
    task_planner(std::move(task_planner_)),
    node(std::move(node_))
  {
    // Do nothing
  }

  rmf_task::ConstRequestPtr new_request;
  Expectations expect;
  rmf_task::TaskPlanner task_planner;
  std::shared_ptr<Node> node;
};

//==============================================================================
void FleetUpdateHandle::Implementation::bid_notice_cb(
  const BidNoticeMsg& bid_notice,
  rmf_task_ros2::bidding::AsyncBidder::Respond respond)
{
  // TODO(YV): Consider moving these checks into convert()
  const auto& task_id = bid_notice.task_id;
  if (task_managers.empty())
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet [%s] does not have any robots to accept task [%s]. Use "
      "FleetUpdateHadndle::add_robot(~) to add robots to this fleet. ",
      name.c_str(), task_id.c_str());
    return;
  }

  if (task_id.empty())
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Received BidNotice for a task with empty task_id. Request will be "
      "ignored.");
    return;
  }

  // TODO remove this block when we support task revival
  if (bid_notice_assignments.find(task_id) != bid_notice_assignments.end())
    return;

  if (!task_planner)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Fleet [%s] is not configured with parameters for task planning."
      "Use FleetUpdateHandle::set_task_planner_params(~) to set the "
      "parameters required.", name.c_str());

    return;
  }

  const auto request_msg = nlohmann::json::parse(bid_notice.request);
  static const auto request_validator =
    nlohmann::json_schema::json_validator(
    rmf_api_msgs::schemas::task_request);

  try
  {
    request_validator.validate(request_msg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Received a request with an invalid format. Error: %s\nRequest:\n%s",
      e.what(),
      request_msg.dump(2, ' ').c_str());

    return respond(
      {
        std::nullopt,
        {make_error_str(5, "Invalid request format", e.what())}
      });
  }

  // If a fleet_name was specified in the request, only proceed if the value matches
  // the name of this fleet.
  if (request_msg.contains("fleet_name") &&
    request_msg["fleet_name"].template get<std::string>() != name)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Ignoring BidNotice request as it is for fleet [%s].",
      request_msg["fleet_name"].template get<std::string>().c_str());
    return;
  }

  std::vector<std::string> errors = {};
  const auto new_request = convert(task_id, request_msg, errors);
  if (!new_request)
  {
    return respond(
      {
        std::nullopt,
        errors
      });
  }

  calculate_bid = std::make_shared<AllocateTasks>(
    new_request,
    aggregate_expectations(),
    *task_planner,
    node);

  auto receive_allocation = [w = weak_self, respond, task_id](
    AllocateTasks::Result result)
    {
      const auto self = w.lock();
      if (!self)
        return;

      auto allocation_result = result.assignments;
      if (!allocation_result.has_value())
        return respond({std::nullopt, std::move(result.errors)});

      const auto& assignments = allocation_result.value();

      const double cost = self->_pimpl->compute_cost(assignments);

      // Display computed assignments for debugging
      std::stringstream debug_stream;
      debug_stream << "Cost: " << cost << std::endl;
      for (const auto& [context, queue] : assignments)
      {
        debug_stream << "--Agent: " << context->requester_id() << std::endl;
        for (const auto& a : queue)
        {
          const auto& s = a.finish_state();
          const double request_seconds =
            a.request()->booking()->earliest_start_time().time_since_epoch().
            count()
            /1e9;
          const double start_seconds =
            a.deployment_time().time_since_epoch().count()/1e9;
          const rmf_traffic::Time finish_time = s.time().value();
          const double finish_seconds = finish_time.time_since_epoch().count()/
            1e9;
          debug_stream << "    <" << a.request()->booking()->id() << ": " <<
            request_seconds
                       << ", " << start_seconds
                       << ", "<< finish_seconds << ", " <<
            s.battery_soc().value()
                       << "%>" << std::endl;
        }
      }
      debug_stream << " ----------------------" << std::endl;

      RCLCPP_DEBUG(
        self->_pimpl->node->get_logger(),
        "%s",
        debug_stream.str().c_str());

      std::optional<std::string> robot_name;
      std::optional<rmf_traffic::Time> finish_time;
      for (const auto& [context, queue] : assignments)
      {
        for (const auto& assignment : queue)
        {
          if (assignment.request()->booking()->id() == task_id)
          {
            finish_time = assignment.finish_state().time().value();
            robot_name = context->name();
            break;
          }
        }
      }

      if (!robot_name.has_value() || !finish_time.has_value())
      {
        result.errors.push_back(
          make_error_str(
            13, "Internal bug",
            "Failed to find robot_name or finish_time after allocating task. "
            "Please report this bug to the RMF developers."));

        return respond({std::nullopt, std::move(result.errors)});
      }

      // Publish BidProposal
      respond(
        {
          rmf_task_ros2::bidding::Response::Proposal{
            self->_pimpl->name,
            *robot_name,
            self->_pimpl->current_assignment_cost,
            cost,
            *finish_time
          },
          std::move(result.errors)
        });

      RCLCPP_INFO(
        self->_pimpl->node->get_logger(),
        "Submitted BidProposal to accommodate task [%s] by robot [%s] with new cost [%f]",
        task_id.c_str(), robot_name->c_str(), cost);

      // Store assignments in internal map
      self->_pimpl->bid_notice_assignments.insert({task_id, assignments});
    };

  calculate_bid_subscription = rmf_rxcpp::make_job<AllocateTasks::Result>(
    calculate_bid)
    .observe_on(rxcpp::identity_same_worker(worker))
    .subscribe(receive_allocation);
}

//==============================================================================
void FleetUpdateHandle::Implementation::dispatch_command_cb(
  const DispatchCmdMsg::SharedPtr msg)
{
  const auto& task_id = msg->task_id;
  if (msg->fleet_name != name)
  {
    // This task is either being awarded or canceled for another fleet. Either
    // way, we will delete it from our record of bid notice assignments.
    bid_notice_assignments.erase(task_id);
    return;
  }

  DispatchAck dispatch_ack;
  dispatch_ack.success = false;
  dispatch_ack.dispatch_id = msg->dispatch_id;
  if (msg->type == DispatchCmdMsg::TYPE_AWARD)
  {
    last_bid_assignment = task_id;
    const auto task_it = bid_notice_assignments.find(task_id);
    if (task_it == bid_notice_assignments.end())
    {
      // We don't have a record of this bid_notice, so let's check if we already
      // received the task assignment.
      bool already_assigned = false;
      for (const auto& tm : task_managers)
      {
        const auto current = tm.second->current_task_id();
        if (current == task_id)
        {
          already_assigned = true;
          break;
        }

        for (const auto& p : tm.second->get_queue())
        {
          if (p.request()->booking()->id() == task_id)
          {
            already_assigned = true;
            break;
          }
        }

        if (already_assigned)
          break;
      }

      if (already_assigned)
      {
        dispatch_ack.success = true;
        dispatch_ack_pub->publish(dispatch_ack);
        return;
      }

      std::string error_str =
        "Received DispatchRequest award for task_id [" + task_id
        + "] before receiving BidNotice. This request will be ignored.";

      RCLCPP_ERROR(node->get_logger(), "%s", error_str.c_str());
      dispatch_ack.errors.push_back(
        make_error_str(14, "Invalid sequence", std::move(error_str)));
      dispatch_ack_pub->publish(dispatch_ack);
      return;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Bid for task_id [%s] awarded to fleet [%s]. Processing request...",
      task_id.c_str(),
      name.c_str());

    auto assignments = std::move(task_it->second);
    bid_notice_assignments.erase(task_it);

    // Here we make sure none of the tasks in the assignments has already begun
    // execution. If so, we replan assignments until a valid set is obtained
    // and only then update the task manager queues
    bool valid_assignments = is_valid_assignments(assignments);
    if (!valid_assignments)
    {
      rmf_task::ConstRequestPtr request;
      for (const auto& [_, queue] : assignments)
      {
        for (const auto& r : queue)
        {
          if (r.request()->booking()->id() == task_id)
          {
            request = r.request();
            break;
          }
        }

        if (request)
          break;
      }

      if (!request)
      {
        std::string error_str =
          "Could not find task_id [" + task_id + "] in the set of assignments "
          "associated with it. This is a critical bug and should be reported "
          "to the RMF developers";

        RCLCPP_ERROR(node->get_logger(), "%s", error_str.c_str());
        dispatch_ack.errors.push_back(
          make_error_str(13, "Internal bug", std::move(error_str)));
        dispatch_ack_pub->publish(dispatch_ack);
        return;
      }

      unassigned_requests.push_back(request);
      reassign_dispatched_tasks(
        // On success
        [dispatch_ack, dispatch_ack_pub = dispatch_ack_pub]()
        {
          auto msg = dispatch_ack;
          msg.success = true;
          dispatch_ack_pub->publish(dispatch_ack);
        },
        // On failure
        [dispatch_ack, w = weak_self, request](
          std::vector<std::string> errors)
        {
          const auto self = w.lock();
          if (!self)
            return;

          auto r_it = std::remove(
            self->_pimpl->unassigned_requests.begin(),
            self->_pimpl->unassigned_requests.end(),
            request);

          self->_pimpl->unassigned_requests.erase(
            r_it, self->_pimpl->unassigned_requests.end());

          std::string error_str =
          "Unable to replan assignments when accommodating task_id ["
          + request->booking()->id() + "]. Reasons:";
          if (errors.empty())
          {
            error_str += " No reason given by planner.";
          }
          else
          {
            for (const auto& e : errors)
            {
              error_str += "\n -- " + e;
            }
          }

          RCLCPP_ERROR(
            self->_pimpl->node->get_logger(),
            "%s",
            error_str.c_str());
          auto msg = dispatch_ack;
          msg.success = false;
          msg.errors.push_back(
            make_error_str(9, "Not feasible", std::move(error_str)));
          self->_pimpl->dispatch_ack_pub->publish(msg);
        });
    }
    else
    {
      for (const auto& [context, queue] : assignments)
      {
        context->task_manager()->set_queue(queue);
      }

      // Any unassigned requests would have been collected by the aggregator and
      // given an assignment while calculating this bid. As long as
      // is_valid_assignments was able to pass, then all tasks have been
      // assigned to properly commissioned robots.
      unassigned_requests.clear();

      current_assignment_cost = compute_cost(assignments);
      dispatch_ack.success = true;
      dispatch_ack_pub->publish(dispatch_ack);

      RCLCPP_INFO(
        node->get_logger(),
        "TaskAssignments updated for robots in fleet [%s] to accommodate task_id [%s]",
        name.c_str(), task_id.c_str());
    }
  }
  else if (msg->type == DispatchCmdMsg::TYPE_REMOVE)
  {
    const auto bid_it = bid_notice_assignments.find(task_id);
    if (bid_it != bid_notice_assignments.end())
    {
      // The task was still in the bid notice assignments, so it was never
      // actually assigned to a robot. We will just delete it from this map
      // and we're done.
      bid_notice_assignments.erase(bid_it);
    }
    else
    {
      bool task_was_found = false;
      // Make sure the task isn't running in any of the task managers
      for (const auto& [_, tm] : task_managers)
      {
        task_was_found = tm->cancel_task_if_present(task_id);
        if (task_was_found)
          break;
      }

      if (task_was_found)
      {
        reassign_dispatched_tasks(
          // On success
          [task_id, name = name, node = node]()
          {
            RCLCPP_INFO(
              node->get_logger(),
              "Task with task_id [%s] has successfully been cancelled. "
              "TaskAssignments updated for robots in fleet [%s].",
              task_id.c_str(), name.c_str());
          },
          // On failure
          [task_id, name = name, node = node](std::vector<std::string> errors)
          {
            std::stringstream ss;
            ss << "Unabled to replan assignments when cancelling task ["
               << task_id << "] for fleet [" << name << "]. ";
            if (errors.empty())
            {
              ss << "No planner error messages were provided.";
            }
            else
            {
              ss << "The following planner errors occurred:";
              for (const auto& e : errors)
              {
                const auto err = nlohmann::json::parse(e);
                ss << "\n -- " << err["detail"].get<std::string>();
              }
            }
            ss << "\n";

            RCLCPP_WARN(node->get_logger(), "%s", ss.str().c_str());
          });
      }
    }

    dispatch_ack.success = true;
    dispatch_ack_pub->publish(dispatch_ack);
  }
  else
  {
    std::string error_str =
      "Received unknown dispatch request type: " + std::to_string(msg->type);
    RCLCPP_ERROR(node->get_logger(), "%s", error_str.c_str());
    dispatch_ack.errors.push_back(
      make_error_str(4, "Unsupported type", std::move(error_str)));
    dispatch_ack_pub->publish(dispatch_ack);
  }
}

//==============================================================================
double FleetUpdateHandle::Implementation::compute_cost(
  const TaskAssignments& assignments) const
{
  rmf_task::TaskPlanner::Assignments raw_assignments;
  raw_assignments.reserve(assignments.size());
  for (const auto [_, queue] : assignments)
  {
    raw_assignments.push_back(queue);
  }

  return task_planner->compute_cost(raw_assignments);
}

//==============================================================================
auto FleetUpdateHandle::Implementation::is_valid_assignments(
  TaskAssignments& assignments,
  std::string* report_error) const -> bool
{
  std::unordered_set<std::string> executed_tasks;
  for (const auto& [context, mgr] : task_managers)
  {
    const auto& tasks = mgr->get_executed_tasks();
    executed_tasks.insert(tasks.begin(), tasks.end());
  }

  for (const auto& [context, queue] : assignments)
  {
    for (const auto& a : queue)
    {
      if (executed_tasks.find(a.request()->booking()->id()) !=
        executed_tasks.end())
      {
        if (report_error)
        {
          *report_error = "task [" + a.request()->booking()->id()
            + "] already began executing";
        }
        return false;
      }
    }

    if (!queue.empty())
    {
      if (!context->commission().is_accepting_dispatched_tasks())
      {
        if (report_error)
        {
          *report_error = "robot [" + context->name()
            + "] has been decommissioned";
        }
        // If any tasks were assigned to this robot after it has been
        // decommissioned, then those were invalid assignments.
        return false;
      }

      if (task_managers.find(context) == task_managers.end())
      {
        if (report_error)
        {
          *report_error = "robot [" + context->name()
            + "] has been removed from the fleet";
        }
        // This robot is no longer part of the fleet, so we cannot assign
        // tasks to it.
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
void FleetUpdateHandle::Implementation::reassign_dispatched_tasks(
  std::function<void()> on_success,
  std::function<void(std::vector<std::string>)> on_failure)
{
  auto expectations = aggregate_expectations();
  if (expectations.states.empty() && !expectations.pending_requests.empty())
  {
    // If there are no robots that can perform any tasks but there are pending
    // requests, then we should immediately assume failure.
    worker.schedule(
      [on_failure](const auto&)
      {
        on_failure({"no more robots available"});
      });
    return;
  }

  auto on_plan_received = [
    on_success,
    on_failure,
    w = weak_self,
    initial_last_bid_assignment = last_bid_assignment
    ](TaskAssignments assignments)
    {
      const auto self = w.lock();
      if (!self)
        return;

      const bool new_task_came_in =
        initial_last_bid_assignment != self->_pimpl->last_bid_assignment;

      std::string error;
      const bool valid_assignments =
        !new_task_came_in
        && self->_pimpl->is_valid_assignments(assignments, &error);

      if (!valid_assignments)
      {
        // We need to replan because some important aspect of the planning
        // problem has changed since we originally tried to reassign.
        if (new_task_came_in)
        {
          RCLCPP_WARN(
            self->_pimpl->node->get_logger(),
            "Redoing task reassignment for fleet [%s] because a new task was "
            "dispatched while the reassignment was being calculated.",
            self->_pimpl->name.c_str());
        }
        else
        {
          RCLCPP_WARN(
            self->_pimpl->node->get_logger(),
            "Redoing task reassignment for fleet [%s] because %s.",
            self->_pimpl->name.c_str(),
            error.c_str());
        }

        self->_pimpl->reassign_dispatched_tasks(on_success, on_failure);
        return;
      }

      for (const auto& [context, queue] : assignments)
      {
        context->task_manager()->set_queue(queue);
      }

      // All requests should be assigned now, no matter which robot they were
      // originally assigned to, so we can clear this buffer.
      //
      // As long as no new tasks were dispatched to this fleet (verified by the
      // new_task_came_in check), any tasks that might have been previously
      // unassigned will be accounted for in the current set of assignments.
      self->_pimpl->unassigned_requests.clear();

      self->_pimpl->current_assignment_cost =
        self->_pimpl->compute_cost(assignments);

      on_success();
    };

  reassignment_worker.schedule(
    [
      on_plan_received,
      on_failure,
      expectations = std::move(expectations),
      task_planner = *task_planner,
      node = node,
      worker = worker
    ](const auto&)
    {
      std::vector<std::string> errors;
      const auto replan_results = AllocateTasks(
        nullptr, expectations, task_planner, node)
      .run(errors);

      if (replan_results.has_value())
      {
        worker.schedule(
          [
            assignments = std::move(*replan_results),
            on_plan_received
          ](const auto&)
          {
            on_plan_received(assignments);
          });
      }
      else
      {
        worker.schedule(
          [errors = std::move(errors), on_failure](const auto&)
          {
            on_failure(errors);
          });
      }
    });
}

//==============================================================================
std::optional<std::size_t> FleetUpdateHandle::Implementation::
get_nearest_charger(
  const rmf_traffic::agv::Planner::Start& start)
{
  if (charging_waypoints.empty())
    return std::nullopt;

  double min_cost = std::numeric_limits<double>::max();
  std::optional<std::size_t> nearest_charger = std::nullopt;
  for (const auto& wp : charging_waypoints)
  {
    const rmf_traffic::agv::Planner::Goal goal{wp};
    const auto& planner_result = (*planner)->setup(start, goal);
    const auto ideal_cost = planner_result.ideal_cost();
    if (ideal_cost.has_value() && ideal_cost.value() < min_cost)
    {
      min_cost = ideal_cost.value();
      nearest_charger = wp;
    }
  }

  return nearest_charger;
}

namespace {
//==============================================================================
std::optional<rmf_fleet_msgs::msg::Location> convert_location(
  const agv::RobotContext& context)
{
  if (context.location().empty())
  {
    const auto& lost = context.lost();
    if (lost.has_value() && lost->location.has_value())
    {
      const auto& l = *lost->location;
      return rmf_fleet_msgs::build<rmf_fleet_msgs::msg::Location>()
        .t(rmf_traffic_ros2::convert(l.time))
        .x(l.position[0])
        .y(l.position[1])
        .yaw(l.position[2])
        .obey_approach_speed_limit(false)
        .approach_speed_limit(0.0)
        .level_name(l.map)
        .index(0);
    }

    // TODO(MXG): We should emit some kind of critical error if there is no
    // location and also no lost information, because that means an issue ticket
    // has not been created.
    return std::nullopt;
  }

  const auto& graph = context.planner()->get_configuration().graph();
  const auto& l = context.location().front();
  const auto& wp = graph.get_waypoint(l.waypoint());
  const Eigen::Vector2d p = l.location().value_or(wp.get_location());

  return rmf_fleet_msgs::build<rmf_fleet_msgs::msg::Location>()
    .t(rmf_traffic_ros2::convert(l.time()))
    .x(p.x())
    .y(p.y())
    .yaw(l.orientation())
    .obey_approach_speed_limit(false)
    .approach_speed_limit(0.0)
    .level_name(wp.get_map_name())
    // NOTE(MXG): This field is only used by the fleet drivers. For now, we
    // will just fill it with a zero.
    .index(0);
}

//==============================================================================
std::optional<rmf_fleet_msgs::msg::RobotState> convert_state(
  const TaskManager& mgr)
{
  const RobotContext& context = *mgr.context();
  const auto location = convert_location(context);
  if (!location.has_value())
    return std::nullopt;

  const auto mode = mgr.robot_mode();

  return rmf_fleet_msgs::build<rmf_fleet_msgs::msg::RobotState>()
    .name(context.name())
    .model(context.description().owner())
    .task_id(mgr.current_task_id().value_or(""))
    // TODO(MXG): We could keep track of the seq value and increment it once
    // with each publication. This is not currently an important feature
    // outside of the fleet driver, so for now we just set it to zero.
    .seq(0)
    .mode(std::move(mode))
    // We multiply by 100 to convert from the [0.0, 1.0] range to percentage
    .battery_percent(context.current_battery_soc()*100.0)
    .location(*location)
    // NOTE(MXG): The path field is only used by the fleet drivers. For now,
    // we will just fill it with a zero. We could consider filling it in based
    // on the robot's plan, but that seems redundant with the traffic schedule
    // information.
    .path({});
}
} // anonymous namespace

//==============================================================================
void FleetUpdateHandle::Implementation::publish_fleet_state_topic() const
{
  std::vector<rmf_fleet_msgs::msg::RobotState> robot_states;
  for (const auto& [context, mgr] : task_managers)
  {
    auto state = convert_state(*mgr);
    if (!state.has_value())
      continue;

    robot_states.emplace_back(std::move(*state));
  }

  auto fleet_state = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::FleetState>()
    .name(name)
    .robots(std::move(robot_states));

  fleet_state_pub->publish(std::move(fleet_state));
}

//==============================================================================
void FleetUpdateHandle::Implementation::update_fleet() const
{
  update_fleet_state();
  update_fleet_logs();
}

//==============================================================================
void FleetUpdateHandle::Implementation::update_fleet_state() const
{
  // Publish to API server
  if (broadcast_client)
  {
    nlohmann::json fleet_state_update_msg;
    fleet_state_update_msg["type"] = "fleet_state_update";
    auto& fleet_state_msg = fleet_state_update_msg["data"];
    fleet_state_msg["name"] = name;
    auto& robots = fleet_state_msg["robots"];
    robots = std::unordered_map<std::string, nlohmann::json>();
    for (const auto& [context, mgr] : task_managers)
    {
      const auto& name = context->name();
      nlohmann::json& json = robots[name];
      json["name"] = name;
      json["status"] = mgr->robot_status();
      json["task_id"] = mgr->current_task_id().value_or("");
      json["unix_millis_time"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        context->now().time_since_epoch()).count();
      json["battery"] = context->current_battery_soc();

      const auto location_msg = convert_location(*context);
      if (location_msg.has_value())
      {
        nlohmann::json& location = json["location"];
        location["map"] = location_msg->level_name;
        location["x"] = location_msg->x;
        location["y"] = location_msg->y;
        location["yaw"] = location_msg->yaw;
      }

      std::lock_guard<std::mutex> lock(context->reporting().mutex());
      const auto& issues = context->reporting().open_issues();
      auto& issues_msg = json["issues"];
      issues_msg = std::vector<nlohmann::json>();
      for (const auto& issue : issues)
      {
        nlohmann::json issue_msg;
        issue_msg["category"] = issue->category;
        issue_msg["detail"] = issue->detail;
        issues_msg.push_back(std::move(issue_msg));
      }

      const auto& commission = context->commission();
      nlohmann::json commission_json;
      commission_json["dispatch_tasks"] =
        commission.is_accepting_dispatched_tasks();
      commission_json["direct_tasks"] = commission.is_accepting_direct_tasks();
      commission_json["idle_behavior"] =
        commission.is_performing_idle_behavior();

      json["commission"] = commission_json;

      nlohmann::json mutex_groups_json;
      std::vector<std::string> locked_mutex_groups;
      for (const auto& g : context->locked_mutex_groups())
      {
        locked_mutex_groups.push_back(g.first);
      }
      mutex_groups_json["locked"] = std::move(locked_mutex_groups);

      std::vector<std::string> requesting_mutex_groups;
      for (const auto& g : context->requesting_mutex_groups())
      {
        requesting_mutex_groups.push_back(g.first);
      }
      mutex_groups_json["requesting"] = std::move(requesting_mutex_groups);

      json["mutex_groups"] = std::move(mutex_groups_json);
    }

    try
    {
      static const auto validator =
        make_validator(rmf_api_msgs::schemas::fleet_state_update);

      validator.validate(fleet_state_update_msg);

      std::unique_lock<std::mutex> lock(*update_callback_mutex);
      if (update_callback)
        update_callback(fleet_state_update_msg);
      broadcast_client->publish(fleet_state_update_msg);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Malformed outgoing fleet state json message: %s\nMessage:\n%s",
        e.what(),
        fleet_state_update_msg.dump(2).c_str());
    }
  }
}

//==============================================================================
void FleetUpdateHandle::Implementation::update_fleet_logs() const
{
  if (broadcast_client)
  {
    nlohmann::json fleet_log_update_msg;
    fleet_log_update_msg["type"] = "fleet_log_update";
    auto& fleet_log_msg = fleet_log_update_msg["data"];
    fleet_log_msg["name"] = name;
    // TODO(MXG): fleet_log_msg["log"]
    auto& robots_msg = fleet_log_msg["robots"];
    robots_msg = std::unordered_map<std::string, nlohmann::json>();
    for (const auto& [context, _] : task_managers)
    {
      auto robot_log_msg_array = std::vector<nlohmann::json>();

      std::lock_guard<std::mutex> lock(context->reporting().mutex());
      const auto& log = context->reporting().log();
      for (const auto& entry : log_reader.read(log.view()))
        robot_log_msg_array.push_back(log_to_json(entry));

      if (!robot_log_msg_array.empty())
        robots_msg[context->name()] = std::move(robot_log_msg_array);
    }

    if (robots_msg.empty())
    {
      // No new logs to report
      return;
    }

    try
    {
      static const auto validator =
        make_validator(rmf_api_msgs::schemas::fleet_log_update);

      validator.validate(fleet_log_update_msg);

      std::unique_lock<std::mutex> lock(*update_callback_mutex);
      if (update_callback)
        update_callback(fleet_log_update_msg);
      broadcast_client->publish(fleet_log_update_msg);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Malformed outgoing fleet log json message: %s\nMessage:\n%s",
        e.what(),
        fleet_log_update_msg.dump(2).c_str());
    }
  }
}

//==============================================================================
void FleetUpdateHandle::Implementation::handle_emergency(
  const bool is_emergency)
{
  if (is_emergency == emergency_active)
    return;

  emergency_active = is_emergency;
  if (is_emergency)
  {
    update_emergency_planner();
  }

  for (const auto& [context, _] : task_managers)
  {
    context->_set_emergency(is_emergency);
  }
  emergency_publisher.get_subscriber().on_next(is_emergency);
}

//==============================================================================
namespace {
class EmergencyLaneCloser : public rmf_traffic::agv::Graph::Lane::Executor
{
public:
  void execute(const DoorOpen&) override {}
  void execute(const DoorClose&) override {}
  void execute(const LiftSessionEnd&) override {}
  void execute(const LiftMove&) override {}
  void execute(const Wait&) override {}
  void execute(const Dock& dock) override {}
  void execute(const LiftSessionBegin& info) override
  {
    lift = info.lift_name();
    enter = true;
  }
  void execute(const LiftDoorOpen& info) override
  {
    lift = info.lift_name();
    exit = true;
  }

  std::string lift;
  bool enter = false;
  bool exit = false;
};
} // anonymous namespace

//==============================================================================
std::vector<std::size_t> find_emergency_lift_closures(
  const rmf_traffic::agv::Graph& graph,
  const std::unordered_map<std::string, std::string>& emergency_level_for_lift)
{
  std::vector<std::size_t> closures;
  for (std::size_t i = 0; i < graph.num_lanes(); ++i)
  {
    EmergencyLaneCloser executor;
    const auto& lane = graph.get_lane(i);
    if (const auto* event = lane.entry().event())
      event->execute(executor);

    if (const auto* event = lane.exit().event())
      event->execute(executor);

    const auto wp0 = lane.entry().waypoint_index();
    const auto wp1 = lane.exit().waypoint_index();
    if (executor.enter)
    {
      if (emergency_level_for_lift.count(executor.lift) > 0)
      {
        closures.push_back(i);
      }
    }
    else if (executor.exit)
    {
      const auto l_it = emergency_level_for_lift.find(executor.lift);
      if (l_it != emergency_level_for_lift.end())
      {
        const auto& map = graph.get_waypoint(
          lane.exit().waypoint_index()).get_map_name();
        if (l_it->second != map)
        {
          closures.push_back(i);
        }
      }
    }
  }

  return closures;
}

//==============================================================================
void FleetUpdateHandle::Implementation::update_emergency_planner()
{
  const auto& config = (*planner)->get_configuration();
  const auto lift_closures = find_emergency_lift_closures(
    config.graph(),
    emergency_level_for_lift);
  const auto& normal_closures = config.lane_closures();
  auto emergency_closures = normal_closures;
  for (const std::size_t lane : lift_closures)
  {
    emergency_closures.close(lane);
  }

  auto emergency_config = config;
  emergency_config.lane_closures(std::move(emergency_closures));

  *emergency_planner = std::make_shared<const rmf_traffic::agv::Planner>(
    emergency_config, rmf_traffic::agv::Planner::Options(nullptr));
}

//==============================================================================
void FleetUpdateHandle::Implementation::update_charging_assignments(
  const ChargingAssignments& charging)
{
  if (charging.fleet_name != name)
    return;

  RCLCPP_INFO(
    node->get_logger(),
    "Fleet [%s] received new charging assignments",
    name.c_str());

  for (const ChargingAssignment& assignment : charging.assignments)
  {
    bool found_robot = false;
    for (const auto& [context, _] : task_managers)
    {
      if (context->name() != assignment.robot_name)
        continue;

      const rmf_traffic::agv::Graph& graph = context->navigation_graph();
      const auto wp = graph.find_waypoint(assignment.waypoint_name);
      if (!wp)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Cannot change charging waypoint for [%s] to [%s] because it does "
          "not exist in the graph",
          context->requester_id().c_str(),
          assignment.waypoint_name.c_str());
      }
      const bool wait_for_charger = assignment.mode == assignment.MODE_WAIT;
      context->_set_charging(wp->index(), wait_for_charger);
    }

    if (!found_robot)
    {
      unregistered_charging_assignments[assignment.robot_name] = assignment;
    }
  }
}

//==============================================================================
nlohmann::json_schema::json_validator
FleetUpdateHandle::Implementation::make_validator(
  const nlohmann::json& schema) const
{
  const auto loader =
    [n = node, s = schema_dictionary](const nlohmann::json_uri& id,
      nlohmann::json& value)
    {
      const auto it = s.find(id.url());
      if (it == s.end())
      {
        RCLCPP_ERROR(
          n->get_logger(),
          "url: %s not found in schema dictionary", id.url().c_str());
        return;
      }

      value = it->second;
    };

  return nlohmann::json_schema::json_validator(schema, loader);
}

namespace {
//==============================================================================
PlaceDeserializer make_place_deserializer(
  std::shared_ptr<const std::shared_ptr<const rmf_traffic::agv::Planner>>
  planner)
{
  return [planner = std::move(planner)](const nlohmann::json& msg)
    -> agv::DeserializedPlace
    {
      std::optional<rmf_traffic::agv::Plan::Goal> place;
      const auto& graph = (*planner)->get_configuration().graph();
      if (msg.is_number() || (msg.is_object() && msg["waypoint"].is_number()))
      {
        const auto wp_index = msg.is_number() ?
          msg.get<std::size_t>() : msg["waypoint"].get<std::size_t>();

        if (graph.num_waypoints() <= wp_index)
        {
          return {
            std::nullopt,
            {"waypoint index value for Place ["
              + std::to_string(wp_index) + "] exceeds the limits for the nav "
              "graph size [" + std::to_string(graph.num_waypoints()) + "]"}
          };
        }

        place = rmf_traffic::agv::Plan::Goal(wp_index);
      }
      else if (msg.is_string() ||
        (msg.is_object() && msg["waypoint"].is_string()))
      {
        const auto& wp_name = msg.is_string() ?
          msg.get<std::string>() : msg["waypoint"].get<std::string>();

        const auto* wp = graph.find_waypoint(wp_name);
        if (!wp)
        {
          return {
            std::nullopt,
            {"waypoint name for Place [" + wp_name + "] cannot be "
              "found in the navigation graph"}
          };
        }

        place = rmf_traffic::agv::Plan::Goal(wp->index());
      }
      else
      {
        return {
          std::nullopt,
          {"invalid data type provided for Place: expected a number "
            "or a string or an object but got " + std::string(msg.type_name())
            + " type instead"}
        };
      }

      if (msg.is_object())
      {
        const auto& ori_it = msg.find("orientation");
        if (ori_it != msg.end())
          place->orientation(ori_it->get<double>());
      }

      return {place, {}};
    };
}
} // anonymous namespace

//==============================================================================
void FleetUpdateHandle::Implementation::add_standard_tasks()
{
  activation.task = std::make_shared<rmf_task::Activator>();
  activation.phase = std::make_shared<rmf_task_sequence::Phase::Activator>();
  activation.event = std::make_shared<rmf_task_sequence::Event::Initializer>();

  rmf_task_sequence::Task::add(
    *activation.task, activation.phase, node->clock());

  rmf_task_sequence::phases::SimplePhase::add(
    *activation.phase, activation.event);

  events::GoToPlace::add(*activation.event);
  events::PerformAction::add(*activation.event);
  deserialization.place = make_place_deserializer(planner);
  deserialization.add_schema(schemas::place);

  events::ResponsiveWait::add(*activation.event);

  tasks::add_delivery(
    deserialization,
    activation,
    node->clock());

  tasks::add_patrol(
    deserialization,
    activation,
    node->clock());

  tasks::add_clean(
    dock_param_map,
    (*planner)->get_configuration().vehicle_traits(),
    deserialization,
    activation,
    node->clock());

  tasks::add_charge_battery(
    *activation.task,
    activation.phase,
    *activation.event,
    node->clock());

  tasks::add_compose(
    deserialization,
    activation,
    node->clock());
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::add_performable_action(
  const std::string& category,
  ConsiderRequest consider)
{
  if (category.empty())
  {
    RCLCPP_ERROR(
      _pimpl->node->get_logger(),
      "FleetUpdateHandle::add_performable_action(~) called with empty category"
    );
    return *this;
  }

  _pimpl->deserialization.consider_actions->insert_or_assign(
    category, consider);

  return *this;
}

//==============================================================================
auto FleetUpdateHandle::Implementation::aggregate_expectations() const
-> Expectations
{
  Expectations expect;
  for (const auto& t : task_managers)
  {
    // Ignore any robots that are not currently commissioned.
    if (!t.first->commission().is_accepting_dispatched_tasks())
      continue;

    expect.states.push_back(
      ExpectedState {
        t.first,
        t.second->expected_finish_state()
      });
    const auto requests = t.second->dispatched_requests();
    expect.pending_requests.insert(
      expect.pending_requests.end(), requests.begin(), requests.end());
  }

  expect.pending_requests.insert(
    expect.pending_requests.end(),
    unassigned_requests.begin(),
    unassigned_requests.end());

  return expect;
}

//==============================================================================
const std::string& FleetUpdateHandle::fleet_name() const
{
  return _pimpl->name;
}

//==============================================================================
void FleetUpdateHandle::add_robot(
  std::shared_ptr<RobotCommandHandle> command,
  const std::string& name,
  const rmf_traffic::Profile& profile,
  rmf_traffic::agv::Plan::StartSet start,
  std::function<void(std::shared_ptr<RobotUpdateHandle>)> handle_cb)
{
  if (start.empty())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[FleetUpdateHandle::add_robot] StartSet is empty. Adding a robot to a "
      "fleet requires at least one rmf_traffic::agv::Plan::Start to be "
      "specified.");
    // *INDENT-ON*
  }

  rmf_traffic::schedule::ParticipantDescription description(
    name,
    _pimpl->name,
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    profile);

  _pimpl->writer->async_make_participant(
    std::move(description),
    [worker = _pimpl->worker,
    command = std::move(command),
    start = std::move(start),
    handle_cb = std::move(handle_cb),
    fleet_wptr = weak_from_this()](
      rmf_traffic::schedule::Participant participant)
    {
      auto fleet = fleet_wptr.lock();
      if (!fleet)
        return;

      const auto charger_wp = fleet->_pimpl->get_nearest_charger(start[0]);

      if (!charger_wp.has_value())
      {
        // *INDENT-OFF*
        throw std::runtime_error(
          "[FleetUpdateHandle::add_robot] Unable to find nearest charging "
          "waypoint. Adding a robot to a fleet requires at least one charging"
          "waypoint to be present in its navigation graph.");
        // *INDENT-ON*
      }

      rmf_task::State state;
      state.load_basic(start[0], charger_wp.value(), 1.0);

      auto context = RobotContext::make(
        std::move(command),
        std::move(start),
        std::move(participant),
        fleet->_pimpl->mirror,
        fleet->_pimpl->planner,
        fleet->_pimpl->emergency_planner,
        fleet->_pimpl->activation.task,
        fleet->_pimpl->task_parameters,
        fleet->_pimpl->node,
        fleet->_pimpl->worker,
        fleet->_pimpl->default_maximum_delay,
        state,
        fleet->_pimpl->task_planner);

      // We schedule the following operations on the worker to make sure we do not
      // have a multiple read/write race condition on the FleetUpdateHandle.
      worker.schedule(
        [fleet_wptr = std::weak_ptr<FleetUpdateHandle>(fleet),
        node_wptr = std::weak_ptr<Node>(fleet->_pimpl->node),
        context = std::move(context),
        handle_cb = std::move(handle_cb)](const auto&)
        {
          auto fleet = fleet_wptr.lock();
          if (!fleet)
            return;

          auto node = node_wptr.lock();
          if (!node)
            return;

          if (fleet->_pimpl->emergency_active)
          {
            context->_set_emergency(true);
          }

          // TODO(MXG): We need to perform this test because we do not currently
          // support the distributed negotiation in unit test environments. We
          // should create an abstract NegotiationRoom interface in rmf_traffic and
          // use that instead.
          if (fleet->_pimpl->negotiation)
          {
            using namespace std::chrono_literals;
            auto last_interrupt_time =
            std::make_shared<std::optional<rmf_traffic::Time>>(std::nullopt);
            auto negotiation_license =
            fleet->_pimpl->negotiation
            ->register_negotiator(
              context->itinerary().id(),
              std::make_unique<LiaisonNegotiator>(context),
              [w = std::weak_ptr<RobotContext>(context), last_interrupt_time]()
              {
                if (const auto c = w.lock())
                {
                  const auto& graph = c->navigation_graph();
                  std::stringstream ss;
                  ss << "Failed negotiation for [" << c->requester_id()
                     << "] with these starts:"
                     << print_starts(c->location(), graph);
                  std::cout << ss.str() << std::endl;

                  auto& last_time = *last_interrupt_time;
                  const auto now = std::chrono::steady_clock::now();
                  if (last_time.has_value())
                  {
                    if (now < *last_time + 10s)
                      return;
                  }

                  last_time = now;
                  if (!c->is_stubborn())
                  {
                    RCLCPP_INFO(
                      c->node()->get_logger(),
                      "Requesting replan for [%s] because it failed to negotiate",
                      c->requester_id().c_str());
                    c->request_replan();
                  }
                }
              });
            context->_set_negotiation_license(std::move(negotiation_license));
          }

          RCLCPP_INFO(
            node->get_logger(),
            "Added a robot named [%s] with participant ID [%ld]",
            context->name().c_str(),
            context->itinerary().id());

          std::optional<std::weak_ptr<rmf_websocket::BroadcastClient>>
          broadcast_client = std::nullopt;

          if (fleet->_pimpl->broadcast_client)
            broadcast_client = fleet->_pimpl->broadcast_client;

          const auto mgr = TaskManager::make(
            context,
            broadcast_client,
            std::weak_ptr<FleetUpdateHandle>(fleet));

          fleet->_pimpl->task_managers.insert({context, mgr});

          const auto c_it = fleet->_pimpl
          ->unregistered_charging_assignments.find(context->name());
          if (c_it != fleet->_pimpl->unregistered_charging_assignments.end())
          {
            const auto& charging = c_it->second;
            const auto& graph = context->navigation_graph();
            const auto* wp = graph.find_waypoint(charging.waypoint_name);
            if (!wp)
            {
              RCLCPP_ERROR(
                fleet->_pimpl->node->get_logger(),
                "Cannot find a waypoing named [%s] for robot [%s], which was "
                "requested as its charging point",
                charging.waypoint_name.c_str(),
                context->requester_id().c_str());
            }
            else
            {
              context->_set_charging(
                wp->index(),
                charging.mode == charging.MODE_WAIT);
            }
            fleet->_pimpl->unregistered_charging_assignments.erase(c_it);
          }

          mgr->set_idle_task(fleet->_pimpl->idle_task);
          mgr->configure_retreat_to_charger(fleet->retreat_to_charger_interval());

          // -- Calling the handle_cb should always happen last --
          if (handle_cb)
          {
            handle_cb(RobotUpdateHandle::Implementation::make(std::move(context)));
          }
          else
          {
            RCLCPP_WARN(
              node->get_logger(),
              "FleetUpdateHandle::add_robot(~) was not provided a callback to "
              "receive the RobotUpdateHandle of the new robot. This means you will "
              "not be able to update the state of the new robot. This is likely to "
              "be a fleet adapter development error.");
          }
        });
    });
}

//==============================================================================
class FleetUpdateHandle::Confirmation::Implementation
{
public:
  bool is_accepted = false;
  std::vector<std::string> errors;
};

//==============================================================================
FleetUpdateHandle::Confirmation::Confirmation()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto FleetUpdateHandle::Confirmation::accept() -> Confirmation&
{
  _pimpl->is_accepted = true;
  return *this;
}

//==============================================================================
bool FleetUpdateHandle::Confirmation::is_accepted() const
{
  return _pimpl->is_accepted;
}

//==============================================================================
auto FleetUpdateHandle::Confirmation::errors(
  std::vector<std::string> error_messages) -> Confirmation&
{
  _pimpl->errors = std::move(error_messages);
  return *this;
}

//==============================================================================
auto FleetUpdateHandle::Confirmation::add_errors(
  std::vector<std::string> error_messages) -> Confirmation&
{
  _pimpl->errors.insert(
    _pimpl->errors.end(),
    std::make_move_iterator(error_messages.begin()),
    std::make_move_iterator(error_messages.end()));

  return *this;
}

//==============================================================================
const std::vector<std::string>& FleetUpdateHandle::Confirmation::errors() const
{
  return _pimpl->errors;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::consider_delivery_requests(
  ConsiderRequest consider_pickup,
  ConsiderRequest consider_dropoff)
{
  *_pimpl->deserialization.consider_pickup = std::move(consider_pickup);
  *_pimpl->deserialization.consider_dropoff = std::move(consider_dropoff);
  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::consider_cleaning_requests(
  ConsiderRequest consider)
{
  *_pimpl->deserialization.consider_clean = std::move(consider);
  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::consider_patrol_requests(
  ConsiderRequest consider)
{
  *_pimpl->deserialization.consider_patrol = std::move(consider);
  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::consider_composed_requests(
  ConsiderRequest consider)
{
  *_pimpl->deserialization.consider_composed = std::move(consider);
  return *this;
}

//==============================================================================
void FleetUpdateHandle::close_lanes(std::vector<std::size_t> lane_indices)
{
  _pimpl->worker.schedule(
    [w = weak_from_this(), lane_indices = std::move(lane_indices)](const auto&)
    {
      const auto self = w.lock();
      if (!self)
        return;

      bool any_changes = false;
      for (const auto& lane : lane_indices)
      {
        if (self->_pimpl->closed_lanes.insert(lane).second)
        {
          any_changes = true;
        }
      }

      if (!any_changes)
      {
        // No changes are needed to the planner
        return;
      }

      auto new_config = (*self->_pimpl->planner)->get_configuration();
      auto& new_lane_closures = new_config.lane_closures();
      for (const auto& lane : lane_indices)
      {
        new_lane_closures.close(lane);
      }

      *self->_pimpl->planner =
      std::make_shared<const rmf_traffic::agv::Planner>(
        new_config, rmf_traffic::agv::Planner::Options(nullptr));

      if (self->_pimpl->emergency_active)
      {
        self->_pimpl->update_emergency_planner();
      }

      self->_pimpl->task_parameters->planner(*self->_pimpl->planner);
      self->_pimpl->publish_lane_states();

      RobotContext::GraphChange changes{lane_indices};
      for (auto& [ctx, _] : self->_pimpl->task_managers)
      {
        ctx->notify_graph_change(changes);
      }
    });
}

//==============================================================================
void FleetUpdateHandle::open_lanes(std::vector<std::size_t> lane_indices)
{
  _pimpl->worker.schedule(
    [w = weak_from_this(), lane_indices = std::move(lane_indices)](const auto&)
    {
      const auto self = w.lock();
      if (!self)
        return;

      // Note that this approach to tracking whether to open a lane will make
      // it impossible for an external user to open a lane which has been closed
      // by the emergency_level_for_lift behavior. For now this is intentional,
      // but in future implementations we may want to allow users to decide if
      // that is desirable behavior.
      bool any_changes = false;
      for (const auto& lane : lane_indices)
      {
        if (self->_pimpl->closed_lanes.erase(lane) > 0)
        {
          any_changes = true;
        }
      }

      if (!any_changes)
      {
        // No changes are needed to the planner
        return;
      }

      auto new_config = (*self->_pimpl->planner)->get_configuration();
      auto& new_lane_closures = new_config.lane_closures();
      for (const auto& lane : lane_indices)
      {
        new_lane_closures.open(lane);
      }

      *self->_pimpl->planner =
      std::make_shared<const rmf_traffic::agv::Planner>(
        new_config, rmf_traffic::agv::Planner::Options(nullptr));

      if (self->_pimpl->emergency_active)
      {
        self->_pimpl->update_emergency_planner();
      }

      self->_pimpl->task_parameters->planner(*self->_pimpl->planner);
      self->_pimpl->publish_lane_states();
    });
}

//==============================================================================
void FleetUpdateHandle::set_lift_emergency_level(
  std::string lift_name,
  std::string emergency_level_name)
{
  _pimpl->emergency_level_for_lift[std::move(lift_name)] =
    std::move(emergency_level_name);
}

//==============================================================================
class FleetUpdateHandle::SpeedLimitRequest::Implementation
{
public:
  std::size_t lane_index;
  double speed_limit;
};

//==============================================================================
FleetUpdateHandle::SpeedLimitRequest::SpeedLimitRequest(
  std::size_t lane_index,
  double speed_limit)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{
      std::move(lane_index),
      std::move(speed_limit)
    }))
{
  // Do nothing
}

//==============================================================================
std::size_t FleetUpdateHandle::SpeedLimitRequest::lane_index() const
{
  return _pimpl->lane_index;
}

//==============================================================================
double FleetUpdateHandle::SpeedLimitRequest::speed_limit() const
{
  return _pimpl->speed_limit;
}

//==============================================================================
auto FleetUpdateHandle::limit_lane_speeds(
  std::vector<SpeedLimitRequest> requests) -> void
{
  _pimpl->worker.schedule(
    [w = weak_from_this(), requests = std::move(requests)](const auto&)
    {
      if (requests.empty())
        return;
      const auto self = w.lock();
      if (!self)
        return;

      auto new_config = (*self->_pimpl->planner)->get_configuration();
      auto& new_graph = new_config.graph();
      for (const auto& request : requests)
      {
        // TODO: Check if planner supports negative speed limits.
        if (request.lane_index() >= new_graph.num_lanes() ||
        request.speed_limit() <= 0.0)
        {
          RCLCPP_WARN(
            self->_pimpl->node->get_logger(),
            "Ignoring speed limit request %f for lane %lu in fleet %s as it is "
            "not greater than zero. If you would like to close the lane, use "
            "the FleetUpdateHandle::close_lanes(~) API instead.",
            request.speed_limit(),
            request.lane_index(),
            self->_pimpl->name.c_str()
          );
          continue;
        }
        auto& properties = new_graph.get_lane(
          request.lane_index()).properties();
        properties.speed_limit(request.speed_limit());
        // Bookkeeping
        self->_pimpl->speed_limited_lanes[request.lane_index()] =
        request.speed_limit();
      }

      *self->_pimpl->planner =
      std::make_shared<const rmf_traffic::agv::Planner>(
        new_config, rmf_traffic::agv::Planner::Options(nullptr));

      self->_pimpl->task_parameters->planner(*self->_pimpl->planner);
      self->_pimpl->publish_lane_states();
    });
}

//==============================================================================
void FleetUpdateHandle::remove_speed_limits(std::vector<std::size_t> requests)
{
  _pimpl->worker.schedule(
    [w = weak_from_this(), requests = std::move(requests)](const auto&)
    {
      if (requests.empty())
        return;
      const auto self = w.lock();
      if (!self)
        return;

      auto new_config = (*self->_pimpl->planner)->get_configuration();
      auto& new_graph = new_config.graph();
      for (const auto& request : requests)
      {

        if (auto it = self->_pimpl->speed_limited_lanes.find(request) ==
        self->_pimpl->speed_limited_lanes.end())
          continue;
        auto& properties = new_graph.get_lane(
          request).properties();
        properties.speed_limit(std::nullopt);
        // Bookkeeping
        self->_pimpl->speed_limited_lanes.erase(request);
      }

      *self->_pimpl->planner =
      std::make_shared<const rmf_traffic::agv::Planner>(
        new_config, rmf_traffic::agv::Planner::Options(nullptr));

      self->_pimpl->task_parameters->planner(*self->_pimpl->planner);
      self->_pimpl->publish_lane_states();
    });
}

//==============================================================================
void FleetUpdateHandle::Implementation::publish_lane_states() const
{
  if (lane_states_pub == nullptr)
    return;
  auto msg = std::make_unique<LaneStates>();
  msg->fleet_name = name;
  for (const auto& index : closed_lanes)
    msg->closed_lanes.push_back(index);
  for (const auto& [index, limit] : speed_limited_lanes)
  {
    // TODO(YV): Use type_adapter in Humble to avoid this copy
    msg->speed_limits.push_back(
      rmf_fleet_msgs::build<rmf_fleet_msgs::msg::SpeedLimitedLane>()
      .lane_index(index)
      .speed_limit(limit));
  }
  lane_states_pub->publish(std::move(msg));
}
//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::accept_task_requests(
  AcceptTaskRequest check)
{
  const auto legacy_converter =
    [check = std::move(check)](
    const rmf_task_msgs::msg::TaskProfile& profile,
    Confirmation& confirm)
    {
      if (check(profile))
      {
        confirm.accept();
        return;
      }

      confirm.errors({"Task rejected by legacy AcceptTaskRequest callback"});
    };

  const auto convert_item = [](const nlohmann::json& item)
    -> rmf_dispenser_msgs::msg::DispenserRequestItem
    {
      rmf_dispenser_msgs::msg::DispenserRequestItem output;
      output.type_guid = item["sku"];
      output.quantity = item["quantity"];
      const auto compartment_it = item.find("compartment");
      if (compartment_it != item.end())
        output.compartment_name = compartment_it->get<std::string>();

      return output;
    };

  const auto convert_items = [convert_item](const nlohmann::json& payload)
    -> std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem>
    {
      std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items;
      if (payload.is_object())
      {
        items.push_back(convert_item(payload));
      }
      else if (payload.is_array())
      {
        for (const auto& p : payload)
          items.push_back(convert_item(p));
      }
      else
      {
        /* *INDENT-OFF* */
        throw std::runtime_error(
          "Invalid payload message for delivery request: " + payload.dump());
        /* *INDENT-ON* */
      }

      return items;
    };

  auto consider_pickup = [legacy_converter, convert_items](
    const nlohmann::json& msg, Confirmation& confirm)
    {
      rmf_task_msgs::msg::TaskProfile profile;
      profile.description.task_type.type =
        rmf_task_msgs::msg::TaskType::TYPE_DELIVERY;

      profile.description.delivery =
        rmf_task_msgs::build<rmf_task_msgs::msg::Delivery>()
        .task_id("")
        .items(convert_items(msg["payload"]))
        .pickup_place_name(msg["place"].get<std::string>())
        .pickup_dispenser(msg["handler"].get<std::string>())
        .pickup_behavior(rmf_task_msgs::msg::Behavior{})
        .dropoff_place_name("")
        .dropoff_ingestor("")
        .dropoff_behavior(rmf_task_msgs::msg::Behavior{});

      legacy_converter(profile, confirm);
    };

  auto consider_dropoff = [legacy_converter, convert_items](
    const nlohmann::json& msg, Confirmation& confirm)
    {
      rmf_task_msgs::msg::TaskProfile profile;
      profile.description.task_type.type =
        rmf_task_msgs::msg::TaskType::TYPE_DELIVERY;

      profile.description.delivery =
        rmf_task_msgs::build<rmf_task_msgs::msg::Delivery>()
        .task_id("")
        .items(convert_items(msg["payload"]))
        .pickup_place_name("")
        .pickup_dispenser("")
        .pickup_behavior(rmf_task_msgs::msg::Behavior{})
        .dropoff_place_name(msg["place"].get<std::string>())
        .dropoff_ingestor(msg["handler"].get<std::string>())
        .dropoff_behavior(rmf_task_msgs::msg::Behavior{});

      legacy_converter(profile, confirm);
    };

  consider_delivery_requests(
    std::move(consider_pickup),
    std::move(consider_dropoff));

  consider_patrol_requests(
    [legacy_converter](const nlohmann::json& msg, Confirmation& confirm)
    {
      rmf_task_msgs::msg::TaskProfile profile;

      // We use loop here because patrol wasn't supported during the legacy
      // versions anyway.
      profile.description.task_type.type =
      rmf_task_msgs::msg::TaskType::TYPE_LOOP;

      rmf_task_msgs::msg::Loop loop;
      const auto& places = msg["places"];
      if (places.size() == 1)
      {
        const auto& place = places[0];
        if (!place.is_string())
        {
          confirm.errors(
            {"Legacy AcceptTaskRequest only accepts destination names "
              "for patrol requests"});
          return;
        }

        loop.start_name = places[0].get<std::string>();
        loop.finish_name = loop.start_name;
      }
      else
      {
        const auto& start_place = places[0];
        const auto& finish_place = places[1];
        if (!start_place.is_string() || !finish_place.is_string())
        {
          confirm.errors(
            {"Legacy AcceptTaskRequest only accepts destination names "
              "for patrol requests"});
          return;
        }

        loop.start_name = start_place.get<std::string>();
        loop.finish_name = finish_place.get<std::string>();
      }

      loop.num_loops = msg["rounds"].get<uint32_t>();
      profile.description.loop = std::move(loop);

      legacy_converter(profile, confirm);
    });

  consider_cleaning_requests(
    [legacy_converter](const nlohmann::json& msg, Confirmation& confirm)
    {
      rmf_task_msgs::msg::TaskProfile profile;

      profile.description.task_type.type =
      rmf_task_msgs::msg::TaskType::TYPE_CLEAN;

      profile.description.clean.start_waypoint = msg["zone"].get<std::string>();

      legacy_converter(profile, confirm);
    });

  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::accept_delivery_requests(
  AcceptDeliveryRequest check)
{
  _pimpl->accept_delivery = std::move(check);
  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::default_maximum_delay(
  std::optional<rmf_traffic::Duration> value)
{
  _pimpl->default_maximum_delay = value;
  return *this;
}

//==============================================================================
std::optional<rmf_traffic::Duration>
FleetUpdateHandle::default_maximum_delay() const
{
  return _pimpl->default_maximum_delay;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::fleet_state_publish_period(
  std::optional<rmf_traffic::Duration> value)
{
  return fleet_state_topic_publish_period(value);
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::fleet_state_topic_publish_period(
  std::optional<rmf_traffic::Duration> value)
{
  if (value.has_value())
  {
    _pimpl->fleet_state_topic_publish_timer =
      _pimpl->node->try_create_wall_timer(
      value.value(),
      [me = weak_from_this()]()
      {
        if (const auto self = me.lock())
          self->_pimpl->publish_fleet_state_topic();
      });
  }
  else
  {
    _pimpl->fleet_state_topic_publish_timer = nullptr;
  }

  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::fleet_state_update_period(
  std::optional<rmf_traffic::Duration> value)
{
  if (value.has_value())
  {
    _pimpl->fleet_state_update_timer =
      _pimpl->node->try_create_wall_timer(
      value.value(),
      [me = weak_from_this()]
      {
        if (const auto self = me.lock())
          self->_pimpl->update_fleet();
      });
  }
  else
  {
    _pimpl->fleet_state_update_timer = nullptr;
  }

  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::set_update_listener(
  std::function<void(const nlohmann::json&)> listener)
{
  std::unique_lock<std::mutex> lock(*_pimpl->update_callback_mutex);
  _pimpl->update_callback = std::move(listener);
  return *this;
}

//==============================================================================
std::optional<rmf_traffic::Duration>
FleetUpdateHandle::retreat_to_charger_interval() const
{
  return _pimpl->retreat_to_charger_interval;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::set_retreat_to_charger_interval(
  std::optional<rmf_traffic::Duration> duration)
{
  _pimpl->retreat_to_charger_interval = duration;

  // Start retreat timer
  for (const auto& t : _pimpl->task_managers)
  {
    t.second->configure_retreat_to_charger(duration);
  }
  return *this;
}

//==============================================================================
void FleetUpdateHandle::reassign_dispatched_tasks()
{
  _pimpl->worker.schedule(
    [w = weak_from_this()](const auto&)
    {
      const auto self = w.lock();
      if (!self)
        return;
      self->_pimpl->reassign_dispatched_tasks([]() {}, [](auto) {});
    }
  );
}

//==============================================================================
std::shared_ptr<rclcpp::Node> FleetUpdateHandle::node()
{
  return _pimpl->node;
}

//==============================================================================
std::shared_ptr<const rclcpp::Node> FleetUpdateHandle::node() const
{
  return _pimpl->node;
}

//==============================================================================
bool FleetUpdateHandle::set_task_planner_params(
  rmf_battery::agv::ConstBatterySystemPtr battery_system,
  rmf_battery::ConstMotionPowerSinkPtr motion_sink,
  rmf_battery::ConstDevicePowerSinkPtr ambient_sink,
  rmf_battery::ConstDevicePowerSinkPtr tool_sink,
  double recharge_threshold,
  double recharge_soc,
  bool account_for_battery_drain,
  rmf_task::ConstRequestFactoryPtr idle_task)
{
  if (battery_system &&
    motion_sink &&
    ambient_sink &&
    tool_sink &&
    (recharge_threshold >= 0.0 && recharge_threshold <= 1.0) &&
    (recharge_soc >= 0.0 && recharge_threshold <= 1.0))
  {
    const rmf_task::Parameters parameters{
      *_pimpl->planner,
      *battery_system,
      motion_sink,
      ambient_sink,
      tool_sink};
    const rmf_task::Constraints constraints{
      recharge_threshold,
      recharge_soc,
      account_for_battery_drain};
    const rmf_task::TaskPlanner::Configuration task_config{
      parameters,
      constraints,
      _pimpl->cost_calculator};
    const rmf_task::TaskPlanner::Options options{
      false,
      nullptr,
      // The finishing request is no longer handled by the planner, we handle
      // it separately as a waiting behavior now.
      nullptr};

    _pimpl->worker.schedule(
      [w = weak_from_this(), task_config, options, idle_task](const auto&)
      {
        const auto self = w.lock();
        if (!self)
          return;

        self->_pimpl->idle_task = idle_task;

        // Here we update the task planner in all the RobotContexts.
        // The TaskManagers rely on the parameters in the task planner for
        // automatic retreat. Hence, we also update them whenever the
        // task planner here is updated.
        self->_pimpl->task_planner = std::make_shared<rmf_task::TaskPlanner>(
          self->_pimpl->name, std::move(task_config), std::move(options));

        for (const auto& t : self->_pimpl->task_managers)
        {
          t.first->task_planner(self->_pimpl->task_planner);
          t.second->set_idle_task(idle_task);
        }
      });

    return true;
  }

  return false;
}

//==============================================================================
FleetUpdateHandle::FleetUpdateHandle()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
