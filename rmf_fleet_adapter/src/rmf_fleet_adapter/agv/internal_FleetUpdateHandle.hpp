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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP

#include <rmf_task_msgs/msg/loop.hpp>

#include <rmf_task_ros2/bidding/AsyncBidder.hpp>

#include <rmf_task_msgs/msg/dispatch_command.hpp>
#include <rmf_task_msgs/msg/dispatch_ack.hpp>

#include <rmf_task/TaskPlanner.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/requests/Clean.hpp>
#include <rmf_task/BinaryPriorityScheme.hpp>

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/Phase.hpp>
#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/PerformAction.hpp>

#include <rmf_building_map_msgs/msg/graph.hpp>

#include <rmf_fleet_msgs/msg/dock_summary.hpp>

#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include "Node.hpp"
#include "RobotContext.hpp"
#include "../TaskManager.hpp"
#include "../DeserializeJSON.hpp"
#include <rmf_websocket/BroadcastClient.hpp>

#include <rmf_traffic/schedule/Mirror.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/LaneClosure.hpp>

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>
#include <rmf_api_msgs/schemas/fleet_state_update.hpp>
#include <rmf_api_msgs/schemas/fleet_state.hpp>
#include <rmf_api_msgs/schemas/robot_state.hpp>
#include <rmf_api_msgs/schemas/location_2D.hpp>
#include <rmf_api_msgs/schemas/fleet_log_update.hpp>
#include <rmf_api_msgs/schemas/fleet_log.hpp>
#include <rmf_api_msgs/schemas/log_entry.hpp>

#include <rmf_fleet_adapter/schemas/event_description__perform_action.hpp>

#include <iostream>
#include <unordered_set>
#include <optional>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
struct TaskActivation
{
  rmf_task::ActivatorPtr task;
  rmf_task_sequence::Phase::ActivatorPtr phase;
  rmf_task_sequence::Event::InitializerPtr event;
};

//==============================================================================
template<typename T>
struct DeserializedDescription
{
  T description;
  std::vector<std::string> errors;
};

//==============================================================================
using DeserializedTask =
  DeserializedDescription<std::shared_ptr<const rmf_task::Task::Description>>;

//==============================================================================
using TaskDescriptionDeserializer =
  std::function<DeserializedTask(const nlohmann::json&)>;

//==============================================================================
using DeserializedPhase =
  DeserializedDescription<
  std::shared_ptr<const rmf_task_sequence::Phase::Description>
  >;

//==============================================================================
using PhaseDescriptionDeserializer =
  std::function<DeserializedPhase(const nlohmann::json&)>;

//==============================================================================
using DeserializedEvent =
  DeserializedDescription<
  std::shared_ptr<const rmf_task_sequence::Event::Description>
  >;

//==============================================================================
using EventDescriptionDeserializer =
  std::function<DeserializedEvent(const nlohmann::json&)>;

//==============================================================================
using DeserializedPlace =
  DeserializedDescription<std::optional<rmf_traffic::agv::Plan::Goal>>;

//==============================================================================
using PlaceDeserializer =
  std::function<DeserializedPlace(const nlohmann::json&)>;

//==============================================================================
struct TaskDeserialization
{
  DeserializeJSONPtr<DeserializedTask> task;
  DeserializeJSONPtr<DeserializedPhase> phase;
  DeserializeJSONPtr<DeserializedEvent> event;
  PlaceDeserializer place;

  std::shared_ptr<FleetUpdateHandle::ConsiderRequest> consider_pickup;
  std::shared_ptr<FleetUpdateHandle::ConsiderRequest> consider_dropoff;
  std::shared_ptr<FleetUpdateHandle::ConsiderRequest> consider_clean;
  std::shared_ptr<FleetUpdateHandle::ConsiderRequest> consider_patrol;
  std::shared_ptr<FleetUpdateHandle::ConsiderRequest> consider_composed;
  // Map category string to its ConsiderRequest for PerformAction events
  std::shared_ptr<std::unordered_map<
      std::string, FleetUpdateHandle::ConsiderRequest>> consider_actions;

  void add_schema(const nlohmann::json& schema);

  nlohmann::json_schema::json_validator make_validator(
    nlohmann::json schema) const;

  std::shared_ptr<nlohmann::json_schema::json_validator> make_validator_shared(
    nlohmann::json schema) const;

  TaskDeserialization();
private:
  using SchemaDictionary = std::unordered_map<std::string, nlohmann::json>;
  std::shared_ptr<SchemaDictionary> _schema_dictionary;
  std::function<void(const nlohmann::json_uri&, nlohmann::json&)> _loader;
};

//==============================================================================
/// This abstract interface class allows us to use the same implementation of
/// FleetUpdateHandle whether we are running it in a distributed system or in a
/// single-process testing environment.
class ParticipantFactory
{
public:

  using ReadyCallback = std::function<void(rmf_traffic::schedule::Participant)>;

  virtual void async_make_participant(
    rmf_traffic::schedule::ParticipantDescription description,
    ReadyCallback ready_callback) = 0;

  virtual ~ParticipantFactory() = default;
};

//==============================================================================
class SimpleParticipantFactory : public ParticipantFactory
{
public:

  SimpleParticipantFactory(
    std::shared_ptr<rmf_traffic::schedule::Writer> writer)
  : _writer{std::move(writer)}
  {
    // Do nothing
  }

  void async_make_participant(
    rmf_traffic::schedule::ParticipantDescription description,
    ReadyCallback ready_callback) final
  {
    ready_callback(
      rmf_traffic::schedule::make_participant(
        std::move(description),
        _writer,
        nullptr)
    );
  }

private:
  std::shared_ptr<rmf_traffic::schedule::Writer> _writer;
};

//==============================================================================
class ParticipantFactoryRos2 : public ParticipantFactory
{
public:

  ParticipantFactoryRos2(
    rmf_traffic_ros2::schedule::WriterPtr writer)
  : _writer{std::move(writer)}
  {
    // Do nothing
  }

  void async_make_participant(
    rmf_traffic::schedule::ParticipantDescription description,
    ReadyCallback ready_callback) final
  {
    _writer->async_make_participant(
      std::move(description),
      std::move(ready_callback));
  }

private:
  rmf_traffic_ros2::schedule::WriterPtr _writer;
};

//==============================================================================
class FleetUpdateHandle::Implementation
{
public:

  std::weak_ptr<FleetUpdateHandle> weak_self;
  std::string name;
  std::shared_ptr<std::shared_ptr<const rmf_traffic::agv::Planner>> planner;
  std::shared_ptr<Node> node;
  rxcpp::schedulers::worker worker;
  std::shared_ptr<ParticipantFactory> writer;
  std::shared_ptr<const rmf_traffic::schedule::Mirror> mirror;
  std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation;
  std::optional<std::string> server_uri;

  std::shared_ptr<std::mutex> update_callback_mutex =
    std::make_shared<std::mutex>();
  std::function<void(const nlohmann::json&)> update_callback = nullptr;

  TaskActivation activation = TaskActivation();
  TaskDeserialization deserialization = TaskDeserialization();

  // LegacyTask planner params
  std::shared_ptr<rmf_task::CostCalculator> cost_calculator =
    rmf_task::BinaryPriorityScheme::make_cost_calculator();
  std::shared_ptr<rmf_task::Parameters> task_parameters = nullptr;
  std::shared_ptr<rmf_task::TaskPlanner> task_planner = nullptr;

  rmf_utils::optional<rmf_traffic::Duration> default_maximum_delay =
    std::chrono::nanoseconds(std::chrono::seconds(10));

  AcceptDeliveryRequest accept_delivery = nullptr;
  std::unordered_map<RobotContextPtr,
    std::shared_ptr<TaskManager>> task_managers = {};

  std::shared_ptr<rmf_websocket::BroadcastClient> broadcast_client = nullptr;
  // Map uri to schema for validator loader function
  std::unordered_map<std::string, nlohmann::json> schema_dictionary = {};

  rclcpp::Publisher<rmf_fleet_msgs::msg::FleetState>::SharedPtr
    fleet_state_pub = nullptr;
  rclcpp::TimerBase::SharedPtr fleet_state_topic_publish_timer = nullptr;
  rclcpp::TimerBase::SharedPtr fleet_state_update_timer = nullptr;

  // Map task id to pair of <RequestPtr, Assignments>
  using Assignments = rmf_task::TaskPlanner::Assignments;

  using DockParamMap =
    std::unordered_map<
    std::string,
    rmf_fleet_msgs::msg::DockParameter
    >;

  using ConstDockParamsPtr = std::shared_ptr<const DockParamMap>;

  // Map of dock name to dock parameters
  std::shared_ptr<DockParamMap> dock_param_map =
    std::make_shared<DockParamMap>();

  // TODO Support for various charging configurations
  std::unordered_set<std::size_t> charging_waypoints = {};

  std::shared_ptr<rmf_task_ros2::bidding::AsyncBidder> bidder = nullptr;

  double current_assignment_cost = 0.0;
  // Map to store task id with assignments for BidNotice
  std::unordered_map<std::string, Assignments> bid_notice_assignments = {};

  using BidNoticeMsg = rmf_task_msgs::msg::BidNotice;

  using DispatchCmdMsg = rmf_task_msgs::msg::DispatchCommand;
  using DispatchCommandSub = rclcpp::Subscription<DispatchCmdMsg>::SharedPtr;
  DispatchCommandSub dispatch_command_sub = nullptr;

  using DispatchAck = rmf_task_msgs::msg::DispatchAck;
  using DispatchAckPub = rclcpp::Publisher<DispatchAck>::SharedPtr;
  DispatchAckPub dispatch_ack_pub = nullptr;

  using DockSummary = rmf_fleet_msgs::msg::DockSummary;
  using DockSummarySub = rclcpp::Subscription<DockSummary>::SharedPtr;
  DockSummarySub dock_summary_sub = nullptr;

  using GraphMsg = rmf_building_map_msgs::msg::Graph;
  rclcpp::Publisher<GraphMsg>::SharedPtr nav_graph_pub = nullptr;

  mutable rmf_task::Log::Reader log_reader = {};

  template<typename... Args>
  static std::shared_ptr<FleetUpdateHandle> make(Args&& ... args)
  {
    auto handle = std::shared_ptr<FleetUpdateHandle>(new FleetUpdateHandle);
    handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
      Implementation{handle, std::forward<Args>(args)...});

    handle->_pimpl->add_standard_tasks();

    // TODO(MXG): This is a very crude implementation. We create a dummy set of
    // task planner parameters to stand in until the user sets the task planner
    // parameters. We'll distribute this shared_ptr to the robot contexts and
    // update it with the proper values once they're available.
    //
    // Note that we also need to manually update the traffic planner when it
    // changes.
    handle->_pimpl->task_parameters =
      std::make_shared<rmf_task::Parameters>(
      *handle->_pimpl->planner,
      *rmf_battery::agv::BatterySystem::make(1.0, 1.0, 1.0),
      nullptr, nullptr, nullptr);

    handle->_pimpl->fleet_state_pub = handle->_pimpl->node->fleet_state();
    handle->fleet_state_topic_publish_period(std::chrono::seconds(1));
    handle->fleet_state_update_period(std::chrono::seconds(1));

    // Create subs and pubs for bidding
    auto transient_qos = rclcpp::QoS(10).transient_local();
    auto reliable_transient_qos =
      rclcpp::ServicesQoS().keep_last(20).transient_local();

    // Subscribe DispatchCommand
    handle->_pimpl->dispatch_command_sub =
      handle->_pimpl->node->create_subscription<DispatchCmdMsg>(
      DispatchRequestTopicName,
      reliable_transient_qos,
      [w = handle->weak_from_this()](const DispatchCmdMsg::SharedPtr msg)
      {
        if (const auto self = w.lock())
          self->_pimpl->dispatch_command_cb(msg);
      });

    // Publish DispatchAck
    handle->_pimpl->dispatch_ack_pub =
      handle->_pimpl->node->create_publisher<DispatchAck>(
      DispatchAckTopicName, reliable_transient_qos);

    // Make a dispatch bidder
    handle->_pimpl->bidder = rmf_task_ros2::bidding::AsyncBidder::make(
      handle->_pimpl->node,
      [w = handle->weak_from_this()](
        const auto& msg, auto respond)
      {
        if (const auto self = w.lock())
          self->_pimpl->bid_notice_cb(msg, std::move(respond));
      });

    // Publisher for navigation graph
    handle->_pimpl->nav_graph_pub =
      handle->_pimpl->node->create_publisher<GraphMsg>(
      NavGraphTopicName, transient_qos);
    handle->_pimpl->publish_nav_graph();

    // Subscribe DockSummary
    handle->_pimpl->dock_summary_sub =
      handle->_pimpl->node->create_subscription<DockSummary>(
      DockSummaryTopicName,
      transient_qos,
      [w = handle->weak_from_this()](const DockSummary::SharedPtr msg)
      {
        if (const auto self = w.lock())
          self->_pimpl->dock_summary_cb(msg);
      });

    // Populate charging waypoints
    const auto& graph = (*handle->_pimpl->planner)->get_configuration().graph();
    for (std::size_t i = 0; i < graph.num_waypoints(); ++i)
    {
      if (graph.get_waypoint(i).is_charger())
        handle->_pimpl->charging_waypoints.insert(i);
    }

    // Initialize schema dictionary
    const std::vector<nlohmann::json> schemas = {
      rmf_api_msgs::schemas::fleet_state_update,
      rmf_api_msgs::schemas::fleet_state,
      rmf_api_msgs::schemas::robot_state,
      rmf_api_msgs::schemas::location_2D,
      rmf_api_msgs::schemas::fleet_log,
      rmf_api_msgs::schemas::log_entry
    };

    for (const auto& schema : schemas)
    {
      const auto json_uri = nlohmann::json_uri{schema["$id"]};
      handle->_pimpl->schema_dictionary.insert({json_uri.url(), schema});
    }

    // Start the BroadcastClient
    if (handle->_pimpl->server_uri.has_value())
    {
      handle->_pimpl->broadcast_client = rmf_websocket::BroadcastClient::make(
        handle->_pimpl->server_uri.value(),
        handle->_pimpl->node,
        [handle]()
        {
          std::vector<nlohmann::json> task_logs;
          for (const auto& [conext, mgr] : handle->_pimpl->task_managers)
          {
            // Publish all task logs to the server
            task_logs.push_back(mgr->task_log_updates());
          }
          return task_logs;
        });
    }

    // Add PerformAction event to deserialization
    auto validator = handle->_pimpl->deserialization.make_validator_shared(
      schemas::event_description__perform_action);

    const auto deserializer =
      [
      validator,
      place = handle->_pimpl->deserialization.place,
      consider_actions = handle->_pimpl->deserialization.consider_actions
      ](const nlohmann::json& msg) -> DeserializedEvent
      {
        try
        {
          validator->validate(msg);
          const std::string& category = msg["category"].get<std::string>();
          const auto consider_action_it = consider_actions->find(category);
          if (consider_action_it == consider_actions->end())
          {
            return {nullptr, {"Fleet not configured to perform this action"}};
          }
          Confirmation confirm;
          const auto& consider = consider_action_it->second;
          consider(msg, confirm);
          if (!confirm.is_accepted())
          {
            return {nullptr, confirm.errors()};
          }

          const nlohmann::json desc = msg["description"];
          rmf_traffic::Duration duration_estimate = rmf_traffic::Duration(0);
          bool use_tool_sink = false;
          std::optional<rmf_traffic::agv::Planner::Goal> finish_location =
            std::nullopt;
          auto it = msg.find("unix_millis_action_duration_estimate");
          if (it != msg.end())
          {
            duration_estimate =
              rmf_traffic::Duration(
              std::chrono::milliseconds(it->get<uint64_t>()));
          }
          it = msg.find("use_tool_sink");
          if (it != msg.end())
          {
            use_tool_sink = it->get<bool>();
          }
          it = msg.find("expected_finish_location");
          if (it != msg.end())
          {
            auto deser_place =
              place(msg["expected_finish_location"]);
            if (!deser_place.description.has_value())
            {
              return {nullptr, deser_place.errors};
            }
            finish_location = deser_place.description.value();
          }

          const auto description =
            rmf_task_sequence::events::PerformAction::Description::make(
            category,
            desc,
            duration_estimate,
            use_tool_sink,
            finish_location);

          std::vector<std::string> errors = {};
          return {description, errors};
        }
        catch (const std::exception& e)
        {
          return {nullptr, {e.what()}};
        }
      };

    handle->_pimpl->deserialization.event->add(
      "perform_action", validator, deserializer);

    return handle;
  }

  void publish_nav_graph() const;

  void dock_summary_cb(const DockSummary::SharedPtr& msg);

  void bid_notice_cb(
    const BidNoticeMsg& msg,
    rmf_task_ros2::bidding::AsyncBidder::Respond respond);

  void dispatch_command_cb(const DispatchCmdMsg::SharedPtr msg);

  std::optional<std::size_t> get_nearest_charger(
    const rmf_traffic::agv::Planner::Start& start);

  struct Expectations
  {
    std::vector<rmf_task::State> states;
    std::vector<rmf_task::ConstRequestPtr> pending_requests;
  };

  Expectations aggregate_expectations() const;

  /// Generate task assignments for a collection of task requests comprising of
  /// task requests currently in TaskManager queues while optionally including a
  /// new request and while optionally ignoring a specific request.
  std::optional<Assignments> allocate_tasks(
    rmf_task::ConstRequestPtr new_request = nullptr,
    std::vector<std::string>* errors = nullptr,
    std::optional<Expectations> expectations = std::nullopt) const;

  /// Helper function to check if assignments are valid. An assignment set is
  /// invalid if one of the assignments has already begun execution.
  bool is_valid_assignments(Assignments& assignments) const;

  static Implementation& get(FleetUpdateHandle& fleet)
  {
    return *fleet._pimpl;
  }

  static const Implementation& get(const FleetUpdateHandle& fleet)
  {
    return *fleet._pimpl;
  }

  void publish_fleet_state_topic() const;

  void update_fleet() const;

  void update_fleet_state() const;
  void update_fleet_logs() const;

  nlohmann::json_schema::json_validator make_validator(
    const nlohmann::json& schema) const;

  void add_standard_tasks();

  std::string make_error_str(
    uint64_t code, std::string category, std::string detail) const;

  std::shared_ptr<rmf_task::Request> convert(
    const std::string& task_id,
    const nlohmann::json& request_msg,
    std::vector<std::string>& errors) const;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
