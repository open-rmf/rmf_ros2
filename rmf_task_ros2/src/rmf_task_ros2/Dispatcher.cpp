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

#include <rmf_task_ros2/Dispatcher.hpp>
#include <rmf_task_ros2/StandardNames.hpp>

#include <rmf_websocket/BroadcastClient.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <rmf_task_msgs/msg/task_description.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>
#include <rmf_task_msgs/srv/submit_task.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <rmf_task_msgs/msg/dispatch_states.hpp>
#include <rmf_task_msgs/msg/dispatch_command.hpp>
#include <rmf_task_msgs/msg/dispatch_ack.hpp>
#include <rmf_task_msgs/srv/get_dispatch_states.hpp>
#include <rmf_task_msgs/msg/tasks.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>

#include <rmf_api_msgs/schemas/dispatch_task_request.hpp>
#include <rmf_api_msgs/schemas/dispatch_task_response.hpp>
#include <rmf_api_msgs/schemas/task_request.hpp>
#include <rmf_api_msgs/schemas/task_state.hpp>
#include <rmf_api_msgs/schemas/error.hpp>

#include <unordered_set>

namespace rmf_task_ros2 {

namespace {
//==============================================================================
std::function<void(const nlohmann::json_uri& id, nlohmann::json& value)>
make_schema_loader()
{
  const std::vector<nlohmann::json> schemas = {
    rmf_api_msgs::schemas::dispatch_task_request,
    rmf_api_msgs::schemas::dispatch_task_response,
    rmf_api_msgs::schemas::task_request,
    rmf_api_msgs::schemas::task_state,
    rmf_api_msgs::schemas::error
  };

  std::unordered_map<std::string, nlohmann::json> dictionary;
  for (const auto& schema : schemas)
  {
    const auto json_uri = nlohmann::json_uri(schema["$id"]);
    dictionary[json_uri.url()] = schema;
  }

  return [dictionary = std::move(dictionary)](
    const nlohmann::json_uri& id, nlohmann::json& value)
    {
      const auto it = dictionary.find(id.url());
      if (it != dictionary.end())
        value = it->second;
    };
}

//==============================================================================
static const
std::function<void(const nlohmann::json_uri& id, nlohmann::json& value)>
schema_loader = make_schema_loader();

//==============================================================================
nlohmann::json_schema::json_validator make_validator(nlohmann::json schema)
{
  return nlohmann::json_schema::json_validator(
    std::move(schema), schema_loader);
}
} // anonymous namespace

//==============================================================================
class Dispatcher::Implementation
{
public:

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<bidding::Auctioneer> auctioneer;
  std::shared_ptr<rmf_websocket::BroadcastClient> broadcast_client;

  const nlohmann::json _task_state_update_json =
  {{"type", "task_state_update"}, {"data", {}}};

  using SubmitTaskSrv = rmf_task_msgs::srv::SubmitTask;
  using CancelTaskSrv = rmf_task_msgs::srv::CancelTask;
  using GetDispatchStatesSrv = rmf_task_msgs::srv::GetDispatchStates;
  using DispatchStateMsg = rmf_task_msgs::msg::DispatchState;
  using DispatchStatesMsg = rmf_task_msgs::msg::DispatchStates;
  using TaskDescription = rmf_task_msgs::msg::TaskDescription;
  using DispatchCommandMsg = rmf_task_msgs::msg::DispatchCommand;
  using DispatchAckMsg = rmf_task_msgs::msg::DispatchAck;

  rclcpp::Service<SubmitTaskSrv>::SharedPtr submit_task_srv;
  rclcpp::Service<CancelTaskSrv>::SharedPtr cancel_task_srv;
  rclcpp::Service<GetDispatchStatesSrv>::SharedPtr get_dispatch_states_srv;

  using ApiRequestMsg = rmf_task_msgs::msg::ApiRequest;
  using ApiResponseMsg = rmf_task_msgs::msg::ApiResponse;
  rclcpp::Subscription<ApiRequestMsg>::SharedPtr api_request;
  rclcpp::Publisher<ApiResponseMsg>::SharedPtr api_response;

  class ApiMemory
  {
  public:

    std::optional<ApiResponseMsg> lookup(const std::string& api_id) const
    {
      const auto it = _cached_responses.find(api_id);
      if (it == _cached_responses.end())
        return std::nullopt;

      return it->second;
    }

    void add(ApiResponseMsg msg)
    {
      if (_tracker.size() > _max_size)
      {
        _cached_responses.erase(_tracker.front());
        _tracker.pop_front();
      }

      _tracker.push_back(msg.request_id);
      _cached_responses[msg.request_id] = std::move(msg);
    }

  private:

    std::unordered_map<std::string, ApiResponseMsg> _cached_responses;
    std::list<std::string> _tracker;
    std::size_t _max_size = 50;
  };

  ApiMemory api_memory;

  using DispatchStatesPub = rclcpp::Publisher<DispatchStatesMsg>;
  DispatchStatesPub::SharedPtr dispatch_states_pub;
  rclcpp::TimerBase::SharedPtr dispatch_states_pub_timer;

  uint64_t next_dispatch_command_id = 0;
  std::unordered_map<uint64_t, DispatchCommandMsg> lingering_commands;
  rclcpp::TimerBase::SharedPtr dispatch_command_timer;
  rclcpp::Publisher<DispatchCommandMsg>::SharedPtr dispatch_command_pub;

  rclcpp::Subscription<DispatchAckMsg>::SharedPtr dispatch_ack_sub;

  DispatchStateCallback on_change_fn;

  DispatchStates active_dispatch_states;
  DispatchStates finished_dispatch_states;
  std::size_t task_counter = 0; // index for generating task_id
  builtin_interfaces::msg::Duration bidding_time_window;
  std::size_t terminated_tasks_max_size;
  int publish_active_tasks_period;
  bool use_timestamp_for_task_id;

  std::unordered_map<std::size_t, std::string> legacy_task_type_names =
  {
    {1, "patrol"},
    {2, "delivery"},
    {4, "clean"}
  };

  using LegacyConversion =
    std::function<nlohmann::json(const TaskDescription&)>;
  using LegacyConversionMap = std::unordered_map<std::string, LegacyConversion>;
  LegacyConversionMap legacy_task_types;

  Implementation(std::shared_ptr<rclcpp::Node> node_)
  : node{std::move(node_)}
  {
    // ros2 param
    double bidding_time_window_param =
      node->declare_parameter<double>("bidding_time_window", 2.0);
    RCLCPP_INFO(node->get_logger(),
      " Declared Time Window Param as: %f secs", bidding_time_window_param);
    bidding_time_window = rmf_traffic_ros2::convert(
      rmf_traffic::time::from_seconds(bidding_time_window_param));

    terminated_tasks_max_size =
      node->declare_parameter<int>("terminated_tasks_max_size", 100);
    RCLCPP_INFO(node->get_logger(),
      " Declared Terminated Tasks Max Size Param as: %lu",
      terminated_tasks_max_size);
    publish_active_tasks_period =
      node->declare_parameter<int>("publish_active_tasks_period", 2);
    RCLCPP_INFO(node->get_logger(),
      " Declared publish_active_tasks_period as: %d secs",
      publish_active_tasks_period);
    use_timestamp_for_task_id =
      node->declare_parameter<bool>("use_timestamp_for_task_id", false);
    RCLCPP_INFO(node->get_logger(),
      " Use timestamp with task_id: %s",
      (use_timestamp_for_task_id ? "true" : "false"));

    std::optional<std::string> server_uri = std::nullopt;
    const std::string uri =
      node->declare_parameter("server_uri", std::string());
    if (!uri.empty())
    {
      RCLCPP_INFO(
        node->get_logger(),
        "API server URI: [%s]", uri.c_str());
      server_uri = uri;
    }

    const auto qos = rclcpp::ServicesQoS().reliable();
    dispatch_states_pub = node->create_publisher<DispatchStatesMsg>(
      rmf_task_ros2::DispatchStatesTopicName, qos);

    // TODO(MXG): Sync up with rmf_fleet_adapter/StandardNames on these topic
    // names
    api_request = node->create_subscription<ApiRequestMsg>(
      "task_api_requests",
      rclcpp::SystemDefaultsQoS().reliable().transient_local(),
      [this](const ApiRequestMsg::UniquePtr msg)
      {
        this->handle_api_request(*msg);
      });

    api_response = node->create_publisher<ApiResponseMsg>(
      "task_api_responses",
      rclcpp::SystemDefaultsQoS().reliable().transient_local());

    // TODO(MXG): The smallest resolution this supports is 1 second. That
    // doesn't seem great.
    dispatch_states_pub_timer = node->create_wall_timer(
      std::chrono::seconds(publish_active_tasks_period),
      [this]() { this->publish_dispatch_states(); });

    dispatch_command_pub = node->create_publisher<DispatchCommandMsg>(
      rmf_task_ros2::DispatchCommandTopicName,
      rclcpp::ServicesQoS().keep_last(20).reliable().transient_local());

    // TODO(MXG): Make this publishing period configurable
    dispatch_command_timer = node->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { this->publish_lingering_commands(); });

    dispatch_ack_sub = node->create_subscription<DispatchAckMsg>(
      rmf_task_ros2::DispatchAckTopicName,
      rclcpp::ServicesQoS().keep_last(20).transient_local(),
      [this](const DispatchAckMsg::UniquePtr msg)
      {
        this->handle_dispatch_ack(*msg);
      });

    if (server_uri)
      broadcast_client = rmf_websocket::BroadcastClient::make(
        *server_uri, node);

    auctioneer = bidding::Auctioneer::make(
      node,
      [this](
        const TaskID& task_id,
        const std::optional<bidding::Response::Proposal> winner,
        const std::vector<std::string>& errors)
      {
        this->conclude_bid(task_id, std::move(winner), errors);
      },
      std::make_shared<bidding::QuickestFinishEvaluator>());

    // Setup up stream srv interfaces
    submit_task_srv = node->create_service<SubmitTaskSrv>(
      rmf_task_ros2::SubmitTaskSrvName,
      [this](
        const std::shared_ptr<SubmitTaskSrv::Request> request,
        std::shared_ptr<SubmitTaskSrv::Response> response)
      {
        const auto id = this->submit_task(request->description);
        if (id == std::nullopt)
        {
          response->success = false;
          response->message = "Task type is invalid";
          return;
        }

        response->task_id = *id;
        response->success = true;
      }
    );

    cancel_task_srv = node->create_service<CancelTaskSrv>(
      rmf_task_ros2::CancelTaskSrvName,
      [this](
        const std::shared_ptr<CancelTaskSrv::Request> request,
        std::shared_ptr<CancelTaskSrv::Response> response)
      {
        auto id = request->task_id;
        response->success = this->cancel_task(id);
      }
    );

    get_dispatch_states_srv = node->create_service<GetDispatchStatesSrv>(
      rmf_task_ros2::GetDispatchStatesSrvName,
      [this](
        const std::shared_ptr<GetDispatchStatesSrv::Request> request,
        std::shared_ptr<GetDispatchStatesSrv::Response> response)
      {
        std::unordered_set<std::string> relevant_tasks;
        relevant_tasks.insert(
          request->task_ids.begin(),
          request->task_ids.end());

        /* *INDENT-OFF* */
        const auto fill_states = [&relevant_tasks](auto& into, const auto& from)
          {
            for (const auto& [id, state] : from)
            {
              if (relevant_tasks.empty())
                into.push_back(convert(*state));
              else if (relevant_tasks.count(id))
                into.push_back(convert(*state));
            }
          };
        /* *INDENT-ON* */

        fill_states(response->states.active, this->active_dispatch_states);
        fill_states(response->states.finished, this->finished_dispatch_states);

        response->success = true;
      }
    );

    // Loop
    legacy_task_types["patrol"] =
      [](const TaskDescription& task_description) -> nlohmann::json
      {
        const auto& loop = task_description.loop;
        nlohmann::json description;
        std::vector<std::string> places;
        places.push_back(loop.start_name);
        places.push_back(loop.finish_name);
        description["places"] = std::move(places);
        description["rounds"] = loop.num_loops;

        return description;
      };

    // Delivery
    legacy_task_types["delivery"] =
      [](const TaskDescription& task_description) -> nlohmann::json
      {
        const auto& delivery = task_description.delivery;
        std::vector<nlohmann::json> payload;
        payload.reserve(delivery.items.size());
        for (const auto& item : delivery.items)
        {
          nlohmann::json item_json;
          item_json["sku"] = item.type_guid;
          item_json["quantity"] = item.quantity;
          item_json["compartment"] = item.compartment_name;
          payload.push_back(item_json);
        }

        nlohmann::json pickup;
        pickup["place"] = delivery.pickup_place_name;
        pickup["handler"] = delivery.pickup_dispenser;
        pickup["payload"] = payload;

        nlohmann::json dropoff;
        dropoff["place"] = delivery.dropoff_place_name;
        dropoff["handler"] = delivery.dropoff_ingestor;
        dropoff["payload"] = payload;

        nlohmann::json description;
        description["pickup"] = pickup;
        description["dropoff"] = dropoff;

        return description;
      };

    // Clean
    legacy_task_types["clean"] =
      [](const TaskDescription& task_description) -> nlohmann::json
      {
        const auto& clean = task_description.clean;
        nlohmann::json description;
        description["zone"] = clean.start_waypoint;

        return description;
      };
  }

  void handle_api_request(const ApiRequestMsg& msg)
  {
    const auto check = api_memory.lookup(msg.request_id);
    if (check.has_value())
    {
      api_response->publish(*check);
      return;
    }

    nlohmann::json msg_json;
    try
    {
      msg_json = nlohmann::json::parse(msg.json_msg);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Error parsing json_msg: %s",
        e.what());
      return;
    }

    const auto type_it = msg_json.find("type");
    if (type_it == msg_json.end())
    {
      // Whatever type of message this is, we don't support it
      return;
    }

    if (!type_it.value().is_string())
    {
      // We expect the type field to contain a string
      return;
    }

    try
    {
      const auto& type_str = type_it.value().get<std::string>();
      if (type_str != "dispatch_task_request")
      {
        return;
      }

      static const auto request_validator =
        make_validator(rmf_api_msgs::schemas::dispatch_task_request);

      try
      {
        request_validator.validate(msg_json);
      }
      catch (const std::exception& e)
      {
        nlohmann::json error;
        error["code"] = 5;
        error["category"] = "Invalid request format";
        error["detail"] = e.what();

        nlohmann::json response_json;
        response_json["success"] = false;
        response_json["errors"] =
          std::vector<nlohmann::json>({std::move(error)});

        auto response = rmf_task_msgs::build<ApiResponseMsg>()
          .type(ApiResponseMsg::TYPE_RESPONDING)
          .json_msg(std::move(response_json))
          .request_id(msg.request_id);

        api_memory.add(response);
        api_response->publish(response);
        return;
      }

      const auto& task_request_json = msg_json["request"];
      std::string task_id =
        task_request_json["category"].get<std::string>()
        + ".dispatch-";

      if (use_timestamp_for_task_id)
      {
        task_id += std::to_string(
          static_cast<int>(node->get_clock()->now().nanoseconds()/1e6));
      }
      else
      {
        task_id += std::to_string(task_counter++);
      }

      const auto task_state = push_bid_notice(
        rmf_task_msgs::build<bidding::BidNoticeMsg>()
        .request(task_request_json.dump())
        .task_id(task_id)
        .time_window(bidding_time_window));

      nlohmann::json response_json;
      response_json["success"] = true;
      response_json["state"] = task_state;

      auto response = rmf_task_msgs::build<ApiResponseMsg>()
        .type(ApiResponseMsg::TYPE_RESPONDING)
        .json_msg(response_json.dump())
        .request_id(msg.request_id);

      api_memory.add(response);
      api_response->publish(response);

      // TODO(MXG): Make some way to keep pushing task state updates to the
      // api-server as the bidding process progresses. We could do a websocket
      // connection or maybe just a simple ROS2 publisher.
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Failed to handle API request message: %s", e.what());
    }
  }

  std::optional<TaskID> submit_task(const TaskDescription& submission)
  {
    const auto task_type_index = submission.task_type.type;
    const auto desc_it = legacy_task_type_names.find(task_type_index);
    if (desc_it == legacy_task_type_names.end())
    {
      RCLCPP_ERROR(
        node->get_logger(), "TaskType: %u is invalid", task_type_index);
      return std::nullopt;
    }

    const std::string category = desc_it->second;

    // auto generate a task_id for a given submitted task
    const auto task_id =
      category + ".dispatch-" + std::to_string(task_counter++);

    RCLCPP_INFO(node->get_logger(),
      "Received Task Submission [%s]", task_id.c_str());

    nlohmann::json task_request;
    task_request["unix_millis_earliest_start_time"] =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      rmf_traffic_ros2::convert(submission.start_time).time_since_epoch())
      .count();

    auto& priority = task_request["priority"];
    priority["type"] = "binary";
    priority["value"] = submission.priority.value;
    task_request["category"] = category;
    task_request["description"] = legacy_task_types.at(category)(submission);
    task_request["labels"] = std::vector<std::string>({"legacy_request"});

    push_bid_notice(
      rmf_task_msgs::build<bidding::BidNoticeMsg>()
      .request(task_request.dump())
      .task_id(task_id)
      .time_window(bidding_time_window));

    return task_id;
  }

  nlohmann::json push_bid_notice(bidding::BidNoticeMsg bid_notice)
  {
    const auto request = nlohmann::json::parse(bid_notice.request);
    auto new_dispatch_state =
      std::make_shared<DispatchState>(
      bid_notice.task_id, std::chrono::steady_clock::now());
    new_dispatch_state->request = request;

    // Publish this initial task state message to the websocket
    auto state = publish_task_state_ws(new_dispatch_state, "queued");

    active_dispatch_states[bid_notice.task_id] = new_dispatch_state;

    if (on_change_fn)
      on_change_fn(*new_dispatch_state);

    auctioneer->request_bid(bid_notice);
    return state;
  }

  bool cancel_task(const TaskID& task_id)
  {
    using Status = DispatchState::Status;

    // Check if the task has already terminated for some reason
    const auto finished_it = finished_dispatch_states.find(task_id);
    if (finished_it != finished_dispatch_states.end())
    {
      if (finished_it->second->status == Status::FailedToAssign
        || finished_it->second->status == Status::CanceledInFlight)
      {
        // This task was never assigned to a fleet adapter so we will respond
        // positively that it is cancelled.
        return true;
      }

      if (finished_it->second->status == Status::Dispatched)
      {
        // This task is now the responsibility of a fleet adapter
        return false;
      }

      // If the dispatch status is Queued or Selected then this task is in the
      // wrong dispatch state set.
      RCLCPP_ERROR(
        node->get_logger(),
        "Canceled task [%s] is in the set of finished dispatches but has an "
        "invalid status for that set: %d",
        task_id.c_str(),
        static_cast<uint8_t>(finished_it->second->status));

      return false;
    }

    const auto it = active_dispatch_states.find(task_id);
    if (it == active_dispatch_states.end())
    {
      // This must mean that some other system besides the dispatch system
      // created the task ..?
      RCLCPP_INFO(node->get_logger(),
        "Canceled task [%s] was not dispatched", task_id.c_str());
      return false;
    }

    // Cancel bidding. This will remove the bidding process
    const auto& canceled_dispatch = it->second;
    if (canceled_dispatch->status == DispatchState::Status::Queued)
    {
      canceled_dispatch->status = DispatchState::Status::CanceledInFlight;
      move_to_finished(task_id);

      if (on_change_fn)
        on_change_fn(*canceled_dispatch);

      return true;
    }

    // Cancel the assignment. We will need to make sure the cancelation is seen
    // by the fleet adapter that the task was assigned to.
    if (canceled_dispatch->status == DispatchState::Status::Selected)
    {
      canceled_dispatch->status = DispatchState::Status::CanceledInFlight;
      if (!canceled_dispatch->assignment.has_value())
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Canceled task [%s] has Selected status but its assignment is empty. "
          "This indicates a serious bug. Please report this to the RMF "
          "developers.",
          task_id.c_str());
      }
      else
      {
        auto cancel_command = rmf_task_msgs::build<DispatchCommandMsg>()
          .fleet_name(canceled_dispatch->assignment->fleet_name)
          .task_id(task_id)
          .dispatch_id(next_dispatch_command_id++)
          .timestamp(node->get_clock()->now())
          .type(DispatchCommandMsg::TYPE_REMOVE);

        lingering_commands[cancel_command.dispatch_id] = cancel_command;
        dispatch_command_pub->publish(cancel_command);
      }

      move_to_finished(task_id);

      if (on_change_fn)
        on_change_fn(*canceled_dispatch);

      return true;
    }

    RCLCPP_ERROR(
      node->get_logger(),
      "Canceled task [%s] is in the set of active dispatches but has an "
      "invalid status for that set: %d",
      task_id.c_str(),
      static_cast<uint8_t>(canceled_dispatch->status));

    return false;
  }

  void conclude_bid(
    const TaskID& task_id,
    const std::optional<bidding::Response::Proposal> winner,
    const std::vector<std::string>& errors)
  {
    const auto it = active_dispatch_states.find(task_id);
    if (it == active_dispatch_states.end())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Received a winning bid for a task request [%s] which is no longer "
        "being dispatched. This may indicate a bug and should be reported to "
        "the developers of RMF.",
        task_id.c_str());
      return;
    }

    auto& dispatch_state = it->second;
    for (const auto& error : errors)
    {
      try
      {
        dispatch_state->errors.push_back(nlohmann::json::parse(error));
      }
      catch (const std::exception&)
      {
        // If the message cannot be converted directly into an error message,
        // then we will create an error message for it
        nlohmann::json error_msg;
        error_msg["code"] = 255;
        error_msg["category"] = "unknown";
        error_msg["detail"] = error;

        dispatch_state->errors.push_back(std::move(error_msg));
      }
    }

    if (!winner)
    {
      RCLCPP_WARN(node->get_logger(),
        "Dispatcher Bidding Result: task [%s] has no submissions during "
        "bidding. Dispatching failed, and the task will not be performed.",
        task_id.c_str());

      dispatch_state->status = DispatchState::Status::FailedToAssign;
      nlohmann::json error;
      // TODO(MXG): Standardize the codes
      error["code"] = 10;
      error["category"] = "rejection";
      error["detail"] =
        "No fleet adapters offered a bid for task [" + task_id + "]";

      dispatch_state->errors.push_back(std::move(error));

      if (on_change_fn)
        on_change_fn(*dispatch_state);

      // Print the errors
      std::size_t error_count = 1;
      for (const auto& error : errors)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "No submission error[%lu]: %s",
          error_count,
          error.c_str());
        ++error_count;
      }

      /// Publish failed bid
      publish_task_state_ws(dispatch_state, "failed");

      auctioneer->ready_for_next_bid();
      return;
    }

    // now we know which fleet will execute the task
    dispatch_state->assignment = DispatchState::Assignment{
      winner->fleet_name,
      winner->expected_robot_name
    };
    dispatch_state->status = DispatchState::Status::Selected;

    RCLCPP_INFO(
      node->get_logger(),
      "Dispatcher Bidding Result: task [%s] is awarded to fleet adapter [%s], "
      "with expected robot [%s].",
      task_id.c_str(),
      winner->fleet_name.c_str(),
      winner->expected_robot_name.c_str());

    auto award_command = rmf_task_msgs::build<DispatchCommandMsg>()
      .fleet_name(winner->fleet_name)
      .task_id(task_id)
      .dispatch_id(next_dispatch_command_id++)
      .timestamp(node->get_clock()->now())
      .type(DispatchCommandMsg::TYPE_AWARD);

    lingering_commands[award_command.dispatch_id] = award_command;
    dispatch_command_pub->publish(award_command);
  }

  //==============================================================================
  nlohmann::json publish_task_state_ws(
    const std::shared_ptr<DispatchState> state,
    const std::string& status)
  {
    nlohmann::json task_state;
    auto& booking = task_state["booking"];
    booking["id"] = state->task_id;

    static const std::vector<std::string> copy_fields = {
      "unix_millis_earliest_start_time",
      "priority",
      "labels"
    };

    for (const auto& field : copy_fields)
    {
      const auto f_it = state->request.find(field);
      if (f_it != state->request.end())
        booking[field] = f_it.value();
    }

    task_state["category"] = state->request["category"];
    task_state["detail"] = state->request["description"];
    task_state["status"] = status;

    /// NOTE: This should be null, but the reason of populating this for
    /// now is to provide an estimated start_time to the dashboard, so
    /// sort by start time will still work
    task_state["unix_millis_start_time"] =
      booking["unix_millis_earliest_start_time"];

    /// TODO: populate assignment from state

    nlohmann::json dispatch_json;
    dispatch_json["status"] = status_to_string(state->status);
    dispatch_json["errors"] = state->errors;
    task_state["dispatch"] = dispatch_json;

    auto task_state_update = _task_state_update_json;
    task_state_update["data"] = task_state;

    /// TODO: (YL) json validator for taskstateupdate

    if (broadcast_client)
      broadcast_client->publish(task_state_update);
    return task_state;
  }

  void move_to_finished(const std::string& task_id)
  {
    const auto active_it = active_dispatch_states.find(task_id);

    // TODO(MXG): We can make this more efficient by having a queue of
    // finished dispatch state IDs in chronological order
    if (finished_dispatch_states.size() >= terminated_tasks_max_size)
    {
      auto check_it = finished_dispatch_states.begin();
      auto oldest_it = check_it++;
      for (; check_it != finished_dispatch_states.end(); ++check_it)
      {
        const auto& check = check_it->second->submission_time;
        const auto& oldest = oldest_it->second->submission_time;
        if (check < oldest)
          oldest_it = check_it;
      }

      finished_dispatch_states.erase(oldest_it);
    }

    finished_dispatch_states[active_it->first] = active_it->second;
  }

  void publish_dispatch_states()
  {
    const auto fill_states = [](auto& into, const auto& from)
      {
        for (const auto& [id, state] : from)
          into.push_back(convert(*state));
      };

    std::vector<DispatchStateMsg> active;
    std::vector<DispatchStateMsg> finished;
    fill_states(active, active_dispatch_states);
    fill_states(finished, finished_dispatch_states);

    dispatch_states_pub->publish(
      rmf_task_msgs::build<DispatchStatesMsg>()
      .active(std::move(active))
      .finished(std::move(finished)));
  }

  void publish_lingering_commands()
  {
    std::vector<uint64_t> expired_commands;
    const auto now = node->get_clock()->now();

    // TODO(MXG): Make this timeout period configurable
    const auto timeout = std::chrono::seconds(10);

    for (const auto& [id, r] : lingering_commands)
    {
      const auto timestamp = rclcpp::Time(r.timestamp);
      if (timestamp + timeout < now)
      {
        // This request has expired.
        expired_commands.push_back(id);
        continue;
      }

      dispatch_command_pub->publish(r);
    }

    for (const auto& id : expired_commands)
    {
      const auto it = lingering_commands.find(id);
      if (it == lingering_commands.end())
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Weird bug, [%lu] is no longer in the lingering requests even though "
          "it was just detected as being expired. Please report this to the "
          "RMF developers.", id);
        continue;
      }

      const auto& request = it->second;
      RCLCPP_ERROR(
        node->get_logger(),
        "Dispatch command [%lu] type [%u] for task [%s] directed at fleet [%s] "
        "has expired. This likely means something is wrong with the fleet "
        "adapter for [%s] preventing it from responding.",
        id,
        request.type,
        request.task_id.c_str(),
        request.fleet_name.c_str(),
        request.fleet_name.c_str());

      if (request.type == request.TYPE_AWARD)
        auctioneer->ready_for_next_bid();

      lingering_commands.erase(it);
    }
  }

  void handle_dispatch_ack(const DispatchAckMsg& ack)
  {
    const auto command_it = lingering_commands.find(ack.dispatch_id);
    if (command_it == lingering_commands.end())
    {
      // Already processed this acknowledgment
      return;
    }

    const auto command = std::move(command_it->second);
    lingering_commands.erase(command_it);

    if (command.type == DispatchCommandMsg::TYPE_AWARD)
    {
      const auto state_it = active_dispatch_states.find(command.task_id);
      if (state_it == active_dispatch_states.end())
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Could not find active dispatch state for [%s] despite receiving an "
          "acknowledgment for its bid award. This indicates a bug. Please "
          "report this to the RMF developers.",
          command.task_id.c_str());
        return;
      }

      const auto state = state_it->second;
      if (state->status == DispatchState::Status::Selected)
      {
        state->status = DispatchState::Status::Dispatched;
        move_to_finished(state->task_id);
      }
      else if (state->status == DispatchState::Status::CanceledInFlight)
      {
        // If the task was canceled in flight then we will simply ignore this
        // acknowledgment. There should be another lingering command telling the
        // fleet adapter to cancel the task.
      }
      else
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Dispatch status for [%s] is [%u], but we have received an award "
          "acknowledgment for it. This indicates a bug. Please report this to "
          "the RMF developers.",
          state->task_id.c_str(),
          static_cast<uint8_t>(state->status));
      }

      auctioneer->ready_for_next_bid();
      return;
    }
    else if (command.type == DispatchCommandMsg::TYPE_REMOVE)
    {
      // No further action is needed. We simply remove the lingering command.
    }

    RCLCPP_ERROR(
      node->get_logger(),
      "Unrecognized command type [%u] in lingering dispatch command queue. "
      "This indicates a bug. Please report this to the RMF developers.",
      command.type);
  }
};

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::init_and_make_node(
  const std::string dispatcher_node_name)
{
  rclcpp::init(0, nullptr);
  return make_node(dispatcher_node_name);
}

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::make_node(
  const std::string dispatcher_node_name)
{
  return make(rclcpp::Node::make_shared(dispatcher_node_name));
}

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::make(
  const std::shared_ptr<rclcpp::Node>& node)
{
  auto dispatcher = std::shared_ptr<Dispatcher>(new Dispatcher());
  dispatcher->_pimpl = rmf_utils::make_impl<Implementation>(node);
  return dispatcher;
}

//==============================================================================
std::optional<TaskID> Dispatcher::submit_task(
  const rmf_task_msgs::msg::TaskDescription& task_description)
{
  return _pimpl->submit_task(task_description);
}

//==============================================================================
bool Dispatcher::cancel_task(const TaskID& task_id)
{
  return _pimpl->cancel_task(task_id);
}

//==============================================================================
std::optional<DispatchState> Dispatcher::get_dispatch_state(
  const TaskID& task_id) const
{
  const auto active_it = _pimpl->active_dispatch_states.find(task_id);
  if (active_it != _pimpl->active_dispatch_states.end())
    return *active_it->second;

  const auto finished_it = _pimpl->finished_dispatch_states.find(task_id);
  if (finished_it != _pimpl->finished_dispatch_states.end())
    return *finished_it->second;

  return std::nullopt;
}

//==============================================================================
const Dispatcher::DispatchStates& Dispatcher::active_dispatches() const
{
  return _pimpl->active_dispatch_states;
}

//==============================================================================
const Dispatcher::DispatchStates& Dispatcher::finished_dispatches() const
{
  return _pimpl->finished_dispatch_states;
}

//==============================================================================
void Dispatcher::on_change(DispatchStateCallback on_change_fn)
{
  _pimpl->on_change_fn = on_change_fn;
}

//==============================================================================
void Dispatcher::evaluator(bidding::Auctioneer::ConstEvaluatorPtr evaluator)
{
  _pimpl->auctioneer->set_evaluator(std::move(evaluator));
}

//==============================================================================
std::shared_ptr<rclcpp::Node> Dispatcher::node()
{
  return _pimpl->node;
}

//==============================================================================
void Dispatcher::spin()
{
  rclcpp::ExecutorOptions options;
  options.context = _pimpl->node->get_node_options().context();
  rclcpp::executors::SingleThreadedExecutor executor(options);
  executor.add_node(_pimpl->node);
  executor.spin();
}

//==============================================================================
Dispatcher::Dispatcher()
{
  // Do Nothing
}

} // namespace rmf_task_ros2
