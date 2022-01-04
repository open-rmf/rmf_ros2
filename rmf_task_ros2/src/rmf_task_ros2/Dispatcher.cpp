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

#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "action/Client.hpp"

#include <rmf_task_msgs/msg/task_description.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>
#include <rmf_task_msgs/srv/submit_task.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <rmf_task_msgs/srv/get_task_list.hpp>
#include <rmf_task_msgs/msg/tasks.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include <nlohmann/json.hpp>

namespace rmf_task_ros2 {

//==============================================================================
class Dispatcher::Implementation
{
public:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<bidding::Auctioneer> auctioneer;
  std::shared_ptr<action::Client> action_client;

  using SubmitTaskSrv = rmf_task_msgs::srv::SubmitTask;
  using CancelTaskSrv = rmf_task_msgs::srv::CancelTask;
  using GetTaskListSrv = rmf_task_msgs::srv::GetTaskList;
  using TasksMsg = rmf_task_msgs::msg::Tasks;
  using TaskDescription = rmf_task_msgs::msg::TaskDescription;

  rclcpp::Service<SubmitTaskSrv>::SharedPtr submit_task_srv;
  rclcpp::Service<CancelTaskSrv>::SharedPtr cancel_task_srv;
  rclcpp::Service<GetTaskListSrv>::SharedPtr get_task_list_srv;

  using ApiRequest = rmf_task_msgs::msg::ApiRequest;
  using ApiResponse = rmf_task_msgs::msg::ApiResponse;
  rclcpp::Subscription<ApiRequest>::SharedPtr api_request;
  rclcpp::Publisher<ApiResponse>::SharedPtr api_response;

  using ActiveTasksPub = rclcpp::Publisher<TasksMsg>;
  ActiveTasksPub::SharedPtr ongoing_tasks_pub;

  rclcpp::TimerBase::SharedPtr timer;

  DispatchStateCallback on_change_fn;

  std::queue<bidding::BidNotice> queue_bidding_tasks;

  /// TODO: should rename "active" to "ongoing" to prevent confusion
  /// of with task STATE_ACTIVE
  DispatchTasks active_dispatch_tasks;
  DispatchTasks terminal_dispatch_tasks;
  std::set<std::string> user_submitted_tasks;  // ongoing submitted task_ids
  std::size_t task_counter = 0; // index for generating task_id
  double bidding_time_window;
  int terminated_tasks_max_size;
  int publish_active_tasks_period;

  std::unordered_map<std::size_t, std::string> legacy_task_type_names =
  {
    {1, "patrol"},
    {2, "delivery"},
    {4, "patrol"}
  };

  using LegacyConversion = std::function<std::string(const TaskDescription&)>;
  using LegacyConversionMap = std::unordered_map<std::string, LegacyConversion>;
  LegacyConversionMap legacy_task_types;

  Implementation(std::shared_ptr<rclcpp::Node> node_)
  : node{std::move(node_)}
  {
    // ros2 param
    bidding_time_window =
      node->declare_parameter<double>("bidding_time_window", 2.0);
    RCLCPP_INFO(node->get_logger(),
      " Declared Time Window Param as: %f secs", bidding_time_window);
    terminated_tasks_max_size =
      node->declare_parameter<int>("terminated_tasks_max_size", 100);
    RCLCPP_INFO(node->get_logger(),
      " Declared Terminated Tasks Max Size Param as: %d",
      terminated_tasks_max_size);
    publish_active_tasks_period =
      node->declare_parameter<int>("publish_active_tasks_period", 2);
    RCLCPP_INFO(node->get_logger(),
      " Declared publish_active_tasks_period as: %d secs",
      publish_active_tasks_period);

    const auto qos = rclcpp::ServicesQoS().reliable();
    ongoing_tasks_pub = node->create_publisher<TasksMsg>(
      rmf_task_ros2::ActiveTasksTopicName, qos);

    timer = node->create_wall_timer(
      std::chrono::seconds(publish_active_tasks_period),
      std::bind(
        &Dispatcher::Implementation::publish_ongoing_tasks, this));

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

    get_task_list_srv = node->create_service<GetTaskListSrv>(
      rmf_task_ros2::GetTaskListSrvName,
      [this](
        const std::shared_ptr<GetTaskListSrv::Request> request,
        std::shared_ptr<GetTaskListSrv::Response> response)
      {
        for (auto task : (this->active_dispatch_tasks))
        {
          response->active_tasks.push_back(
            rmf_task_ros2::convert_status(*(task.second)));
        }

        // Terminated Tasks
        for (auto task : (this->terminal_dispatch_tasks))
        {
          response->terminated_tasks.push_back(
            rmf_task_ros2::convert_status(*(task.second)));
        }
        response->success = true;
      }
    );

    // Loop
    legacy_task_types["patrol"] =
      [](const TaskDescription& task_description) -> std::string
      {
        const auto& loop = task_description.loop;
        nlohmann::json description;
        std::vector<std::string> places;
        places.push_back(loop.start_name);
        places.push_back(loop.finish_name);
        description["places"] = std::move(places);
        description["rounds"] = loop.num_loops;

        return description.dump();
      };

    // Delivery
    legacy_task_types["delivery"] =
      [](const TaskDescription& task_description) -> std::string
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

        return description.dump();
      };

    // Clean
    legacy_task_types["clean"] =
      [](const TaskDescription& task_description) -> std::string
      {
        const auto& clean = task_description.clean;
        nlohmann::json description;
        description["zone"] = clean.start_waypoint;

        return description.dump();
      };
  }

  void start()
  {
    using namespace std::placeholders;
    auctioneer = bidding::Auctioneer::make(node,
        std::bind(&Implementation::receive_bidding_winner_cb, this, _1, _2));
    action_client->on_terminate(
      std::bind(&Implementation::terminate_task, this, _1));
    action_client->on_change(
      std::bind(&Implementation::task_status_cb, this, _1));
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
    const auto task_id = "dispatch#" + std::to_string(task_counter++);

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

    bidding::BidNotice bid_notice;
    bid_notice.request = task_request.dump();
    bid_notice.time_window = rmf_traffic_ros2::convert(
      rmf_traffic::time::from_seconds(bidding_time_window));
    push_bid_notice(std::move(bid_notice));

    return task_id;
  }

  void push_bid_notice(bidding::BidNotice bid_notice)
  {
    nlohmann::json state;
    auto& booking = state["booking"];
    booking["id"] = bid_notice.task_id;

    const auto request = nlohmann::json::parse(bid_notice.request);
    static const std::vector<std::string> copy_fields = {
        "unix_millis_earliest_start_time",
        "priority",
        "labels"
      };

    for (const auto& field : copy_fields)
      booking[field] = request.at(field);

    state["category"] = request["category"];
    state["detail"] = request["description"];

    if (on_change_fn)
      on_change_fn(state);

    // add task to internal cache
    active_dispatch_tasks[bid_notice.task_id] = new_task_status;
    user_submitted_tasks.insert(submitted_task.task_id);

    queue_bidding_tasks.push(bid_notice);

    if (queue_bidding_tasks.size() == 1)
      auctioneer->start_bidding(queue_bidding_tasks.front());
  }

  bool cancel_task(const TaskID& task_id)
  {
    // check if key exists
    const auto it = active_dispatch_tasks.find(task_id);
    if (it == active_dispatch_tasks.end())
    {
      RCLCPP_ERROR(node->get_logger(),
        "Task [%s] is not found in active_tasks", task_id.c_str());
      return false;
    }

    RCLCPP_WARN(node->get_logger(), "Cancel task: [%s]", task_id.c_str());

    // Cancel bidding. This will remove the bidding process
    const auto& cancel_task_status = it->second;
    if (cancel_task_status->state == DispatchState::State::Pending)
    {
      cancel_task_status->state = DispatchState::State::Canceled;
      terminate_task(cancel_task_status);

      if (on_change_fn)
        on_change_fn(cancel_task_status);

      return true;
    }

    // only user submitted task is cancelable
    if (user_submitted_tasks.find(task_id) == user_submitted_tasks.end())
    {
      RCLCPP_ERROR(node->get_logger(),
        "only user submitted task is cancelable");
      return false;
    }

    // Curently cancel can only work on Queued Task in Fleet Adapter
    if (cancel_task_status->state != DispatchState::State::Queued)
    {
      RCLCPP_ERROR(node->get_logger(),
        "Unable to cancel task [%s] as it is not a Queued Task",
        task_id.c_str());
      return false;
    }

    // Remove non-user submitted task from "active_dispatch_tasks"
    // this is to prevent duplicated task during reassignation
    // TODO: a better way to impl this
    for (auto it = active_dispatch_tasks.begin();
      it != active_dispatch_tasks.end(); )
    {
      const bool is_fleet_name =
        (cancel_task_status->fleet_name == it->second->fleet_name);
      const bool is_self_gererated =
        (user_submitted_tasks.find(it->first) == user_submitted_tasks.end());

      if (is_self_gererated && is_fleet_name)
      {
        it->second->state = DispatchState::State::Canceled;
        terminate_task((it++)->second);
      }
      else
        ++it;
    }

    // Cancel action task, this will only send a cancel to FA. up to
    // the FA whether to cancel the task. On change is implemented
    // internally in action client
    return action_client->cancel_task(cancel_task_status->task_profile);
  }

  const std::optional<DispatchState::State> get_task_state(
    const TaskID& task_id) const
  {
    // check if taskid exists in active tasks
    auto it = active_dispatch_tasks.find(task_id);
    if (it != active_dispatch_tasks.end())
      return it->second->state;

    // check if taskid exists in terminated tasks
    it = terminal_dispatch_tasks.find(task_id);
    if (it != terminal_dispatch_tasks.end())
      return it->second->state;

    return std::nullopt;
  }

  void receive_bidding_winner_cb(
    const TaskID& task_id,
    const rmf_utils::optional<bidding::Submission> winner)
  {
    const auto it = active_dispatch_tasks.find(task_id);
    if (it == active_dispatch_tasks.end())
      return;
    const auto& pending_task_status = it->second;

    if (!winner)
    {
      RCLCPP_WARN(node->get_logger(), "Dispatcher Bidding Result: task [%s]"
        " has no submissions during bidding, Task Failed", task_id.c_str());
      pending_task_status->state = DispatchState::State::Failed;
      terminate_task(pending_task_status);

      if (on_change_fn)
        on_change_fn(pending_task_status);

      queue_bidding_tasks.pop();
      if (!queue_bidding_tasks.empty())
        auctioneer->start_bidding(queue_bidding_tasks.front());
      return;
    }

    // now we know which fleet will execute the task
    pending_task_status->fleet_name = winner->fleet_name;

    RCLCPP_INFO(node->get_logger(), "Dispatcher Bidding Result: task [%s]"
      " is accepted by fleet adapter [%s]",
      task_id.c_str(), winner->fleet_name.c_str());

    // Remove non-user submitted charging task from "active_dispatch_tasks"
    // this is to prevent duplicated task during reassignation.
    // TODO: a better way to impl this
    for (auto it = active_dispatch_tasks.begin();
      it != active_dispatch_tasks.end(); )
    {
      const bool is_fleet_name = (winner->fleet_name == it->second->fleet_name);
      const bool is_self_gererated =
        (user_submitted_tasks.find(it->first) == user_submitted_tasks.end());

      if (is_self_gererated && is_fleet_name)
      {
        it->second->state = DispatchState::State::Canceled;
        terminate_task((it++)->second);
      }
      else
        ++it;
    }

    // add task to action server
    action_client->add_task(
      winner->fleet_name,
      pending_task_status->task_profile,
      pending_task_status);
  }

  void terminate_task(const TaskStatusPtr terminate_status)
  {
    assert(terminate_status->is_terminated());
    publish_ongoing_tasks();

    // prevent terminal_dispatch_tasks from piling up meaning
    if (terminal_dispatch_tasks.size() >= terminated_tasks_max_size)
    {
      RCLCPP_WARN(node->get_logger(),
        "Terminated tasks reached max size, remove earliest submited task");

      auto rm_task = terminal_dispatch_tasks.begin();
      for (auto it = rm_task++; it != terminal_dispatch_tasks.end(); it++)
      {
        const auto t1 = it->second->task_profile.submission_time;
        const auto t2 = rm_task->second->task_profile.submission_time;
        if (rmf_traffic_ros2::convert(t1) < rmf_traffic_ros2::convert(t2))
          rm_task = it;
      }
      terminal_dispatch_tasks.erase(rm_task);
    }

    const auto id = terminate_status->task_profile.task_id;

    // destroy prev status ptr and recreate one
    auto status = std::make_shared<DispatchState>(*terminate_status);
    (terminal_dispatch_tasks)[id] = status;
    user_submitted_tasks.erase(id);
    active_dispatch_tasks.erase(id);
  }

  void task_status_cb(const TaskStatusPtr status)
  {
    // This is to solve the issue that the dispatcher is not aware of those
    // "stray" tasks that are not dispatched by the dispatcher. This will add
    // the stray tasks when an unknown TaskSummary is heard.
    const std::string id = status->task_profile.task_id;
    const auto it = active_dispatch_tasks.find(id);
    if (it == active_dispatch_tasks.end())
    {
      active_dispatch_tasks[id] = status;
      RCLCPP_WARN(node->get_logger(),
        "Add previously unheard task: [%s]", id.c_str());
    }

    // check if there's a change in state for the previous completed bidding task
    // TODO, better way to impl this
    if (!queue_bidding_tasks.empty()
      && id == queue_bidding_tasks.front().task_profile.task_id)
    {
      queue_bidding_tasks.pop();
      if (!queue_bidding_tasks.empty())
        auctioneer->start_bidding(queue_bidding_tasks.front());
    }

    if (on_change_fn)
      on_change_fn(status);
  }

  void publish_ongoing_tasks()
  {
    TasksMsg task_msgs;
    for (auto task : (this->active_dispatch_tasks))
    {
      task_msgs.tasks.push_back(
        rmf_task_ros2::convert_status(*(task.second)));
    }
    ongoing_tasks_pub->publish(task_msgs);
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
  auto pimpl = rmf_utils::make_impl<Implementation>(node);
  pimpl->action_client = action::Client::make(node);

  auto dispatcher = std::shared_ptr<Dispatcher>(new Dispatcher());
  dispatcher->_pimpl = std::move(pimpl);
  dispatcher->_pimpl->start();
  return dispatcher;
}

//==============================================================================
std::optional<TaskID> Dispatcher::submit_task(
  const TaskDescription& task_description)
{
  return _pimpl->submit_task(task_description);
}

//==============================================================================
bool Dispatcher::cancel_task(const TaskID& task_id)
{
  return _pimpl->cancel_task(task_id);
}

//==============================================================================
const rmf_utils::optional<std::string> Dispatcher::get_task_state(
  const TaskID& task_id) const
{
  return _pimpl->get_task_state(task_id);
}

//==============================================================================
const Dispatcher::DispatchTasks& Dispatcher::active_tasks() const
{
  return _pimpl->active_dispatch_tasks;
}

//==============================================================================
const Dispatcher::DispatchTasks& Dispatcher::terminated_tasks() const
{
  return _pimpl->terminal_dispatch_tasks;
}

//==============================================================================
void Dispatcher::on_change(TaskStateCallback on_change_fn)
{
  _pimpl->on_change_fn = on_change_fn;
}

//==============================================================================
void Dispatcher::evaluator(
  std::shared_ptr<bidding::Auctioneer::Evaluator> evaluator)
{
  _pimpl->auctioneer->select_evaluator(evaluator);
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
