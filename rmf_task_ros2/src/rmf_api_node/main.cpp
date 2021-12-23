/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rclcpp/node.hpp>

#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include <rmf_task_msgs/srv/api_service.hpp>

#include <nlohmann/json.hpp>

#include <unordered_map>

namespace rmf_task_ros2 {

class ApiNode : public rclcpp::Node
{
public:

  using Request = rmf_task_msgs::msg::ApiRequest;
  using Response = rmf_task_msgs::msg::ApiResponse;
  using Service = rmf_task_msgs::srv::ApiService;

  static std::shared_ptr<ApiNode> make()
  {
    auto node = std::make_shared<ApiNode>(ApiNode());

    node->_request_pub = node->create_publisher<Request>(
      "api_request", rclcpp::QoS(10).reliable().transient_local());

    // TODO(MXG): Subscribe to parameter changes so we can update the behavior
    // automatically when users change the parameters during runtime.
    node->declare_parameter<double>("timeout_seconds", 2.0);

    auto service_cb =
      [w = std::weak_ptr<ApiNode>(node)](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const Service::Request::SharedPtr request)
      {
        const auto self = w.lock();
        if (!self)
          return;

        const auto request_id = std::to_string(self->_counter++);
        const auto it = self->_inbox.insert(
          std::make_pair(
            request_id,
            Inbox{
              request_id,
              request->json_msg,
              request_header,
              std::chrono::steady_clock::now() + self->_get_timeout()
            }));

        self->_publish_request(it.first->second);
      };

    node->_service = node->create_service<Service>(
      "api_service", std::move(service_cb));

    auto subscribe_cb =
      [w = std::weak_ptr<ApiNode>(node)](const Response::SharedPtr response)
      {
        if (const auto self = w.lock())
          self->_handle_response(response);
      };

    node->_response_sub = node->create_subscription<Response>(
      "api_response", rclcpp::QoS(10).reliable().transient_local(),
      std::move(subscribe_cb));

    node->_timeout = node->create_wall_timer(
      std::chrono::milliseconds(200),
      [w = std::weak_ptr<ApiNode>(node)]()
      {
        if (const auto self = w.lock())
          self->_check_timeouts();
      });

    return node;
  }

private:

  enum ErrorCode
  {
    Timeout = 1,
    UninitializedResponse
  };

  static std::unordered_map<uint64_t, std::string> _error_categories;

  /// This keeps track of request headers and when the requests will timeout
  struct Inbox
  {
    std::string request_id;
    std::string json_msg;
    std::shared_ptr<rmw_request_id_t> header;
    std::chrono::steady_clock::time_point timeout;
    bool acknowledged = false;
  };
  using InboxMap = std::unordered_map<std::string, Inbox>;

  std::chrono::nanoseconds _get_timeout() const
  {
    double timeout_sec = 2.0;
    get_parameter<double>("timeout_seconds", timeout_sec);
    using Sec64 = std::chrono::duration<double>;
    using Dur = std::chrono::nanoseconds;
    return std::chrono::duration_cast<Dur>(Sec64(timeout_sec));
  }

  void _publish_request(const Inbox& request) const
  {
    _request_pub->publish(
      rmf_task_msgs::build<Request>()
          .json_msg(request.json_msg)
          .request_id(request.request_id));
  }

  void _handle_response(const Response::SharedPtr response)
  {
    const auto it = _inbox.find(response->request_id);
    if (it == _inbox.end())
    {
      if (response->type == Response::TYPE_ACKNOWLEDGE)
      {
        // We'll just ignore acknowledgments for missing requests. Maybe they're
        // just arriving late in our processing queue.
        return;
      }

      if (response->type == Response::TYPE_UNINITIALIZED)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Received uninitialized response type for request [%s] which is not "
          "expecting any response.",
          response->request_id.c_str());
        return;
      }

      if (response->type == Response::TYPE_RESPONDING)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Received a response for a request [%s] which is not expecting a "
          "response",
          response->request_id.c_str());
        return;
      }

      RCLCPP_ERROR(
        get_logger(),
        "Received unknown response type [%d] for a request [%s] which is not "
        "expecting any response",
        response->type,
        response->request_id.c_str());

      return;
    }

    if (response->type == Response::TYPE_UNINITIALIZED)
    {
      return _respond(it, _make_error_response(UninitializedResponse, ""));
    }

    if (response->type == Response::TYPE_RESPONDING)
    {
      return _respond(it, response->json_msg);
    }

    if (response->type == Response::TYPE_ACKNOWLEDGE)
    {
      auto& inbox = it->second;
      inbox.acknowledged = true;
      inbox.timeout = std::chrono::steady_clock::now() + _get_timeout();
      return;
    }

    return _respond(
      it, _make_error_response(
        UninitializedResponse,
        "Unknown response type: " + std::to_string(response->type)));
  }

  static nlohmann::json _make_error_response(
    ErrorCode code,
    std::string detail)
  {
    nlohmann::json response_json;
    std::vector<nlohmann::json> errors;
    nlohmann::json error;
    error["code"] = code;
    error["category"] = _error_categories[code];
    error["detail"] = std::move(detail);
    errors.push_back(std::move(error));

    response_json["success"] = false;
    response_json["errors"] = std::move(errors);

    return response_json;
  }

  void _respond(
    InboxMap::iterator it,
    std::string json_msg)
  {
    auto response = rmf_task_msgs::build<Service::Response>()
      .json_msg(std::move(json_msg));

    _service->send_response(*it->second.header, response);

    _inbox.erase(it);
  }

  void _check_timeouts()
  {
    std::vector<std::string> timed_out;
    for (const auto& [_, check] : _inbox)
    {
      if (check.timeout < std::chrono::steady_clock::now())
      {
        timed_out.push_back(check.request_id);
        continue;
      }

      if (!check.acknowledged)
      {
        _publish_request(check);
      }
    }

    for (const auto& id : timed_out)
    {
      const auto it = _inbox.find(id);
      if (it == _inbox.end())
      {
        throw std::runtime_error(
          "[ApiNode::_check_timeouts] Impossible situation: map entry is "
          "missing for [" + id + "]");
      }

      std::string detail;
      if (it->second.acknowledged)
        detail = "Request was acknowledged but the response timed out";
      else
        detail = "Request was never acknowledged";

      _respond(it, _make_error_response(Timeout, std::move(detail)));
    }
  }

  ApiNode() : rclcpp::Node("rmf_api_node")
  {
    // Do nothing
  }

  rclcpp::Publisher<Request>::SharedPtr _request_pub;
  rclcpp::Subscription<Response>::SharedPtr _response_sub;
  rclcpp::Service<Service>::SharedPtr _service;

  InboxMap _inbox;

  rclcpp::TimerBase::SharedPtr _timeout;
  std::size_t _counter = 0;
};

std::unordered_map<uint64_t, std::string> ApiNode::_error_categories  =
{
  {Timeout, "Timeout"},
  {UninitializedResponse, "Uninitialized Response"}
};

} // namespace rmf_task_ros2

int main()
{

}
