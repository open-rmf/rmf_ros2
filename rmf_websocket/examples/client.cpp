#include <rmf_websocket/BroadcastClient.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace rmf_websocket;
using namespace std::chrono_literals;

std::vector<nlohmann::json> new_connection_data()
{
  std::vector<nlohmann::json> msgs;
  nlohmann::json json;
  json["hi"] = "Hello";
  msgs.push_back(json);
  return msgs;
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("web_socket_test_node"), count_(0)
  {
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (!client_)
    {
      client_ = BroadcastClient::make(
        "ws://127.0.0.1:8000/",
        shared_from_this(),
        &new_connection_data
      );
    }
    obj["count"] = count_++;
    client_->publish(obj);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  nlohmann::json obj {{"Otototo", true}};
  std::size_t count_ = 0;
  std::shared_ptr<BroadcastClient> client_ = nullptr;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
