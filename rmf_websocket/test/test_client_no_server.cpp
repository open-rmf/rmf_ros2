#include <iostream>
#define CATCH_CONFIG_MAIN
#include <rmf_utils/catch.hpp>

#include <atomic>
#include <thread>
#include <websocketpp/config/asio.hpp>
#include <websocketpp/server.hpp>
#include <rmf_websocket/BroadcastClient.hpp>
#include <string>
#include <rclcpp/node.hpp>

TEST_CASE("Client", "Reconnecting server") {
  rclcpp::init(0, {});
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  auto broadcaster = rmf_websocket::BroadcastClient::make(
    "ws://localhost:9000/", test_node);
  nlohmann::json jsonString;
  jsonString["test"] = "1";
  broadcaster->publish(jsonString);
}
