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

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr message_ptr;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;


std::atomic_bool terminate_server = false;
std::atomic<int> num_msgs = 0;
std::atomic<int> num_init_msgs = 0;
std::vector<nlohmann::json> msgs;
// Define a handler for incoming messages
void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg)
{
  auto json = nlohmann::json::parse(msg->get_payload());
  if (json["test"] == "init")
  {
    std::cout << "init message" <<"\n";
    num_init_msgs++;
    return;
  }
  std::cout << "normal_message" <<"\n";
  num_msgs++;
  msgs.push_back(json);
  terminate_server = true;
}


void run_server()
{
  server echo_server;

  echo_server.set_access_channels(websocketpp::log::alevel::all);
  echo_server.clear_access_channels(websocketpp::log::alevel::frame_payload);
  echo_server.init_asio();
  echo_server.set_reuse_addr(true);
  // Set on_message handler
  echo_server.set_message_handler(
    bind(&on_message, &echo_server, ::_1, ::_2));


  // Set the port number
  echo_server.listen(9000);

  // Start the server asynchronously
  echo_server.start_accept();

  // Hack to prevent test deadlock
  echo_server.set_timer(20.0, [](auto /*?*/)
  {
    terminate_server = true;
  });

  // Run the server loop
  while (!terminate_server)
  {
    echo_server.run_one();
  }

  echo_server.stop_listening();
}

std::vector<nlohmann::json> init_function()
{
  nlohmann::json json;
  json["test"] = "init";
  std::vector<nlohmann::json> msgs;
  msgs.push_back(json);
  return msgs;
}

TEST_CASE("Client", "Reconnecting server") {
  rclcpp::init(0, {});
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  auto t1 = std::thread([]()
      {
        run_server();
      });
  auto broadcaster = rmf_websocket::BroadcastClient::make(
    "ws://localhost:9000/", test_node, init_function);

  nlohmann::json jsonString;
  jsonString["test"] = "1";
  broadcaster->publish(jsonString);
  t1.join();

  REQUIRE(num_msgs == 1);
  REQUIRE(num_init_msgs == 1);

  terminate_server = false;
  jsonString["test"] = "2";
  broadcaster->publish(jsonString);

  auto t2 = std::thread([]()
      {
        run_server();
      });
  t2.join();
  REQUIRE(num_msgs == 2);

  REQUIRE(msgs[0]["test"] == "1");
  REQUIRE(msgs[1]["test"] == "2");
  REQUIRE(num_init_msgs == 2);
}
