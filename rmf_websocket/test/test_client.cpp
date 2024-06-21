#include <iostream>
#define CATCH_CONFIG_MAIN
#include <rmf_utils/catch.hpp>

#include <atomic>
#include <thread>
#include <string>

#include <rclcpp/node.hpp>

#include <rmf_websocket/BroadcastClient.hpp>

#include <websocketpp/config/asio.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr message_ptr;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;


std::atomic_bool terminate_server = false;
std::atomic_bool timed_out = false;
std::atomic<int> num_msgs = 0;
std::vector<nlohmann::json> msgs;
// Define a handler for incoming messages
void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg)
{
  terminate_server = true;
  num_msgs++;
  msgs.push_back(nlohmann::json::parse(msg->get_payload()));
  server::connection_ptr con = s->get_con_from_hdl(hdl);
  con->close(websocketpp::close::status::normal, "");
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

  // Run the server loop
  while (!terminate_server)
  {
    echo_server.run_one();
  }

  echo_server.stop_listening();
  echo_server.run();
}


TEST_CASE("Client", "Reconnecting server") {
  rclcpp::init(0, {});
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  auto t1 = std::thread([]()
      {
        run_server();
      });
  auto broadcaster = rmf_websocket::BroadcastClient::make(
    "ws://localhost:9000/", test_node);

  nlohmann::json jsonString;
  jsonString["test"] = "1";
  broadcaster->publish(jsonString);
  t1.join();

  std::cout << "Restart server\n";

  REQUIRE(num_msgs == 1);

  terminate_server = false;
  auto t2 = std::thread([]()
      {
        run_server();
      });


  jsonString["test"] = "2";
  auto tries = 0;

  using namespace std::chrono_literals;
  while (tries < 10 && !terminate_server)
  {
    /// TODO(arjoc): Make timeout reconfigureable, that
    /// way we dont need to wait so long.
    broadcaster->publish(jsonString);
    std::this_thread::sleep_for(1000ms);
    ++tries;
  }
  t2.join();

  REQUIRE(num_msgs == 2);

  REQUIRE(msgs[0]["test"] == "1");
  REQUIRE(msgs[1]["test"] == "2");

}
