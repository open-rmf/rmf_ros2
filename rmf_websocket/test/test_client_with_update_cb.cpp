#include <ctime>
#define CATCH_CONFIG_MAIN
#include <rmf_utils/catch.hpp>

#include <atomic>
#include <string>
#include <thread>

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
std::atomic<int> num_init_msgs = 0;
std::promise<void> on_init_promise;
std::promise<void> on_server_down;

std::vector<nlohmann::json> msgs;
// Define a handler for incoming messages
void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg)
{
  auto json = nlohmann::json::parse(msg->get_payload());
  if (json["test"] == "init")
  {
    num_init_msgs++;
    return;
  }
  num_msgs++;
  msgs.push_back(json);
  terminate_server = true;
  s->close(hdl, websocketpp::close::status::going_away, "Bye!");
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
  auto timer = echo_server.set_timer(20.0, [](auto /*?*/)
      {
        terminate_server = true;
        timed_out = true;
      });

  // Run the server loop
  while (!terminate_server)
  {
    echo_server.run_one();
  }

  echo_server.stop_listening();
  timer->cancel();
  on_server_down.set_value();
}


std::vector<nlohmann::json> init_function()
{
  nlohmann::json json;
  json["test"] = "init";
  std::vector<nlohmann::json> msgs;
  msgs.push_back(json);
  on_init_promise.set_value();
  return msgs;
}

TEST_CASE("Client", "Reconnecting server") {
  using namespace std::chrono_literals;

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
  REQUIRE_NOTHROW(on_init_promise.get_future().wait_for(5s));

  t1.join();

  REQUIRE_NOTHROW(on_server_down.get_future().wait_for(5s));
  on_server_down = std::promise<void>();
  on_init_promise = std::promise<void>();

  auto t2 = std::thread([]()
      {
        run_server();
      });

  REQUIRE(num_msgs == 1);
  REQUIRE(num_init_msgs == 1);

  terminate_server = false;
  jsonString["test"] = "2";
  broadcaster->publish(jsonString);


  t2.join();


  REQUIRE_NOTHROW(on_init_promise.get_future().wait_for(5s));

  REQUIRE(num_msgs == 2);

  REQUIRE(msgs[0]["test"] == "1");
  REQUIRE(msgs[1]["test"] == "2");
  REQUIRE(num_init_msgs >= 2);
}
