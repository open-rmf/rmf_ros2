#include "ClientWebSocketEndpoint.hpp"
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;
using namespace rmf_websocket;


//=============================================================================
ConnectionMetadata::ConnectionMetadata(
  websocketpp::connection_hdl hdl, std::string uri)
: _hdl(hdl)
  , _status("Connecting")
  , _uri(uri)
  , _server("N/A")
{
}

//=============================================================================
void ConnectionMetadata::on_open(WsClient* c, websocketpp::connection_hdl hdl)
{
  {
    std::lock_guard<std::mutex> lock(_status_mtx);
    _status = "Open";
    WsClient::connection_ptr con = c->get_con_from_hdl(hdl);
    _server = con->get_response_header("Server");
  }
  _cv.notify_all();
}

//=============================================================================
void ConnectionMetadata::on_fail(WsClient* c, websocketpp::connection_hdl hdl)
{
  {
    std::lock_guard<std::mutex> lock(_status_mtx);
    _status = "Failed";

    WsClient::connection_ptr con = c->get_con_from_hdl(hdl);
    _server = con->get_response_header("Server");
    _error_reason = con->get_ec().message();
  }
  _cv.notify_all();
}

//=============================================================================
void ConnectionMetadata::on_close(WsClient* c, websocketpp::connection_hdl hdl)
{
  std::lock_guard<std::mutex> lock(_status_mtx);
  _status = "Closed";
  WsClient::connection_ptr con = c->get_con_from_hdl(hdl);
  std::stringstream s;
  s << "close code: " << con->get_remote_close_code() << " ("
    << websocketpp::close::status::get_string(con->get_remote_close_code())
    << "), close reason: " << con->get_remote_close_reason();
  _error_reason = s.str();
}


//=============================================================================
std::string ConnectionMetadata::get_status()
{
  std::lock_guard<std::mutex> lock(_status_mtx);
  return _status;
}

//=============================================================================
websocketpp::connection_hdl ConnectionMetadata::get_hdl() const
{
  return _hdl;
}

//=============================================================================
bool ConnectionMetadata::wait_for_ready(const long dur)
{
  std::unique_lock<std::mutex> lk(_status_mtx);
  if (_cv.wait_for(lk, dur * 1ms, [&]
    {
      return _status != "Connecting";
    }))
    return _status == "Open";
  else
    std::cerr << " timed out trying to connect " << '\n';
  return false;
}

//=============================================================================
ClientWebSocketEndpoint::ClientWebSocketEndpoint(std::string const& uri)
: _uri(uri), _stop(false)
{
  _endpoint.clear_access_channels(websocketpp::log::alevel::all);
  _endpoint.clear_error_channels(websocketpp::log::elevel::all);

  _endpoint.init_asio();
  _endpoint.start_perpetual();

  _thread.reset(new websocketpp::lib::thread(&WsClient::run, &_endpoint));
}

//=============================================================================
int ClientWebSocketEndpoint::connect()
{
  websocketpp::lib::error_code ec;

  WsClient::connection_ptr con = _endpoint.get_connection(_uri, ec);

  if (ec)
  {
    std::cout << "> Connect initialization error: " << ec.message() <<
      std::endl;
    return -1;
  }

  ConnectionMetadata::ptr metadata_ptr(new ConnectionMetadata(con->get_handle(),
    _uri));
  _current_connection = metadata_ptr;

  con->set_open_handler(websocketpp::lib::bind(
      &ConnectionMetadata::on_open,
      metadata_ptr,
      &_endpoint,
      websocketpp::lib::placeholders::_1
  ));
  con->set_fail_handler(websocketpp::lib::bind(
      &ConnectionMetadata::on_fail,
      metadata_ptr,
      &_endpoint,
      websocketpp::lib::placeholders::_1
  ));

  con->set_close_handler(websocketpp::lib::bind(
      &ConnectionMetadata::on_close,
      metadata_ptr,
      &_endpoint,
      websocketpp::lib::placeholders::_1
  ));

  _endpoint.connect(con);


  return 0;
}

//=============================================================================
ConnectionMetadata::ptr ClientWebSocketEndpoint::get_metadata() const
{
  return _current_connection;
}

//=============================================================================
void ClientWebSocketEndpoint::send(std::string message)
{
  websocketpp::lib::error_code ec;

  _endpoint.send(
    _current_connection->get_hdl(), message, websocketpp::frame::opcode::text,
    ec);
  if (ec)
  {
    std::cout << "> Error sending message: " << ec.message() << std::endl;
    return;
  }

}

//=============================================================================
ClientWebSocketEndpoint::~ClientWebSocketEndpoint()
{
  _endpoint.stop_perpetual();


  if (_current_connection->get_status() != "Open")
  {
    // Only close open connections
    return;
  }


  websocketpp::lib::error_code ec;
  _endpoint.close(
    _current_connection->get_hdl(), websocketpp::close::status::going_away, "",
    ec);
  if (ec)
  {
    std::cout << "> Error closing connection : "
              << ec.message() << std::endl;
  }

  _thread->join();

  /// Wait for consumer thread to finish
  std::lock_guard<std::mutex> lock(_mtx);
}

//=============================================================================
void ClientWebSocketEndpoint::wait_for_ready()
{
  // Makes sure only one thread does this.
  std::lock_guard<std::mutex> lock(_mtx);

  while (!_current_connection->wait_for_ready(1000))
  {
    //std::cout << "Could not connect... Trying again later" <<std::endl;
    std::this_thread::sleep_for(1000ms);
    if (_stop.load())
    {
      return;
    }
    connect();
  }
}

//=============================================================================
void ClientWebSocketEndpoint::interrupt_waits()
{
  _stop = true;
}