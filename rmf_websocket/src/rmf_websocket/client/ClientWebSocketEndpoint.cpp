#include "ClientWebSocketEndpoint.hpp"
#include <chrono>
#include <mutex>
#include <sstream>

using namespace std::chrono_literals;
using namespace rmf_websocket;


//=============================================================================
ConnectionMetadata::ConnectionMetadata(
  websocketpp::connection_hdl hdl, std::string uri)
: _hdl(hdl)
  , _status(ConnectionStatus::CONNECTING)
  , _uri(uri)
  , _server("N/A")
{
}

//=============================================================================
void ConnectionMetadata::on_open(WsClient* c, websocketpp::connection_hdl hdl)
{
  {
    std::lock_guard<std::mutex> lock(_status_mtx);
    _status = ConnectionStatus::OPEN;
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
    _status = ConnectionStatus::FAILED;

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
  _status = ConnectionStatus::CLOSED;
  WsClient::connection_ptr con = c->get_con_from_hdl(hdl);
  std::stringstream s;
  s << "close code: " << con->get_remote_close_code() << " ("
    << websocketpp::close::status::get_string(con->get_remote_close_code())
    << "), close reason: " << con->get_remote_close_reason();
  _error_reason = s.str();
  _cv.notify_all();
}

//=============================================================================
std::string ConnectionMetadata::debug_data()
{
  std::stringstream out;
  std::string status_string;
  {
    std::lock_guard<std::mutex> lock(_status_mtx);
    switch (_status) 
    {
      case ConnectionStatus::CONNECTING:
        status_string = "Connecting";
        break;
      case ConnectionStatus::OPEN:
        status_string = "Open";
        break;
      case ConnectionStatus::CLOSED:
        status_string = "Closed";
        break;
      case ConnectionStatus::FAILED:
        status_string = "Closed";
        break;
    }
  }

  out << "> URI: " << _uri << "\n"
      << "> Status: " << status_string << "\n"
      << "> Remote Server: "
      << (_server.empty() ? "None Specified" : _server) << "\n"
      << "> Error/close reason: "
      << (_error_reason.empty() ? "N/A" : _error_reason) << "\n";

  return out.str();
}

//=============================================================================
ConnectionMetadata::ConnectionStatus ConnectionMetadata::get_status()
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
      return _status != ConnectionStatus::CONNECTING;
    }))
    return _status == ConnectionStatus::OPEN;

  return false;
}

//=============================================================================
ClientWebSocketEndpoint::ClientWebSocketEndpoint(
  std::string const& uri, Logger logger)
: _uri(uri), _stop(false), _logger(logger)
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
    std::stringstream err;
    err << "> Connect initialization error: " << ec.message();
    _logger(err.str());
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
    std::stringstream out;
    out << "> Error sending message: " << ec.message();
    _logger(out.str());
    return;
  }

}

//=============================================================================
ClientWebSocketEndpoint::~ClientWebSocketEndpoint()
{
  _endpoint.stop_perpetual();


  if (_current_connection->get_status() !=
    ConnectionMetadata::ConnectionStatus::OPEN)
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
    std::stringstream err;
    err << "> Error closing connection : " << ec.message();
    _logger(err.str());
  }

  _thread->join();

  /// Wait for consumer thread to finish
}

//=============================================================================
void ClientWebSocketEndpoint::wait_for_ready()
{
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