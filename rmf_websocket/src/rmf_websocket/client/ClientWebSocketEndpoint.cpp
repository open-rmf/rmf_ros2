#include "ClientWebSocketEndpoint.hpp"
#include <chrono>
#include <mutex>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;
using namespace rmf_websocket;
using namespace boost;

//=============================================================================
ConnectionMetadata::ConnectionMetadata(
  websocketpp::connection_hdl hdl, std::string uri, ConnectionCallback cb,
  ReconnectionCallback rcb)
: _hdl(hdl)
  , _status(ConnectionStatus::CONNECTING)
  , _uri(uri)
  , _server("N/A")
  , _connection_cb(cb)
  , _reconnection_cb(std::move(rcb))
{
}

//=============================================================================
void ConnectionMetadata::reset()
{
  _status = ConnectionStatus::CONNECTING;
}

//=============================================================================
void ConnectionMetadata::on_open(WsClient* c, websocketpp::connection_hdl hdl)
{
  _status = ConnectionStatus::OPEN;
  WsClient::connection_ptr con = c->get_con_from_hdl(hdl);
  _server = con->get_response_header("Server");
  _connection_cb();
}

//=============================================================================
void ConnectionMetadata::on_fail(WsClient* c, websocketpp::connection_hdl hdl)
{
  _status = ConnectionStatus::FAILED;

  WsClient::connection_ptr con = c->get_con_from_hdl(hdl);
  _server = con->get_response_header("Server");
  _error_reason = con->get_ec().message();
  c->get_io_service().post(_reconnection_cb);
}

//=============================================================================
void ConnectionMetadata::on_close(WsClient* c, websocketpp::connection_hdl hdl)
{
  _status = ConnectionStatus::CLOSED;
  WsClient::connection_ptr con = c->get_con_from_hdl(hdl);
  std::stringstream s;
  s << "close code: " << con->get_remote_close_code() << " ("
    << websocketpp::close::status::get_string(con->get_remote_close_code())
    << "), close reason: " << con->get_remote_close_reason();
  _error_reason = s.str();
  c->get_io_service().post(_reconnection_cb);
}

//=============================================================================
std::string ConnectionMetadata::debug_data() const
{
  std::stringstream out;
  std::string status_string;

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
    default:
      status_string = "Unknown";
      break;
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
ConnectionMetadata::ConnectionStatus ConnectionMetadata::get_status() const
{
  return _status;
}

//=============================================================================
websocketpp::connection_hdl ConnectionMetadata::get_hdl() const
{
  return _hdl;
}

//=============================================================================
ClientWebSocketEndpoint::ClientWebSocketEndpoint(
  std::string const& uri, Logger logger, asio::io_service* io_service,
  ConnectionCallback cb)
: _uri(uri), _logger(logger), _init{false}, _reconnect_enqueued(false),
  _connection_cb(std::move(cb))
{
  _endpoint.clear_access_channels(websocketpp::log::alevel::all);
  _endpoint.clear_error_channels(websocketpp::log::elevel::all);
  _endpoint.init_asio(io_service);
  _endpoint.start_perpetual();
}

//=============================================================================
websocketpp::lib::error_code ClientWebSocketEndpoint::connect()
{
  websocketpp::lib::error_code ec;


  _init = true;
  _con = _endpoint.get_connection(_uri, ec);

  if (ec)
  {
    std::stringstream err;
    err << "> Connect initialization error: " << ec.message() << std::endl
        << "> Host: " << _uri << std::endl;
    _logger(err.str());
    return ec;
  }

  auto reconnect_socket = [this]()
    {
      // TODO(arjo) Parametrize the timeout.
      using namespace std::chrono_literals;
      std::stringstream err;
      if (!_reconnect_enqueued)
      {
        err << "> Reconnecting in 1s" << std::endl
            << "> Host: " << _uri << std::endl;
        _logger(err.str());
        _endpoint.set_timer(1.0, std::bind(&ClientWebSocketEndpoint::connect,
          this));
        _reconnect_enqueued = true;
      }
    };
  auto connected_cb = [this]()
    {
      _reconnect_enqueued = false;
      _connection_cb();
    };

  // Not sure why but seems like I have to re-initallize this everytime in order
  // to actually make a clean connection. My guess is the shared pointer is being
  // leaked somewhere.
  _current_connection = std::make_shared<ConnectionMetadata>(
    _con->get_handle(),
    _uri, connected_cb, reconnect_socket);

  _con->set_open_handler(websocketpp::lib::bind(
      &ConnectionMetadata::on_open,
      _current_connection,
      &_endpoint,
      websocketpp::lib::placeholders::_1
  ));
  _con->set_fail_handler(websocketpp::lib::bind(
      &ConnectionMetadata::on_fail,
      _current_connection,
      &_endpoint,
      websocketpp::lib::placeholders::_1
  ));

  _con->set_close_handler(websocketpp::lib::bind(
      &ConnectionMetadata::on_close,
      _current_connection,
      &_endpoint,
      websocketpp::lib::placeholders::_1
  ));


  _endpoint.connect(_con);

  return ec;
}

//=============================================================================
std::optional<ConnectionMetadata::ConnectionStatus> ClientWebSocketEndpoint::
get_status() const
{
  if (!_init)
  {
    return std::nullopt;
  }
  return _current_connection->get_status();
}

//=============================================================================
websocketpp::lib::error_code ClientWebSocketEndpoint::send(
  const std::string& message)
{
  websocketpp::lib::error_code ec;

  _endpoint.send(
    _current_connection->get_hdl(), message, websocketpp::frame::opcode::text,
    ec);
  if (ec)
  {
    return ec;
  }
  return ec;
}

//=============================================================================
ClientWebSocketEndpoint::~ClientWebSocketEndpoint()
{
  _endpoint.stop_perpetual();

  if (!_current_connection)
  {
    return;
  }

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
    err << "> Error closing connection : " << ec.message() << std::endl
        << "> Host: " << _uri << std::endl;
    _logger(err.str());
  }

}
