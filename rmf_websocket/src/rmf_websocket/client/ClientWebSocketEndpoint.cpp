#include "ClientWebSocketEndpoint.hpp"
#include <memory>
#include <sstream>

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
  std::string const& uri, std::shared_ptr<rclcpp::Node> node,
  asio::io_service* io_service,
  ConnectionCallback cb)
: _uri(uri), _node(node), _init{false}, _reconnect_enqueued(false),
  _connection_cb(std::move(cb))
{
  _endpoint = std::make_unique<WsClient>();
  _endpoint->clear_access_channels(websocketpp::log::alevel::all);
  _endpoint->clear_error_channels(websocketpp::log::elevel::all);
  _endpoint->init_asio(io_service);
  _endpoint->start_perpetual();
}

//=============================================================================
websocketpp::lib::error_code ClientWebSocketEndpoint::connect()
{
  websocketpp::lib::error_code ec;

  _init = true;
  _con = _endpoint->get_connection(_uri, ec);
  RCLCPP_INFO(_node->get_logger(), "Attempting to connect to %s", _uri.c_str());

  if (ec)
  {
    RCLCPP_ERROR(_node->get_logger(), "> Connect initialization error: %s\n"
      "> Host: %s\n", ec.message().c_str(), _uri.c_str());
    return ec;
  }

  auto reconnect_socket = [this]()
    {
      // TODO(arjo) Parametrize the timeout.
      RCLCPP_ERROR(_node->get_logger(),
        "Connection lost\n"
        "> Reconnecting in 1s\n"
        "> Host: %s", _uri.c_str());
      _endpoint->stop_perpetual();
      auto io_service = &_endpoint->get_io_service();
      _endpoint = std::make_unique<WsClient>();
      _endpoint->clear_access_channels(websocketpp::log::alevel::all);
      _endpoint->clear_error_channels(websocketpp::log::elevel::all);
      _endpoint->init_asio(io_service);
      _endpoint->start_perpetual();
      websocketpp::lib::error_code ec;

      _endpoint->set_timer(1000, std::bind(&ClientWebSocketEndpoint::connect,
        this));

    };
  auto connected_cb = [this]()
    {
      _connection_cb();
    };

  // Not sure why but seems like I have to re-initallize this everytime in order
  // to actually make a clean connection. My guess is the shared pointer is being
  // leaked somewhere.
  _current_connection = std::make_shared<ConnectionMetadata>(
    _con->get_handle(),
    _uri, connected_cb, reconnect_socket);

  _con->set_open_handler([this](websocketpp::connection_hdl hdl)
    {
      RCLCPP_INFO(_node->get_logger(), "Succesfully connected to %s",
      _uri.c_str());
      _current_connection->on_open(_endpoint.get(), hdl);
    });
  _con->set_fail_handler([this](websocketpp::connection_hdl hdl)
    {
      _current_connection->on_fail(_endpoint.get(), hdl);
      RCLCPP_ERROR(_node->get_logger(), "Connection to %s failed. Reason:\n %s",
      _uri.c_str(), _current_connection->debug_data().c_str());
    });

  _con->set_close_handler([this](websocketpp::connection_hdl hdl)
    {
      _current_connection->on_close(_endpoint.get(), hdl);
      RCLCPP_INFO(_node->get_logger(), "Connection to %s closed. Reason:\n %s",
      _uri.c_str(), _current_connection->debug_data().c_str());
    });


  _endpoint->connect(_con);

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

  _endpoint->send(
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
  _endpoint->stop_perpetual();

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
  _endpoint->close(
    _current_connection->get_hdl(), websocketpp::close::status::going_away, "",
    ec);
  if (ec)
  {
    RCLCPP_ERROR(
      _node->get_logger(), "Error closing connection: %s\n. Host: %s",
      ec.message().c_str(), _uri.c_str());
  }

}
