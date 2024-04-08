#ifndef RMF_WEBSOCKET__CLIENT_CLIENTWEBSOCKETENDPOINT_HPP
#define RMF_WEBSOCKET__CLIENT_CLIENTWEBSOCKETENDPOINT_HPP

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <cstdlib>
#include <functional>
#include <map>
#include <string>
#include <sstream>
#include <vector>

#include <rclcpp/node.hpp>

namespace rmf_websocket {

/// Client
typedef websocketpp::client<websocketpp::config::asio_client> WsClient;


/// Helper class with event handlers for managing connection state.
class ConnectionMetadata
{
public:
  /// Connection callback
  typedef std::function<void()> ConnectionCallback;

  /// Connection callback
  typedef std::function<void()> ReconnectionCallback;

  /// Connection Status
  enum class ConnectionStatus
  {
    CONNECTING,
    OPEN,
    FAILED,
    CLOSED
  };

  typedef websocketpp::lib::shared_ptr<ConnectionMetadata> ptr;

  /// Constuctor
  ConnectionMetadata(
    websocketpp::connection_hdl hdl,
    std::string uri,
    ConnectionCallback cb,
    ReconnectionCallback rcb);

  /// On open event handler
  void on_open(WsClient* c, websocketpp::connection_hdl hdl);

  /// On fail event handler
  void on_fail(WsClient* c, websocketpp::connection_hdl hdl);

  /// On close event handler
  void on_close(WsClient* c, websocketpp::connection_hdl hdl);

  /// Get status
  ConnectionStatus get_status() const;

  /// Get debug string
  std::string debug_data() const;

  /// Get connection handle
  websocketpp::connection_hdl get_hdl() const;

  /// reset
  void reset();

  friend std::ostream& operator<<(std::ostream& out,
    ConnectionMetadata const& data);
private:
  websocketpp::connection_hdl _hdl;
  WsClient::connection_ptr _con;
  ConnectionStatus _status;
  std::string _uri;
  std::string _server;
  std::string _error_reason;
  ConnectionCallback _connection_cb;
  ReconnectionCallback _reconnection_cb;
};


/// This classs abstracts out reconnecting to an end point.
class ClientWebSocketEndpoint
{
public:
  typedef std::function<void()> ConnectionCallback;
  /// Constructor
  /// Pass io service so that multiple endpoints
  /// can run on the same thread
  ClientWebSocketEndpoint(
    std::string const& uri,
    std::shared_ptr<rclcpp::Node> node,
    boost::asio::io_service* io_service,
    ConnectionCallback cb);

  /// Delete move constructor
  ClientWebSocketEndpoint(
    ClientWebSocketEndpoint&& other) = delete;

  /// Delete move assignment
  ClientWebSocketEndpoint& operator=(
    ClientWebSocketEndpoint&& other) = delete;

  /// Delete copy constructor
  ClientWebSocketEndpoint(
    const ClientWebSocketEndpoint& other) = delete;

  /// Delete copy
  ClientWebSocketEndpoint operator=(
    const ClientWebSocketEndpoint& other) = delete;

  /// Initiates a connection returns 0 if everything goes ok.
  /// Note: This is non blocking and does not gaurantee a connection
  /// has been established.
  websocketpp::lib::error_code connect();

  /// Gets the current connection metadata. This includes the current
  /// link state.
  std::optional<ConnectionMetadata::ConnectionStatus> get_status() const;

  /// Send a message.
  websocketpp::lib::error_code send(const std::string& message);

  /// Destructor
  ~ClientWebSocketEndpoint();

private:
  std::unique_ptr<WsClient> _endpoint;
  std::atomic<bool> _stop;

  ConnectionMetadata::ptr _current_connection;
  std::string _uri;
  std::shared_ptr<rclcpp::Node> _node;
  WsClient::connection_ptr _con;
  bool _init, _enqueued_conn, _reconnect_enqueued;
  ConnectionCallback _connection_cb;
};
}
#endif
