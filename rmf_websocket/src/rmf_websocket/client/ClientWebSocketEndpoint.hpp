#ifndef RMF_WEBSOCKET__CLIENT_CLIENTWEBSOCKETENDPOINT_HPP
#define RMF_WEBSOCKET__CLIENT_CLIENTWEBSOCKETENDPOINT_HPP

#include <condition_variable>
#include <mutex>
#include <thread>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>


namespace rmf_websocket {


/// Client
typedef websocketpp::client<websocketpp::config::asio_client> WsClient;

/// Logger
typedef std::function<void(const std::string&)> Logger;

/// Helper class with event handlers for managing connection state.
class ConnectionMetadata
{
public:
  typedef websocketpp::lib::shared_ptr<ConnectionMetadata> ptr;

  /// Constuctor
  ConnectionMetadata(websocketpp::connection_hdl hdl, std::string uri);

  /// On open event handler
  void on_open(WsClient* c, websocketpp::connection_hdl hdl);

  /// On fail event handler
  void on_fail(WsClient* c, websocketpp::connection_hdl hdl);

  /// On close event handler
  void on_close(WsClient* c, websocketpp::connection_hdl hdl);

  /// Get status
  std::string get_status();

  /// Get debug string
  std::string debug_data();

  /// Get connection handle
  websocketpp::connection_hdl get_hdl() const;

  /// Check if the connection is ready to be used.
  /// \param[in] timeout - milliseconds to waitn
  /// \return True if connection was successfully established.
  /// False otherwise.
  bool wait_for_ready(const long timeout);

  friend std::ostream& operator<<(std::ostream& out,
    ConnectionMetadata const& data);
private:
  websocketpp::connection_hdl _hdl;
  WsClient::connection_ptr _con;
  std::string _status;
  std::string _uri;
  std::string _server;
  std::string _error_reason;
  std::mutex _status_mtx;
  std::condition_variable _cv;

};


/// This classs abstracts out reconnecting to an end point.
class ClientWebSocketEndpoint
{
public:
  /// Constructor
  ClientWebSocketEndpoint(
    std::string const& uri,
    Logger _my_logger);

  /// Initiates a connection returns 0 if everything goes ok.
  /// Note: This is non blocking and does not gaurantee a connection
  /// has been established.
  int connect();

  /// Gets the current connection metadata. This includes the current
  /// link state.
  ConnectionMetadata::ptr get_metadata() const;

  /// Send a message.
  void send(std::string message);

  /// Destructor
  ~ClientWebSocketEndpoint();

  /// Waits till a connection is successfully established.
  /// Note: only one thread can call this function.
  void wait_for_ready();

  /// Interrupt any wait.
  void interrupt_waits();

private:
  WsClient _endpoint;
  websocketpp::lib::shared_ptr<websocketpp::lib::thread> _thread;
  std::atomic<bool> _stop;

  ConnectionMetadata::ptr _current_connection;
  std::string _uri;

  /// prevents the destructor from running
  std::mutex _mtx;

  Logger _logger;
};
}
#endif