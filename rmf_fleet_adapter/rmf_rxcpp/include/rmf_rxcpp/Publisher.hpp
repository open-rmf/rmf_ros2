#ifndef RMF_RXCPP__PUBLISHER_HPP
#define RMF_RXCPP__PUBLISHER_HPP

#include <rxcpp/rx.hpp>

namespace rmf_rxcpp {

//==============================================================================
template<typename T>
class Publisher
{
public:

  enum DestructBehavior {
    is_complete_on_destruction,
    not_complete_on_destruction
  };

  /// Constructor
  Publisher(const DestructBehavior behavior = not_complete_on_destruction)
    : _is_completed_on_destruction(behavior == is_complete_on_destruction)
  {
    _observable = rxcpp::observable<>::create<T>(
          [this](rxcpp::subscriber<T> s)
    {
      this->_on_publish = [s](const T& msg) { s.on_next(msg); };
      this->_on_error = [s](std::exception_ptr e) { s.on_error(e); };
      this->_on_completed = [s]() { s.on_completed(); };
    }).publish().ref_count();
  }

  /// Destructor. Emit the on_completed signal during destruction if the
  /// DestructorBehavior was set to is_complete_on_destruction.
  ~Publisher()
  {
    if (_is_completed_on_destruction && _on_completed)
      _on_completed();
  }

  /// Publish a message to any subscribers
  void publish(const T& msg)
  {
    if (_on_publish)
      _on_publish(msg);
  }

  /// Emit an error to any subscribers
  void error(std::exception_ptr e)
  {
    if (_on_error)
      _on_error(std::move(e));
  }

  /// Send the on_completed notification to any subscribers
  void complete()
  {
    if (_on_completed)
      _on_completed();
  }

  /// Get the observable of this publisher
  const rxcpp::observable<T>& observe() const
  {
    return _observable;
  }

private:
  rxcpp::observable<T> _observable;
  std::function<void(const T&)> _on_publish;
  std::function<void(std::exception_ptr)> _on_error;
  std::function<void()> _on_completed;
  bool _is_completed_on_destruction;
};

} // namespace rmf_rxcpp

#endif // RMF_RXCPP__PUBLISHER_HPP
