#ifndef RMF_RXCPP__PUBLISHER_HPP
#define RMF_RXCPP__PUBLISHER_HPP

#include <rxcpp/rx.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_rxcpp {

//==============================================================================
// TODO(MXG): This class appears to be redundant with the
// rxcpp::subjects::subject class, so it should probably be thrown out.
template<typename T>
class Publisher
{
public:

  enum DestructBehavior {
    is_complete_on_destruction,
    not_complete_on_destruction
  };

  enum LatchBehavior {
    latch_last,
    no_latch
  };

  /// Constructor
  //
  // TODO(MXG): latch_last is kind of a hack to help out the ActivePhases that
  // want to publish an initial status in their constructor, before anyone could
  // have subscribed.
  Publisher(
      const LatchBehavior latch = latch_last,
      const DestructBehavior destruct = not_complete_on_destruction)
    : _is_completed_on_destruction(destruct == is_complete_on_destruction),
      _latch_last(latch == latch_last)
  {
    _observable = rxcpp::observable<>::create<T>(
          [this](rxcpp::subscriber<T> s)
    {
      if (_latch_last && _last_msg)
        s.on_next(*_last_msg);

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
    else if (_latch_last)
      _last_msg = msg;
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
  rmf_utils::optional<T> _last_msg;
  bool _is_completed_on_destruction;
  bool _latch_last;
};

} // namespace rmf_rxcpp

#endif // RMF_RXCPP__PUBLISHER_HPP
