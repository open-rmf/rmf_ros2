#ifndef RMF_WEBSOCKET__UTILS_RINGBUFFER_HPP
#define RMF_WEBSOCKET__UTILS_RINGBUFFER_HPP

#include <condition_variable>
#include <cstddef>
#include <optional>
#include <boost/circular_buffer.hpp>
#include <mutex>

namespace rmf_websocket {

//==============================================================================
/// Thread safe fixed sized ring buffer
template<typename T>
class RingBuffer
{

//==============================================================================
public: RingBuffer(std::size_t size)
  : _vec(size)
  {

  }

//==============================================================================
/// Resize the capacity of the wing buffer
public: void resize(std::size_t buffer)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    _vec.set_capacity(buffer);
  }

//==============================================================================
/// Push an item onto the queue
public: void push(T item)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    _vec.push_back(std::move(item));
  }

//==============================================================================
public: bool empty()
  {
    return _vec.empty();
  }

//==============================================================================
public: std::optional<T> pop_item()
  {
    std::lock_guard<std::mutex> lock(_mtx);
    if (_vec.empty())
    {
      return std::nullopt;
    }

    T item = _vec.front();
    _vec.pop_front();

    return item;
  }

private:
  boost::circular_buffer<T> _vec;
  std::mutex _mtx;
};

}

#endif