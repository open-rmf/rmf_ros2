#include <pybind11/pybind11.h>

#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

#include <agv/internal_RobotUpdateHandle.hpp>
#include <agv/RobotContext.hpp>
#include <Reporting.hpp>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace py = pybind11;
namespace agv = rmf_fleet_adapter::agv;

class ReportingMutexFixture
{
public:
  explicit ReportingMutexFixture(agv::RobotUpdateHandle& handle)
  {
    _context =
      agv::RobotUpdateHandle::Implementation::get(handle).context.lock();

    if (!_context)
      throw std::runtime_error("RobotContext expired");

    _worker = std::thread([this]()
      {
        auto& reporting_mutex = _context->reporting().mutex();

        std::unique_lock<std::mutex> reporting_lock(reporting_mutex);

        {
          std::lock_guard<std::mutex> lock(_state_mutex);
          _locked = true;
        }

        _state_cv.notify_all();

        std::unique_lock<std::mutex> state_lock(_state_mutex);

        _state_cv.wait(state_lock, [this]()
          {
            return _release_requested;
          });
      });
  }

  void wait_until_locked()
  {
    py::gil_scoped_release release_gil;

    std::unique_lock<std::mutex> lock(_state_mutex);

    const bool acquired = _state_cv.wait_for(
      lock,
      std::chrono::seconds(5),
      [this]()
      {
        return _locked;
      });

    if (!acquired)
      throw std::runtime_error("Timed out waiting for reporting mutex");
  }

  void release()
  {
    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      _release_requested = true;
    }

    _state_cv.notify_all();
  }

  ~ReportingMutexFixture()
  {
    release();

    if (_worker.joinable())
    {
       py::gil_scoped_release release_gil;
       _worker.join();
    }
  }

private:
  std::shared_ptr<agv::RobotContext> _context;

  std::mutex _state_mutex;
  std::condition_variable _state_cv;

  bool _locked = false;
  bool _release_requested = false;

  std::thread _worker;
};

PYBIND11_MODULE(rmf_reporting_gil_test, m)
{
  m.doc() = "Test-only reporting mutex synchronization fixture";

  py::class_<ReportingMutexFixture>(m, "ReportingMutexFixture")
    .def(py::init<agv::RobotUpdateHandle&>())
    .def("wait_until_locked",
      &ReportingMutexFixture::wait_until_locked)
    .def("release",
      &ReportingMutexFixture::release);
}
