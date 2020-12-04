#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <optional>

#include <memory>
#include <random>
#include <cstdint>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/clone_ptr.hpp>
#include <rmf_utils/optional.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>

namespace py = pybind11;

rmf_task_msgs::msg::Delivery make_delivery_msg(
  std::string task_id,
  std::string pickup_place_name,
  std::string pickup_dispenser,
  std::string dropoff_place_name,
  std::string dropoff_ingestor)
{
  rmf_task_msgs::msg::Delivery request;
  request.task_id = task_id;

  request.pickup_place_name = pickup_place_name;
  request.pickup_dispenser = pickup_dispenser;

  request.dropoff_place_name = dropoff_place_name;
  request.dropoff_ingestor = dropoff_ingestor;

  return request;
}

rmf_task_msgs::msg::Loop make_loop_msg(
  std::string task_id,
  std::string robot_type,
  uint32_t num_loops,
  std::string start_name,
  std::string finish_name)
{
  rmf_task_msgs::msg::Loop request;
  request.task_id = task_id;
  request.robot_type = robot_type;
  request.num_loops = num_loops;

  request.start_name = start_name;
  request.finish_name = finish_name;

  return request;
}

using Duration = rmf_traffic::Duration;

void bind_types(py::module &m) {
  auto m_type = m.def_submodule("type");

  // Wrapper for the std::mt19937 class
  py::class_<std::mt19937>(m_type, "mt19937")
    .def(py::init<int>())
    .def(py::init<unsigned int>())
    .def_static("min", &std::mt19937::min)
    .def_static("max", &std::mt19937::max)
    // .def("seed", &std::mt19937::seed) // Doesn't exist for some reason
    .def("discard", &std::mt19937::discard)
    .def("__call__", &std::mt19937::operator());

  py::class_<rmf_task_msgs::msg::Delivery>(m_type, "CPPDeliveryMsg")
      .def(py::init(&make_delivery_msg),
           py::arg("task_id") = "",
           py::arg("pickup_place_name") = "",
           py::arg("pickup_dispenser") = "",
           py::arg("dropoff_place_name") = "",
           py::arg("dropoff_ingestor") = "")
      .def_property("task_id",
                    [&](rmf_task_msgs::msg::Delivery& self){
                        return self.task_id;},
                    [&](rmf_task_msgs::msg::Delivery& self,
                       std::string task_id){
                        self.task_id = task_id;})
      .def_property("pickup_place_name",
                    [&](rmf_task_msgs::msg::Delivery& self){
                        return self.pickup_place_name;},
                    [&](rmf_task_msgs::msg::Delivery& self,
                       std::string pickup_place_name){
                        self.pickup_place_name = pickup_place_name;})
      .def_property("pickup_dispenser",
                    [&](rmf_task_msgs::msg::Delivery& self){
                        return self.pickup_dispenser;},
                    [&](rmf_task_msgs::msg::Delivery& self,
                       std::string pickup_dispenser){
                        self.pickup_dispenser = pickup_dispenser;})
      .def_property("dropoff_place_name",
                    [&](rmf_task_msgs::msg::Delivery& self){
                        return self.dropoff_place_name;},
                    [&](rmf_task_msgs::msg::Delivery& self,
                       std::string dropoff_place_name){
                        self.dropoff_place_name = dropoff_place_name;})
      .def_property("dropoff_ingestor",
                    [&](rmf_task_msgs::msg::Delivery& self){
                        return self.dropoff_ingestor;},
                    [&](rmf_task_msgs::msg::Delivery& self,
                       std::string dropoff_ingestor){
                        self.dropoff_ingestor = dropoff_ingestor;});

  py::class_<rmf_task_msgs::msg::Loop>(m_type, "CPPLoopMsg")
      .def(py::init(&make_loop_msg),
           py::arg("task_id") = "",
           py::arg("robot_type") = "",
           py::arg("num_loops") = "",
           py::arg("start_name") = "",
           py::arg("finish_name") = "")
      .def_property("task_id",
                    [&](rmf_task_msgs::msg::Loop& self){
                        return self.task_id;},
                    [&](rmf_task_msgs::msg::Loop& self,
                       std::string task_id){
                        self.task_id = task_id;})
      .def_property("robot_type",
                    [&](rmf_task_msgs::msg::Loop& self){
                        return self.robot_type;},
                    [&](rmf_task_msgs::msg::Loop& self,
                       std::string robot_type){
                        self.robot_type = robot_type;})
      .def_property("num_loops",
                    [&](rmf_task_msgs::msg::Loop& self){
                        return self.num_loops;},
                    [&](rmf_task_msgs::msg::Loop& self,
                       uint32_t num_loops){
                        self.num_loops = num_loops;})
      .def_property("start_name",
                    [&](rmf_task_msgs::msg::Loop& self){
                        return self.start_name;},
                    [&](rmf_task_msgs::msg::Loop& self,
                       std::string start_name){
                        self.start_name = start_name;})
      .def_property("finish_name",
                    [&](rmf_task_msgs::msg::Loop& self){
                        return self.finish_name;},
                    [&](rmf_task_msgs::msg::Loop& self,
                       std::string finish_name){
                        self.finish_name = finish_name;});
}
