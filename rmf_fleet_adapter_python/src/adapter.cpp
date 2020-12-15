#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <memory>

#include "rmf_traffic_ros2/Time.hpp"
#include "rmf_fleet_adapter/agv/Adapter.hpp"
#include "rmf_fleet_adapter/agv/test/MockAdapter.hpp"
#include "rmf_fleet_adapter_python/PyRobotCommandHandle.hpp"
#include <rmf_fleet_adapter/agv/Waypoint.hpp>
#include <rclcpp/rclcpp.hpp>

namespace py = pybind11;
namespace agv = rmf_fleet_adapter::agv;

using TimePoint = std::chrono::time_point<std::chrono::system_clock,
                                          std::chrono::nanoseconds>;

void bind_types(py::module &);
void bind_graph(py::module &);
void bind_shapes(py::module &);
void bind_vehicletraits(py::module &);
void bind_plan(py::module &);
void bind_tests(py::module &);
void bind_nodes(py::module &);

PYBIND11_MODULE(rmf_adapter, m) {
    bind_types(m);
    bind_graph(m);
    bind_shapes(m);
    bind_vehicletraits(m);
    bind_plan(m);
    bind_tests(m);
    bind_nodes(m);

    // ROBOTCOMMAND HANDLE =====================================================
    // Abstract class
    py::class_<agv::RobotCommandHandle, PyRobotCommandHandle,
               std::shared_ptr<agv::RobotCommandHandle> >(
                   m, "RobotCommandHandle", py::dynamic_attr())
        .def(py::init<>())
        .def("follow_new_path", &agv::RobotCommandHandle::follow_new_path,
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("stop", &agv::RobotCommandHandle::stop)
        .def("dock", &agv::RobotCommandHandle::dock);

    // ROBOTUPDATE HANDLE ======================================================
    py::class_<agv::RobotUpdateHandle,
               std::shared_ptr<agv::RobotUpdateHandle> >(
                   m, "RobotUpdateHandle")
        // Private constructor: Only to be constructed via FleetUpdateHandle!
        .def("interrupted", &agv::RobotUpdateHandle::interrupted)
        .def("update_current_waypoint",
             py::overload_cast<std::size_t, double>(
                 &agv::RobotUpdateHandle::update_position),
             py::arg("waypoint"),
             py::arg("orientation"),
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("update_current_lanes",
             py::overload_cast<const Eigen::Vector3d&,
                               const std::vector<std::size_t>& >(
                 &agv::RobotUpdateHandle::update_position),
             py::arg("position"),
             py::arg("lanes"),
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("update_off_grid_position",
             py::overload_cast<const Eigen::Vector3d&,
                               std::size_t>(
                 &agv::RobotUpdateHandle::update_position),
             py::arg("position"),
             py::arg("target_waypoint"),
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("update_lost_position",
             py::overload_cast<const std::string&,
                               const Eigen::Vector3d&,
                               const double,
                               const double,
                               const double>(
                                   &agv::RobotUpdateHandle::update_position),
             py::arg("map_name"),
             py::arg("position"),
             py::arg("max_merge_waypoint_distance") = 0.1,
             py::arg("max_merge_lane_distance") = 1.0,
             py::arg("min_lane_length") = 1e-8,
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>());

    // FLEETUPDATE HANDLE ======================================================
    py::class_<agv::FleetUpdateHandle,
               std::shared_ptr<agv::FleetUpdateHandle> >(
                   m, "FleetUpdateHandle")
        // NOTE(CH3): Might have to publicise this constructor if required
        // Otherwise, it's constructable via agv::TestScenario
        // .def(py::init<>())  // Private constructor!

        // Also, intentionally missing accept_delivery_requests
        .def("add_robot", &agv::FleetUpdateHandle::add_robot,
             // NOTE(CH3): Saved for posterity...
             /* py::overload_cast<std::shared_ptr<agv::RobotCommandHandle>,
                               const std::string&,
                               const rmf_traffic::Profile&,
                               rmf_traffic::agv::Plan::StartSet,
                               std::function<void(
                                 std::shared_ptr<RobotUpdateHandle> handle)>
                                 >(&agv::FleetUpdateHandle::add_robot), */
             py::arg("command"),
             py::arg("name"),
             py::arg("profile"),
             py::arg("start"),
             py::arg("handle_cb"))
        .def("accept_delivery_requests",
             &agv::FleetUpdateHandle::accept_delivery_requests);

    // EASY TRAFFIC LIGHT HANDLE ===============================================
    py::class_<agv::Waypoint>(m, "Waypoint")
      .def(py::init<std::string,
                    Eigen::Vector3d,
                    rmf_traffic::Duration,
                    bool>(),
           py::arg("map_name"),
           py::arg("position"),
           py::arg("mandatory_delay"),
           py::arg("yield"))
      .def_property("map_name",
           py::overload_cast<>(&agv::Waypoint::map_name, py::const_),
           py::overload_cast<std::string>(&agv::Waypoint::map_name))
      .def_property("position",
           py::overload_cast<>(&agv::Waypoint::position, py::const_),
           py::overload_cast<Eigen::Vector3d>(&agv::Waypoint::position))
      .def_property("mandatory_delay",
           py::overload_cast<>(&agv::Waypoint::mandatory_delay, py::const_),
           py::overload_cast<rmf_traffic::Duration>(
               &agv::Waypoint::mandatory_delay))
      .def_property("yield",
           py::overload_cast<>(&agv::Waypoint::yield, py::const_),
           py::overload_cast<bool>(&agv::Waypoint::yield));

    // EASY TRAFFIC LIGHT HANDLE ===============================================
    py::class_<agv::EasyTrafficLight,
               std::shared_ptr<agv::EasyTrafficLight> >(
                   m, "EasyTrafficLight", py::dynamic_attr())
        .def("follow_new_path",
            py::overload_cast<const std::vector<agv::Waypoint>&>(
                &agv::EasyTrafficLight::follow_new_path),
            py::arg("waypoint"),
            py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("moving_from",
            py::overload_cast<std::size_t, Eigen::Vector3d>(
                &agv::EasyTrafficLight::moving_from),
            py::arg("checkpoint"),
            py::arg("location"))
        .def("waiting_at",
            py::overload_cast<std::size_t>(
                &agv::EasyTrafficLight::waiting_at),
            py::arg("checkpoint"))
        .def("waiting_after",
            py::overload_cast<std::size_t, Eigen::Vector3d>(
                &agv::EasyTrafficLight::waiting_after),
            py::arg("checkpoint"),
            py::arg("location"))
        .def("last_reached", &agv::EasyTrafficLight::last_reached);

    // prefix traffic light
    auto m_easy_traffic_light = m.def_submodule("easy_traffic_light");

    py::enum_<agv::EasyTrafficLight::MovingInstruction>(
        m_easy_traffic_light, "MovingInstruction")
        .value("MovingError", 
            agv::EasyTrafficLight::MovingInstruction::MovingError)
        .value("ContinueAtNextCheckpoint", 
            agv::EasyTrafficLight::MovingInstruction::ContinueAtNextCheckpoint)
        .value("WaitAtNextCheckpoint", 
            agv::EasyTrafficLight::MovingInstruction::WaitAtNextCheckpoint)        
        .value("PauseImmediately", 
            agv::EasyTrafficLight::MovingInstruction::PauseImmediately);

    py::enum_<agv::EasyTrafficLight::WaitingInstruction>(
        m_easy_traffic_light, "WaitingInstruction")
        .value("WaitingError", 
            agv::EasyTrafficLight::WaitingInstruction::WaitingError)
        .value("Resume", 
            agv::EasyTrafficLight::WaitingInstruction::Resume)     
        .value("Wait", 
            agv::EasyTrafficLight::WaitingInstruction::Wait);

    // ADAPTER =================================================================
    // Light wrappers
    py::class_<rclcpp::NodeOptions>(m, "NodeOptions")
        .def(py::init<>());

    py::class_<rclcpp::Node, std::shared_ptr<rclcpp::Node> >(m, "Node")
        .def("now", [](rclcpp::Node& self){
            return rmf_traffic_ros2::convert(self.now());
        });

    // Python rclcpp init and spin call
    m.def("init_rclcpp", [](){ rclcpp::init(0, nullptr); });
    m.def("spin_rclcpp", [](rclcpp::Node::SharedPtr node_pt){
        rclcpp::spin(node_pt);});
    m.def("spin_some_rclcpp", [](rclcpp::Node::SharedPtr node_pt){
        rclcpp::spin_some(node_pt);});

    py::class_<agv::Adapter, std::shared_ptr<agv::Adapter> >(m, "Adapter")
        // .def(py::init<>())  // Private constructor
        .def_static("make", &agv::Adapter::make,
             py::arg("node_name"),
             py::arg("node_options") = rclcpp::NodeOptions(),
             py::arg("wait_time") = rmf_utils::optional<rmf_traffic::Duration>(
                 rmf_utils::nullopt),
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("add_fleet", &agv::Adapter::add_fleet,
             py::arg("fleet_name"),
             py::arg("traits"),
             py::arg("navigation_graph"))
        .def("add_easy_traffic_light", &agv::Adapter::add_easy_traffic_light,
             py::arg("handle_callback"),
             py::arg("fleet_name"),
             py::arg("robot_name"),
             py::arg("traits"),
             py::arg("pause_callback"),
             py::arg("resume_callback"),
             py::arg("blocker_callback") = nullptr)
        .def_property_readonly("node",
                               py::overload_cast<>(&agv::Adapter::node))
        .def("start", &agv::Adapter::start)
        .def("stop", &agv::Adapter::stop)
        .def("now", [](agv::Adapter& self) {
            return TimePoint(rmf_traffic_ros2::convert(self.node()->now())
                                 .time_since_epoch());
        });

    py::class_<agv::test::MockAdapter,
               std::shared_ptr<agv::test::MockAdapter> >(m, "MockAdapter")
        .def(py::init<const std::string&,
                      const rclcpp::NodeOptions&>(),
             py::arg("node_name"),
             py::arg("node_options") = rclcpp::NodeOptions(),
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("add_fleet", &agv::test::MockAdapter::add_fleet,
             py::arg("fleet_name"),
             py::arg("traits"),
             py::arg("navigation_graph"))
        .def_property_readonly("node",
                               py::overload_cast<>(
                                   &agv::test::MockAdapter::node))
        .def("request_delivery", &agv::test::MockAdapter::request_delivery) // Exposed for testing
        .def("request_loop", &agv::test::MockAdapter::request_loop) // Exposed for testing
        .def("start", &agv::test::MockAdapter::start)
        .def("stop", &agv::test::MockAdapter::stop)
        .def("now", [&](agv::test::MockAdapter& self) {
            return TimePoint(rmf_traffic_ros2::convert(self.node()->now())
                                 .time_since_epoch());
        });
}
