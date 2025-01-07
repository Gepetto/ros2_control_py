#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/system.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

inline void init_system([[maybe_unused]] py::module &m) {
  py::class_<System>(m, "System")
      .def(py::init<>())
      .def(py::init<std::unique_ptr<SystemInterface>>())
      .def("initialize", &System::initialize)
      .def("configure", &System::configure)
      .def("cleanup", &System::cleanup)
      .def("shutdown", &System::shutdown)
      .def("activate", &System::activate)
      .def("deactivate", &System::deactivate)
      .def("error", &System::error)
      .def("export_state_interfaces", &System::export_state_interfaces)
      .def("export_command_interfaces", &System::export_command_interfaces)
      .def("prepare_command_mode_switch", &System::prepare_command_mode_switch)
      .def("perform_command_mode_switch", &System::perform_command_mode_switch)
      .def("get_name", &System::get_name)
      .def("get_lifecycle_state", &System::get_lifecycle_state)
      .def("read", &System::read)
      .def("write", &System::write);
}

}  // namespace ros2_control_py::bind_hardware_interface
