#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/actuator.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

inline void init_actuator([[maybe_unused]] py::module &m) {
  py::class_<Actuator>(m, "Actuator")
      .def(py::init<>())
      .def(py::init<std::unique_ptr<ActuatorInterface>>())
      .def("initialize", &Actuator::initialize)
      .def("configure", &Actuator::configure)
      .def("cleanup", &Actuator::cleanup)
      .def("shutdown", &Actuator::shutdown)
      .def("activate", &Actuator::activate)
      .def("deactivate", &Actuator::deactivate)
      .def("error", &Actuator::error)
      .def("export_state_interfaces", &Actuator::export_state_interfaces)
      .def("export_command_interfaces", &Actuator::export_command_interfaces)
      .def("prepare_command_mode_switch",
           &Actuator::prepare_command_mode_switch)
      .def("perform_command_mode_switch",
           &Actuator::perform_command_mode_switch)
      .def("get_name", &Actuator::get_name)
      .def("get_group_name", &Actuator::get_group_name)
      .def("get_lifecycle_state", &Actuator::get_lifecycle_state)
      .def("read", &Actuator::read)
      .def("write", &Actuator::write);
}

}  // namespace ros2_control_py::bind_hardware_interface
