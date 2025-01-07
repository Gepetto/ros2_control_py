#pragma once

// pybind11
#include <pybind11/pybind11.h>

// transmission_interface
#include <transmission_interface/handle.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_transmission_interface {

namespace py = pybind11;
using namespace transmission_interface;

inline void init_handle([[maybe_unused]] py::module &m) {
  py::class_<ActuatorHandle, hardware_interface::Handle>(m, "ActuatorHandle")
      .def(py::init<const std::string &, const std::string &>())
      .def(py::init<const std::string &, const std::string &, Ref<double>>())
      .def(py::init<const std::string &>())
      .def(py::init<const char *>());

  py::class_<JointHandle, hardware_interface::Handle>(m, "JointHandle")
      .def(py::init<const std::string &, const std::string &>())
      .def(py::init<const std::string &, const std::string &, Ref<double>>())
      .def(py::init<const std::string &>())
      .def(py::init<const char *>());
}

}  // namespace ros2_control_py::bind_transmission_interface
