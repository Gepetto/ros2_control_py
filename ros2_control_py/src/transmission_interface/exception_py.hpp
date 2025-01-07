#pragma once

// pybind11
#include <pybind11/pybind11.h>

// transmission_interface
#include <transmission_interface/exception.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_transmission_interface {

namespace py = pybind11;
using namespace transmission_interface;

inline void init_exception([[maybe_unused]] py::module &m) {
  py::class_<Exception>(m, "Exception")
      .def(py::init<const char *>())
      .def(py::init<const std::string &>())
      .def("what", &Exception::what);
}

}  // namespace ros2_control_py::bind_transmission_interface
