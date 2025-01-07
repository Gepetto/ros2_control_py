#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/lexical_casts.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

inline void init_lexical_casts([[maybe_unused]] py::module &m) {
  m.def("stod", &stod);

  m.def("parse_bool", &parse_bool);
}

}  // namespace ros2_control_py::bind_hardware_interface
