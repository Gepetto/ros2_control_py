#pragma once

// pybind11
#include <pybind11/pybind11.h>

// controller_interface
#include <controller_interface/helpers.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_controller_interface {

namespace py = pybind11;
using namespace controller_interface;

inline void init_helpers([[maybe_unused]] py::module &m) {
  m.def("interface_list_contains_interface_type",
        &interface_list_contains_interface_type);
}

}  // namespace ros2_control_py::bind_controller_interface
