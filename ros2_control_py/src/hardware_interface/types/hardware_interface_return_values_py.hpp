#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/types/hardware_interface_return_values.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

inline void init_types_hardware_interface_return_values(
    [[maybe_unused]] py::module &m) {
  py::enum_<return_type>(m, "return_type")
      .value("OK", return_type::OK)
      .value("ERROR", return_type::ERROR)
      .export_values();
}

}  // namespace ros2_control_py::bind_hardware_interface
