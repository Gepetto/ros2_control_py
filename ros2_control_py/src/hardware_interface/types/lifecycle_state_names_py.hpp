#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/types/lifecycle_state_names.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface::lifecycle_state_names;
using namespace hardware_interface;

inline void init_types_lifecycle_state_names([[maybe_unused]] py::module &m) {
  m.attr("UNKNOWN") = UNKNOWN;

  m.attr("UNCONFIGURED") = UNCONFIGURED;

  m.attr("INACTIVE") = INACTIVE;

  m.attr("ACTIVE") = ACTIVE;

  m.attr("FINALIZED") = FINALIZED;
}

}  // namespace ros2_control_py::bind_hardware_interface
