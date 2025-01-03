#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/types/hardware_interface_type_values.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface
{

namespace py = pybind11;
using namespace hardware_interface;

inline void init_types_hardware_interface_type_values([[maybe_unused]] py::module &m)
{
  m.attr("HW_IF_POSITION") = HW_IF_POSITION;

  m.attr("HW_IF_VELOCITY") = HW_IF_VELOCITY;

  m.attr("HW_IF_ACCELERATION") = HW_IF_ACCELERATION;

  m.attr("HW_IF_EFFORT") = HW_IF_EFFORT;
}

}
