#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/component_parser.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface
{

namespace py = pybind11;
using namespace hardware_interface;

inline void init_component_parser([[maybe_unused]] py::module &m)
{
  m.def("parse_control_resources_from_urdf", &parse_control_resources_from_urdf);
}

}
