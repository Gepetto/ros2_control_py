#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/controller_info.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface
{

namespace py = pybind11;
using namespace hardware_interface;

inline void init_controller_info([[maybe_unused]] py::module &m)
{
  py::class_<ControllerInfo>(m, "ControllerInfo")
      .def(py::init<>())
      .def_readwrite("name", &ControllerInfo::name)
      .def_readwrite("type", &ControllerInfo::type)
      .def_readwrite("parameters_files", &ControllerInfo::parameters_files)
      .def_readwrite("claimed_interfaces", &ControllerInfo::claimed_interfaces)
      .def_readwrite("fallback_controllers_names", &ControllerInfo::fallback_controllers_names);
}

}
