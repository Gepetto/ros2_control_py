#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/hardware_component_info.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

inline void init_hardware_component_info([[maybe_unused]] py::module &m) {
  py::class_<HardwareComponentInfo>(m, "HardwareComponentInfo")
      .def(py::init<>())
      .def_readwrite("name", &HardwareComponentInfo::name)
      .def_readwrite("type", &HardwareComponentInfo::type)
      .def_readwrite("plugin_name", &HardwareComponentInfo::plugin_name)
      .def_readwrite("state", &HardwareComponentInfo::state)
      .def_readwrite("state_interfaces",
                     &HardwareComponentInfo::state_interfaces)
      .def_readwrite("command_interfaces",
                     &HardwareComponentInfo::command_interfaces);
}

}  // namespace ros2_control_py::bind_hardware_interface
