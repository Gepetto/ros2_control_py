#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/hardware_info.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

inline void init_hardware_info([[maybe_unused]] py::module &m) {
  py::class_<InterfaceInfo>(m, "InterfaceInfo")
      .def(py::init<>())
      .def_readwrite("name", &InterfaceInfo::name)
      .def_readwrite("min", &InterfaceInfo::min)
      .def_readwrite("max", &InterfaceInfo::max)
      .def_readwrite("initial_value", &InterfaceInfo::initial_value)
      .def_readwrite("data_type", &InterfaceInfo::data_type)
      .def_readwrite("size", &InterfaceInfo::size);

  py::class_<ComponentInfo>(m, "ComponentInfo")
      .def(py::init<>())
      .def_readwrite("name", &ComponentInfo::name)
      .def_readwrite("type", &ComponentInfo::type)
      .def_readwrite("command_interfaces", &ComponentInfo::command_interfaces)
      .def_readwrite("state_interfaces", &ComponentInfo::state_interfaces)
      .def_readwrite("parameters", &ComponentInfo::parameters);

  py::class_<JointInfo>(m, "JointInfo")
      .def(py::init<>())
      .def_readwrite("name", &JointInfo::name)
      .def_readwrite("state_interfaces", &JointInfo::state_interfaces)
      .def_readwrite("command_interfaces", &JointInfo::command_interfaces)
      .def_readwrite("role", &JointInfo::role)
      .def_readwrite("mechanical_reduction", &JointInfo::mechanical_reduction)
      .def_readwrite("offset", &JointInfo::offset);

  py::class_<ActuatorInfo>(m, "ActuatorInfo")
      .def(py::init<>())
      .def_readwrite("name", &ActuatorInfo::name)
      .def_readwrite("state_interfaces", &ActuatorInfo::state_interfaces)
      .def_readwrite("command_interfaces", &ActuatorInfo::command_interfaces)
      .def_readwrite("role", &ActuatorInfo::role)
      .def_readwrite("mechanical_reduction",
                     &ActuatorInfo::mechanical_reduction)
      .def_readwrite("offset", &ActuatorInfo::offset);

  py::class_<TransmissionInfo>(m, "TransmissionInfo")
      .def(py::init<>())
      .def_readwrite("name", &TransmissionInfo::name)
      .def_readwrite("type", &TransmissionInfo::type)
      .def_readwrite("joints", &TransmissionInfo::joints)
      .def_readwrite("actuators", &TransmissionInfo::actuators)
      .def_readwrite("parameters", &TransmissionInfo::parameters);

  py::class_<HardwareInfo>(m, "HardwareInfo")
      .def(py::init<>())
      .def_readwrite("name", &HardwareInfo::name)
      .def_readwrite("type", &HardwareInfo::type)
      .def_readwrite("hardware_plugin_name",
                     &HardwareInfo::hardware_plugin_name)
      .def_readwrite("hardware_parameters", &HardwareInfo::hardware_parameters)
      .def_readwrite("joints", &HardwareInfo::joints)
      .def_readwrite("sensors", &HardwareInfo::sensors)
      .def_readwrite("gpios", &HardwareInfo::gpios)
      .def_readwrite("transmissions", &HardwareInfo::transmissions)
      .def_readwrite("original_xml", &HardwareInfo::original_xml);
}

}  // namespace ros2_control_py::bind_hardware_interface
