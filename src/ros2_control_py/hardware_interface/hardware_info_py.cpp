#include <hardware_interface/hardware_info.hpp>
#include "hardware_info_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

void init_hardware_info(py::module &hardware_interface_py)
{
  py::class_<InterfaceInfo>
      (hardware_interface_py,"InterfaceInfo")
      .def(py::init<>())
      .def_readwrite("name",&InterfaceInfo::name)
      .def_readwrite("min",&InterfaceInfo::min)
      .def_readwrite("max",&InterfaceInfo::max)
      .def_readwrite("initial_value",&InterfaceInfo::initial_value)
      .def_readwrite("data_type",&InterfaceInfo::data_type)
      .def_readwrite("size",&InterfaceInfo::size);

  py::class_<ComponentInfo>
      (hardware_interface_py,"ComponentInfo")
      .def(py::init<>())
      .def_readwrite("name",&ComponentInfo::name)
      .def_readwrite("type",&ComponentInfo::type)
      .def_readwrite("command_interfaces",
                     &ComponentInfo::command_interfaces)
      .def_readwrite("state_interfaces",
                     &ComponentInfo::state_interfaces)
      .def_readwrite("parameters",
                     &ComponentInfo::parameters);

  py::class_<JointInfo>
      (hardware_interface_py,"JointInfo")
      .def(py::init<>())
      .def_readwrite("name",&JointInfo::name)
      .def_readwrite("interfaces",&JointInfo::interfaces)
      .def_readwrite("role",&JointInfo::role)
      .def_readwrite("mechanical_reduction",&JointInfo::mechanical_reduction)
      .def_readwrite("offset",&JointInfo::offset);

  py::class_<ActuatorInfo>
      (hardware_interface_py,"ActuatorInfo")
      .def(py::init<>())
      .def_readwrite("name",&ActuatorInfo::name)
      .def_readwrite("interfaces",&ActuatorInfo::interfaces)
      .def_readwrite("role",&ActuatorInfo::role)
      .def_readwrite("mechanical_reduction",&ActuatorInfo::mechanical_reduction)
      .def_readwrite("offset",&ActuatorInfo::offset);

  py::class_<TransmissionInfo>
      (hardware_interface_py,"TransmissionInfo")
      .def(py::init<>())
      .def_readwrite("name",&TransmissionInfo::name)
      .def_readwrite("type",&TransmissionInfo::type)
      .def_readwrite("joints",&TransmissionInfo::joints)
      .def_readwrite("actuators",
                     &TransmissionInfo::actuators)
      .def_readwrite("parameters",
                     &TransmissionInfo::parameters);

  py::class_<HardwareInfo>
      (hardware_interface_py,"HardwareInfo")
      .def(py::init<>())
      .def_readwrite("name",&HardwareInfo::name)
      .def_readwrite("type",&HardwareInfo::type)
      .def_readwrite("hardware_class_type",
                     &HardwareInfo::hardware_class_type)
      .def_readwrite("hardware_parameters",
                     &HardwareInfo::hardware_parameters)
      .def_readwrite("joints",
                     &HardwareInfo::joints)
      .def_readwrite("sensors",
                     &HardwareInfo::sensors)
      .def_readwrite("gpios",
                     &HardwareInfo::gpios)
      .def_readwrite("transmissions",
                     &HardwareInfo::transmissions)
      .def_readwrite("original_xml",
                     &HardwareInfo::original_xml);


}
}
}
