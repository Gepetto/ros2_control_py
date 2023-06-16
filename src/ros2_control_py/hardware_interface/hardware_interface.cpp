#include "hardware_interface.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

void init_hardware_interface(py::module &m)
{

  py::module hardware_interface =
      m.def_submodule("hardware_interface");

  py::class_<InterfaceInfo>
      (hardware_interface,"InterfaceInfo")
      .def(py::init<>())
      .def_readwrite("name",&InterfaceInfo::name)
      .def_readwrite("min",&InterfaceInfo::min)
      .def_readwrite("max",&InterfaceInfo::max)
      .def_readwrite("initial_value",&InterfaceInfo::initial_value)
      .def_readwrite("data_type",&InterfaceInfo::data_type)
      .def_readwrite("size",&InterfaceInfo::size);

  py::class_<ComponentInfo>
      (hardware_interface,"ComponentInfo")
      .def(py::init<>())
      .def_readwrite("name",&ComponentInfo::name)
      .def_readwrite("type",&ComponentInfo::type)
      .def_readwrite("command_interfaces",
                     &ComponentInfo::command_interfaces)
      .def_readwrite("state_interfaces",
                     &ComponentInfo::state_interfaces)
      .def_readwrite("parameters",
                     &ComponentInfo::parameters);

  py::class_<ControllerInfo>
      (hardware_interface,"ControllerInfo")
      .def(py::init<>())
      .def_readwrite("name",&ControllerInfo::name)
      .def_readwrite("type",&ControllerInfo::type)
      .def_readwrite("claimed_interfaces",
                     &ControllerInfo::claimed_interfaces);

  py::class_<HardwareComponentInfo>
      (hardware_interface,"HardwareComponentInfo")
      .def(py::init<>())
      .def_readwrite("name",&HardwareComponentInfo::name)
      .def_readwrite("type",&HardwareComponentInfo::type)
      .def_readwrite("class_type",&HardwareComponentInfo::class_type)
      .def_readwrite("state",&HardwareComponentInfo::state)
      .def_readwrite("state_interfaces",
                     &HardwareComponentInfo::state_interfaces)
      .def_readwrite("command_interfaces",
                    &HardwareComponentInfo::command_interfaces);

  py::class_<JointInfo>
      (hardware_interface,"JointInfo")
      .def(py::init<>())
      .def_readwrite("name",&JointInfo::name)
      .def_readwrite("interfaces",&JointInfo::interfaces)
      .def_readwrite("role",&JointInfo::role)
      .def_readwrite("mechanical_reduction",&JointInfo::mechanical_reduction)
      .def_readwrite("offset",&JointInfo::offset);

  py::class_<ActuatorInfo>
      (hardware_interface,"ActuatorInfo")
      .def(py::init<>())
      .def_readwrite("name",&ActuatorInfo::name)
      .def_readwrite("interfaces",&ActuatorInfo::interfaces)
      .def_readwrite("role",&ActuatorInfo::role)
      .def_readwrite("mechanical_reduction",&ActuatorInfo::mechanical_reduction)
      .def_readwrite("offset",&ActuatorInfo::offset);

  py::class_<Actuator>
      (hardware_interface,"Actuator")
      .def(py::init<>())
      .def("initialize",&Actuator::initialize)
      .def("configure",&Actuator::configure)
      .def("cleanup",&Actuator::cleanup)
      .def("shutdown",&Actuator::shutdown)
      .def("activate",&Actuator::activate)
      .def("deactivate",&Actuator::deactivate)
      .def("error",&Actuator::error)
      .def("export_state_interfaces",&Actuator::export_state_interfaces)
      .def("export_command_interfaces",&Actuator::export_command_interfaces)
      .def("prepare_command_mode_switch",
           &Actuator::prepare_command_mode_switch)
      .def("perform_command_mode_switch",
           &Actuator::perform_command_mode_switch)
      .def("get_name",&Actuator::get_name)
      .def("get_state",&Actuator::get_state)
      .def("read",&Actuator::read)
      .def("write",&Actuator::write);

  py::class_<TransmissionInfo>
      (hardware_interface,"TransmissionInfo")
      .def(py::init<>())
      .def_readwrite("name",&TransmissionInfo::name)
      .def_readwrite("type",&TransmissionInfo::type)
      .def_readwrite("joints",&TransmissionInfo::joints)
      .def_readwrite("actuators",
                     &TransmissionInfo::actuators)
      .def_readwrite("parameters",
                     &TransmissionInfo::parameters);

  py::class_<HardwareInfo>
      (hardware_interface,"HardwareInfo")
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

  py::class_<ReadOnlyHandle>
      (hardware_interface,"ReadOnlyHandle")
      .def(py::init<std::string>())
      .def("get_name",&ReadOnlyHandle::get_name)
      .def("get_interface_name",&ReadOnlyHandle::get_interface_name)
      .def("get_full_name",&ReadOnlyHandle::get_full_name)
      .def("get_prefix_name",&ReadOnlyHandle::get_prefix_name)
      .def("get_value",&ReadOnlyHandle::get_value);

  py::class_<ReadWriteHandle>
      (hardware_interface,"ReadWriteHandle")
      .def(py::init<std::string>())
      .def("set_value",&ReadWriteHandle::get_value);


  py::class_<Sensor>
      (hardware_interface,"Sensor")
      .def(py::init<>())
      .def("initialize",&Sensor::initialize)
      .def("configure",&Sensor::configure)
      .def("cleanup",&Sensor::cleanup)
      .def("shutdown",&Sensor::shutdown)
      .def("activate",&Sensor::activate)
      .def("deactivate",&Sensor::deactivate)
      .def("error",&Sensor::error)
      .def("export_state_interfaces",&Sensor::export_state_interfaces)
      .def("get_name",&Sensor::get_name)
      .def("get_state",&Sensor::get_state)
      .def("read",&Sensor::read);

  py::class_<System>
      (hardware_interface,"System")
      .def(py::init<>())
      .def("initialize",&System::initialize)
      .def("configure",&System::configure)
      .def("cleanup",&System::cleanup)
      .def("shutdown",&System::shutdown)
      .def("activate",&System::activate)
      .def("deactivate",&System::deactivate)
      .def("error",&System::error)
      .def("export_state_interfaces",&System::export_state_interfaces)
      .def("export_command_interfaces",&System::export_command_interfaces)
      .def("prepare_command_mode_switch",
           &System::prepare_command_mode_switch)
      .def("perform_command_mode_switch",
           &System::perform_command_mode_switch)
      .def("get_name",&System::get_name)
      .def("get_state",&System::get_state)
      .def("read",&System::read)
      .def("write",&System::write);

  m.def("parse_control_resources_from_urdf",parse_control_resources_from_urdf);

  py::class_<HardwareReadWriteStatus>
      (hardware_interface,"HardwareReadWriteStatus")
      .def(py::init<>())
      .def_readwrite("ok",&HardwareReadWriteStatus::bool)
      .def_readwrite("failed_hardware_names",
                     &HardwareReadWriteStatus::failed_hardware_names)

}
}
}
