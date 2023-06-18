#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/actuator.hpp>
#include "actuator_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

void init_actuator(py::module &hardware_interface_py)
{
  py::class_<Actuator>
      (hardware_interface_py,"Actuator")
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
}
}
}
