#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/system.hpp>
#include "system_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

void init_system(py::module &hardware_interface_py)
{
  py::class_<System>
      (hardware_interface_py,"System")
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

}

}
}
