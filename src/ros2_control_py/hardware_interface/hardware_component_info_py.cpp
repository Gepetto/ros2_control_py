#include <hardware_interface/hardware_component_info.hpp>
#include "hardware_component_info_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

void init_hardware_component_info(py::module &hardware_interface)
{

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

}
}
}
