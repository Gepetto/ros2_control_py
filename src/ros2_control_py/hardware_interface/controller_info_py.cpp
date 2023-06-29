#include <hardware_interface/controller_info.hpp>
#include "controller_info_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

void init_controller_info(py::module &hardware_interface_py)
{
  py::class_<ControllerInfo>
      (hardware_interface_py,
       "ControllerInfo")
      .def(py::init<>())
      .def_readwrite("name",&InterfaceInfo::name)
      .def_readwrite("type",&InterfaceInfo::type)
      .def_readwrite("claimed_interfaces",&InterfaceInfo::claimed_interfaces)
}
}
}
