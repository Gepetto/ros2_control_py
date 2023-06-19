#include <hardware_interface/loaned_command_interface.hpp>
#include "loaned_command_interface_py.hpp"

namespace ros2_control_py
{
  namespace bind_hardware_interface
  {
    using namespace hardware_interface;

    void init_loaned_command_interface(py::module &hardware_interface_py)
    {
      py::class_<LoanedCommandInterface>
        (hardware_interface_py,"LoanedCommandInterface")
        .def("get_name",&LoanedCommandInterface::get_name)
        .def("get_interface_name",&LoanedCommandInterface::get_interface_name)
        .def("get_prefix_name",&LoanedCommandInterface::get_prefix_name)
        .def("get_value",&LoanedCommandInterface::get_value)
        .def("set_value",&LoanedCommandInterface::set_value);
    }
  }
}
