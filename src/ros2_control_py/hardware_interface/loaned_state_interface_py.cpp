#include <hardware_interface/loaned_state_interface.hpp>
#include "loaned_state_interface_py.hpp"

namespace ros2_control_py
{
  namespace bind_hardware_interface
  {
    using namespace hardware_interface;

    void init_loaned_state_interface(py::module &hardware_interface_py)
    {
      py::class_<LoanedStateInterface>
        (hardware_interface_py,"LoanedStateInterface")
        .def("get_name",&LoanedStateInterface::get_name)
        .def("get_interface_name",&LoanedStateInterface::get_interface_name)
        .def("get_prefix_name",&LoanedStateInterface::get_prefix_name)
        .def("get_value",&LoanedStateInterface::get_value);
    }
  }
}
