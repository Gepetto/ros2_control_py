#include <hardware_interface/handle.hpp>
#include "handle_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

void init_handle(py::module &hardware_interface_py)
{
  py::class_<ReadOnlyHandle>
    (hardware_interface_py,"ReadOnlyHandle")
    .def(py::init<const std::string &,
         const std::string &,
         double *>())
    .def(py::init<const std::string & >())
    .def(py::init<const char *>())
    .def("get_name",&ReadOnlyHandle::get_name)
    .def("get_interface_name",&ReadOnlyHandle::get_interface_name)
    .def("get_prefix_name",&ReadOnlyHandle::get_prefix_name)
    .def("get_value",&ReadOnlyHandle::get_value);

  py::class_<ReadWriteHandle>
    (hardware_interface_py,"ReadWriteHandle")
    .def(py::init< const std::string & ,
         const std::string & ,
         double *>())
    .def(py::init<const std::string &>())
    .def(py::init<const char *>())
    .def("set_value",&ReadWriteHandle::set_value);

  py::class_<StateInterface,ReadOnlyHandle>
    (hardware_interface_py,"StateInterface");

  py::class_<CommandInterface,ReadWriteHandle>
    (hardware_interface_py,"CommandInterface");
}
}
}
