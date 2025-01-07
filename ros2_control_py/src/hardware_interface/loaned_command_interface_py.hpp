#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/loaned_command_interface.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

class PubLoanedCommandInterface : public LoanedCommandInterface {
 public:
  using LoanedCommandInterface::deleter_;
};

inline void init_loaned_command_interface([[maybe_unused]] py::module &m) {
  py::class_<LoanedCommandInterface>(m, "LoanedCommandInterface")
      .def(py::init<CommandInterface &>())
      .def("get_name", &LoanedCommandInterface::get_name)
      .def("get_interface_name", &LoanedCommandInterface::get_interface_name)
      .def("get_prefix_name", &LoanedCommandInterface::get_prefix_name)
      .def("get_value", static_cast<double (LoanedCommandInterface::*)() const>(
                            &LoanedCommandInterface::get_value))
      .def_readwrite("deleter_", &PubLoanedCommandInterface::deleter_);
}

}  // namespace ros2_control_py::bind_hardware_interface
