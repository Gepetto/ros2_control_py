#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/loaned_state_interface.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface
{

namespace py = pybind11;
using namespace hardware_interface;


class ProxyLoanedStateInterface 
{

 private:
  LoanedStateInterface a_loaned_state_interface_;
 public:

  ProxyLoanedStateInterface(StateInterface::ConstSharedPtr state_interface):
      a_loaned_state_interface_(state_interface)
  {
  }
  ProxyLoanedStateInterface (StateInterface::ConstSharedPtr state_interface,
                             LoanedStateInterface::Deleter deleter):
      a_loaned_state_interface_(state_interface, std::move(deleter))
  {}

  ProxyLoanedStateInterface (LoanedStateInterface && other):
      a_loaned_state_interface_(std::move(other))
  {}

  const std::string & get_name() const { return a_loaned_state_interface_.get_name(); }
  
  const std::string & get_interface_name() const { return a_loaned_state_interface_.get_interface_name(); }

  const std::string & get_prefix_name() const { return a_loaned_state_interface_.get_prefix_name(); }

  double get_value() const { return a_loaned_state_interface_.get_value(); }
};

inline void init_loaned_state_interface([[maybe_unused]] py::module &m)
{
  py::class_<ProxyLoanedStateInterface>(m, "ProxyLoanedStateInterface")
      .def(py::init<StateInterface::ConstSharedPtr>())
      .def(py::init<StateInterface::ConstSharedPtr,
           LoanedStateInterface::Deleter>())
      .def("get_name", &ProxyLoanedStateInterface::get_name)
      .def("get_interface_name", &ProxyLoanedStateInterface::get_interface_name)
      .def("get_prefix_name", &ProxyLoanedStateInterface::get_prefix_name)
      .def("get_value", &ProxyLoanedStateInterface::get_value);
}

}
