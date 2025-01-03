#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/handle.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface
{

namespace py = pybind11;
using namespace hardware_interface;

class PubHandle: public Handle {
 public:
  using Handle::prefix_name_;
  using Handle::interface_name_;
  using Handle::handle_name_;
  using Handle::value_ptr_;
};

inline void init_handle([[maybe_unused]] py::module &m)
{
  py::class_<Handle>(m, "Handle")
      .def(py::init<const std::string&, const std::string&>())
      .def(py::init<const std::string&, const std::string&, Ref<double>>())
      .def(py::init<const std::string&>())
      .def(py::init<const char*>())
      .def(py::init<const InterfaceDescription &>())
      .def(py::init<const Handle &>())
      .def("get_name", &Handle::get_name)
      .def("get_interface_name", &Handle::get_interface_name)
      .def("get_prefix_name", &Handle::get_prefix_name)
      .def("get_value", static_cast< bool  (Handle::*)(double &) const>(&Handle::get_value))
      .def("set_value", &Handle::set_value)
      .def_readwrite("prefix_name_", &PubHandle::prefix_name_)
      .def_readwrite("interface_name_", &PubHandle::interface_name_)
      .def_readwrite("value_ptr_", &PubHandle::value_ptr_);

  py::class_<StateInterface>(m, "StateInterface")
      .def(py::init<const std::string&, const std::string&>())
      .def(py::init<const std::string&, const std::string&, Ref<double>>())
      .def(py::init<const std::string&>())
      .def(py::init<const InterfaceDescription &>())
      .def(py::init<const StateInterface &>())
      .def(py::init<const char*>());

  py::class_<CommandInterface>(m, "CommandInterface")
      .def(py::init<const std::string&, const std::string&>())
      .def(py::init<const std::string&, const std::string&, Ref<double>>())
      .def(py::init<const std::string&>())
      .def(py::init<const InterfaceDescription &>())
      .def(py::init<const char*>());
}

}
