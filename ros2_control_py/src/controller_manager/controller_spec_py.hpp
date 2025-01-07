#pragma once

// pybind11
#include <pybind11/pybind11.h>

// controller_manager
#include <controller_manager/controller_spec.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_controller_manager {

namespace py = pybind11;
using namespace controller_manager;

inline void init_controller_spec([[maybe_unused]] py::module &m) {
  py::class_<ControllerSpec>(m, "ControllerSpec")
      .def(py::init<>())
      .def_readwrite("info", &ControllerSpec::info)
      .def_readwrite("c", &ControllerSpec::c);
}

}  // namespace ros2_control_py::bind_controller_manager
