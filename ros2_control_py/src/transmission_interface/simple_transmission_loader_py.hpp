#pragma once

// pybind11
#include <pybind11/pybind11.h>

// transmission_interface
#include <transmission_interface/simple_transmission_loader.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_transmission_interface {

namespace py = pybind11;
using namespace transmission_interface;

inline void init_simple_transmission_loader([[maybe_unused]] py::module &m) {
  py::class_<SimpleTransmissionLoader, TransmissionLoader>(
      m, "SimpleTransmissionLoader")
      .def(py::init<>())
      .def("load", &SimpleTransmissionLoader::load);
}

}  // namespace ros2_control_py::bind_transmission_interface
