#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/sensor.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface
{

namespace py = pybind11;
using namespace hardware_interface;

inline void init_sensor([[maybe_unused]] py::module &m)
{
  py::class_<Sensor>(m, "Sensor")
      .def(py::init<>())
      .def(py::init<std::unique_ptr<SensorInterface>>())
      .def("initialize", &Sensor::initialize)
      .def("configure", &Sensor::configure)
      .def("cleanup", &Sensor::cleanup)
      .def("shutdown", &Sensor::shutdown)
      .def("activate", &Sensor::activate)
      .def("deactivate", &Sensor::deactivate)
      .def("error", &Sensor::error)
      .def("export_state_interfaces", &Sensor::export_state_interfaces)
      .def("get_name", &Sensor::get_name)
      .def("get_name", &Sensor::get_group_name)      
      .def("get_state", &Sensor::get_lifecycle_state)
      .def("read", &Sensor::read)
      .def("write", &Sensor::write);
}

}
