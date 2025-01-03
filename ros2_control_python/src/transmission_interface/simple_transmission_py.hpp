#pragma once

// pybind11
#include <pybind11/pybind11.h>

// transmission_interface
#include <transmission_interface/simple_transmission.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_transmission_interface
{

namespace py = pybind11;
using namespace transmission_interface;

class PubSimpleTransmission: public SimpleTransmission {
 public:
  using SimpleTransmission::reduction_;
  using SimpleTransmission::jnt_offset_;
  using SimpleTransmission::joint_position_;
  using SimpleTransmission::joint_velocity_;
  using SimpleTransmission::joint_effort_;
  using SimpleTransmission::actuator_position_;
  using SimpleTransmission::actuator_velocity_;
  using SimpleTransmission::actuator_effort_;
};

inline void init_simple_transmission([[maybe_unused]] py::module &m)
{
  m.def("SimpleTransmission::configure", &SimpleTransmission::configure);

  m.def("SimpleTransmission::actuator_to_joint", &SimpleTransmission::actuator_to_joint);

  m.def("SimpleTransmission::joint_to_actuator", &SimpleTransmission::joint_to_actuator);
  py::class_<SimpleTransmission, Transmission>(m, "SimpleTransmission")
      .def(py::init<const double>())
      .def(py::init<const double, const double>())
      .def("configure", &SimpleTransmission::configure)
      .def("actuator_to_joint", &SimpleTransmission::actuator_to_joint)
      .def("joint_to_actuator", &SimpleTransmission::joint_to_actuator)
      .def("num_actuators", &SimpleTransmission::num_actuators)
      .def("num_joints", &SimpleTransmission::num_joints)
      .def("get_actuator_reduction", &SimpleTransmission::get_actuator_reduction)
      .def("get_joint_offset", &SimpleTransmission::get_joint_offset)
      .def_readwrite("reduction_", &PubSimpleTransmission::reduction_)
      .def_readwrite("jnt_offset_", &PubSimpleTransmission::jnt_offset_)
      .def_readwrite("joint_position_", &PubSimpleTransmission::joint_position_)
      .def_readwrite("joint_velocity_", &PubSimpleTransmission::joint_velocity_)
      .def_readwrite("joint_effort_", &PubSimpleTransmission::joint_effort_)
      .def_readwrite("actuator_position_", &PubSimpleTransmission::actuator_position_)
      .def_readwrite("actuator_velocity_", &PubSimpleTransmission::actuator_velocity_)
      .def_readwrite("actuator_effort_", &PubSimpleTransmission::actuator_effort_);
}

}
