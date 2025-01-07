#pragma once

// pybind11
#include <pybind11/pybind11.h>

// transmission_interface
#include <transmission_interface/differential_transmission.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_transmission_interface {

namespace py = pybind11;
using namespace transmission_interface;

class PubDifferentialTransmission : public DifferentialTransmission {
 public:
  using DifferentialTransmission::actuator_effort_;
  using DifferentialTransmission::actuator_position_;
  using DifferentialTransmission::actuator_reduction_;
  using DifferentialTransmission::actuator_velocity_;
  using DifferentialTransmission::joint_effort_;
  using DifferentialTransmission::joint_offset_;
  using DifferentialTransmission::joint_position_;
  using DifferentialTransmission::joint_reduction_;
  using DifferentialTransmission::joint_velocity_;
};

inline void init_differential_transmission([[maybe_unused]] py::module &m) {
  m.def("DifferentialTransmission::configure",
        &DifferentialTransmission::configure);

  m.def("DifferentialTransmission::actuator_to_joint",
        &DifferentialTransmission::actuator_to_joint);

  m.def("DifferentialTransmission::joint_to_actuator",
        &DifferentialTransmission::joint_to_actuator);

  m.def("DifferentialTransmission::get_handles_info",
        &DifferentialTransmission::get_handles_info);
  py::class_<DifferentialTransmission, Transmission>(m,
                                                     "DifferentialTransmission")
      .def(py::init<const std::vector<double> &, const std::vector<double> &>())
      .def(py::init<const std::vector<double> &, const std::vector<double> &,
                    const std::vector<double> &>())
      .def("configure", &DifferentialTransmission::configure)
      .def("actuator_to_joint", &DifferentialTransmission::actuator_to_joint)
      .def("joint_to_actuator", &DifferentialTransmission::joint_to_actuator)
      .def("num_actuators", &DifferentialTransmission::num_actuators)
      .def("num_joints", &DifferentialTransmission::num_joints)
      .def("get_actuator_reduction",
           &DifferentialTransmission::get_actuator_reduction)
      .def("get_joint_reduction",
           &DifferentialTransmission::get_joint_reduction)
      .def("get_joint_offset", &DifferentialTransmission::get_joint_offset)
      .def("get_handles_info", &DifferentialTransmission::get_handles_info)
      .def_readwrite("actuator_reduction_",
                     &PubDifferentialTransmission::actuator_reduction_)
      .def_readwrite("joint_reduction_",
                     &PubDifferentialTransmission::joint_reduction_)
      .def_readwrite("joint_offset_",
                     &PubDifferentialTransmission::joint_offset_)
      .def_readwrite("joint_position_",
                     &PubDifferentialTransmission::joint_position_)
      .def_readwrite("joint_velocity_",
                     &PubDifferentialTransmission::joint_velocity_)
      .def_readwrite("joint_effort_",
                     &PubDifferentialTransmission::joint_effort_)
      .def_readwrite("actuator_position_",
                     &PubDifferentialTransmission::actuator_position_)
      .def_readwrite("actuator_velocity_",
                     &PubDifferentialTransmission::actuator_velocity_)
      .def_readwrite("actuator_effort_",
                     &PubDifferentialTransmission::actuator_effort_);
}

}  // namespace ros2_control_py::bind_transmission_interface
