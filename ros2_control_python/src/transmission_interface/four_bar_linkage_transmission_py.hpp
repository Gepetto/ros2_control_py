#pragma once

// pybind11
#include <pybind11/pybind11.h>

// transmission_interface
#include <transmission_interface/four_bar_linkage_transmission.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_transmission_interface
{

namespace py = pybind11;
using namespace transmission_interface;

class PubFourBarLinkageTransmission: public FourBarLinkageTransmission {
 public:
  using FourBarLinkageTransmission::actuator_reduction_;
  using FourBarLinkageTransmission::joint_reduction_;
  using FourBarLinkageTransmission::joint_offset_;
  using FourBarLinkageTransmission::joint_position_;
  using FourBarLinkageTransmission::joint_velocity_;
  using FourBarLinkageTransmission::joint_effort_;
  using FourBarLinkageTransmission::actuator_position_;
  using FourBarLinkageTransmission::actuator_velocity_;
  using FourBarLinkageTransmission::actuator_effort_;
};

inline void init_four_bar_linkage_transmission([[maybe_unused]] py::module &m)
{
  m.def("FourBarLinkageTransmission::configure", &FourBarLinkageTransmission::configure);

  m.def("FourBarLinkageTransmission::actuator_to_joint", &FourBarLinkageTransmission::actuator_to_joint);

  m.def("FourBarLinkageTransmission::joint_to_actuator", &FourBarLinkageTransmission::joint_to_actuator);

  m.def("FourBarLinkageTransmission::get_handles_info", &FourBarLinkageTransmission::get_handles_info);
  py::class_<FourBarLinkageTransmission, Transmission>(m, "FourBarLinkageTransmission")
      .def(py::init<const std::vector<double>&, const std::vector<double>&>())
      .def(py::init<const std::vector<double>&, const std::vector<double>&, const std::vector<double>&>())
      .def("configure", &FourBarLinkageTransmission::configure)
      .def("actuator_to_joint", &FourBarLinkageTransmission::actuator_to_joint)
      .def("joint_to_actuator", &FourBarLinkageTransmission::joint_to_actuator)
      .def("num_actuators", &FourBarLinkageTransmission::num_actuators)
      .def("num_joints", &FourBarLinkageTransmission::num_joints)
      .def("get_actuator_reduction", &FourBarLinkageTransmission::get_actuator_reduction)
      .def("get_joint_reduction", &FourBarLinkageTransmission::get_joint_reduction)
      .def("get_joint_offset", &FourBarLinkageTransmission::get_joint_offset)
      .def("get_handles_info", &FourBarLinkageTransmission::get_handles_info)
      .def_readwrite("actuator_reduction_", &PubFourBarLinkageTransmission::actuator_reduction_)
      .def_readwrite("joint_reduction_", &PubFourBarLinkageTransmission::joint_reduction_)
      .def_readwrite("joint_offset_", &PubFourBarLinkageTransmission::joint_offset_)
      .def_readwrite("joint_position_", &PubFourBarLinkageTransmission::joint_position_)
      .def_readwrite("joint_velocity_", &PubFourBarLinkageTransmission::joint_velocity_)
      .def_readwrite("joint_effort_", &PubFourBarLinkageTransmission::joint_effort_)
      .def_readwrite("actuator_position_", &PubFourBarLinkageTransmission::actuator_position_)
      .def_readwrite("actuator_velocity_", &PubFourBarLinkageTransmission::actuator_velocity_)
      .def_readwrite("actuator_effort_", &PubFourBarLinkageTransmission::actuator_effort_);
}

}
