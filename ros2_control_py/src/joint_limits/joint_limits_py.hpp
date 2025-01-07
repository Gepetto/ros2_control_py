#pragma once

// pybind11
#include <pybind11/pybind11.h>

// joint_limits
#include <joint_limits/joint_limits.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_joint_limits {

namespace py = pybind11;
using namespace joint_limits;

inline void init_joint_limits([[maybe_unused]] py::module &m) {
  py::class_<JointLimits>(m, "JointLimits")
      .def(py::init<>())
      .def("to_string", &JointLimits::to_string)
      .def_readwrite("min_position", &JointLimits::min_position)
      .def_readwrite("max_position", &JointLimits::max_position)
      .def_readwrite("max_velocity", &JointLimits::max_velocity)
      .def_readwrite("max_acceleration", &JointLimits::max_acceleration)
      .def_readwrite("max_jerk", &JointLimits::max_jerk)
      .def_readwrite("max_effort", &JointLimits::max_effort)
      .def_readwrite("has_position_limits", &JointLimits::has_position_limits)
      .def_readwrite("has_velocity_limits", &JointLimits::has_velocity_limits)
      .def_readwrite("has_acceleration_limits",
                     &JointLimits::has_acceleration_limits)
      .def_readwrite("has_jerk_limits", &JointLimits::has_jerk_limits)
      .def_readwrite("has_effort_limits", &JointLimits::has_effort_limits)
      .def_readwrite("angle_wraparound", &JointLimits::angle_wraparound);

  py::class_<SoftJointLimits>(m, "SoftJointLimits")
      .def(py::init<>())
      .def("to_string", &SoftJointLimits::to_string)
      .def_readwrite("min_position", &SoftJointLimits::min_position)
      .def_readwrite("max_position", &SoftJointLimits::max_position)
      .def_readwrite("k_position", &SoftJointLimits::k_position)
      .def_readwrite("k_velocity", &SoftJointLimits::k_velocity);
}

}  // namespace ros2_control_py::bind_joint_limits
