#pragma once

// pybind11
#include <pybind11/pybind11.h>

// ros2_control_test_assets
#include <ros2_control_test_assets/components_urdfs.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_ros2_control_test_assets
{

namespace py = pybind11;
using namespace ros2_control_test_assets;

inline void init_components_urdfs([[maybe_unused]] py::module &m)
{
  m.attr("valid_urdf_ros2_control_system_one_interface") = valid_urdf_ros2_control_system_one_interface;

  m.attr("valid_urdf_ros2_control_system_multi_interface") = valid_urdf_ros2_control_system_multi_interface;

  m.attr("valid_urdf_ros2_control_system_robot_with_sensor") = valid_urdf_ros2_control_system_robot_with_sensor;

  m.attr("valid_urdf_ros2_control_system_robot_with_external_sensor") = valid_urdf_ros2_control_system_robot_with_external_sensor;

  m.attr("valid_urdf_ros2_control_actuator_modular_robot") = valid_urdf_ros2_control_actuator_modular_robot;

  m.attr("valid_urdf_ros2_control_actuator_modular_robot_sensors") = valid_urdf_ros2_control_actuator_modular_robot_sensors;

  m.attr("valid_urdf_ros2_control_system_multi_joints_transmission") = valid_urdf_ros2_control_system_multi_joints_transmission;

  m.attr("valid_urdf_ros2_control_sensor_only") = valid_urdf_ros2_control_sensor_only;

  m.attr("valid_urdf_ros2_control_actuator_only") = valid_urdf_ros2_control_actuator_only;

  m.attr("valid_urdf_ros2_control_system_robot_with_gpio") = valid_urdf_ros2_control_system_robot_with_gpio;

  m.attr("valid_urdf_ros2_control_system_robot_with_size_and_data_type") = valid_urdf_ros2_control_system_robot_with_size_and_data_type;

  m.attr("valid_urdf_ros2_control_parameter_empty") = valid_urdf_ros2_control_parameter_empty;

  m.attr("invalid_urdf_ros2_control_invalid_child") = invalid_urdf_ros2_control_invalid_child;

  m.attr("invalid_urdf_ros2_control_missing_attribute") = invalid_urdf_ros2_control_missing_attribute;

  m.attr("invalid_urdf_ros2_control_component_missing_class_type") = invalid_urdf_ros2_control_component_missing_plugin_name;

  m.attr("invalid_urdf_ros2_control_parameter_missing_name") = invalid_urdf_ros2_control_parameter_missing_name;

  m.attr("invalid_urdf_ros2_control_component_class_type_empty") = invalid_urdf_ros2_control_component_plugin_name_empty;

  m.attr("invalid_urdf_ros2_control_component_interface_type_empty") = invalid_urdf_ros2_control_component_interface_type_empty;

  m.attr("invalid_urdf2_ros2_control_illegal_size") = invalid_urdf2_ros2_control_illegal_size;

  m.attr("invalid_urdf2_ros2_control_illegal_size2") = invalid_urdf2_ros2_control_illegal_size2;

  m.attr("invalid_urdf2_hw_transmission_joint_mismatch") = invalid_urdf2_hw_transmission_joint_mismatch;

  m.attr("invalid_urdf2_transmission_given_too_many_joints") = invalid_urdf2_transmission_given_too_many_joints;

  m.attr("invalid_urdf2_ros2_control_system_with_command_fixed_joint") = invalid_urdf_ros2_control_system_with_command_fixed_joint;
}


}
