#pragma once

// pybind11
#include <pybind11/pybind11.h>

// ros2_control_test_assets
#include <ros2_control_test_assets/descriptions.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_ros2_control_test_assets {

namespace py = pybind11;
using namespace ros2_control_test_assets;

inline void init_descriptions([[maybe_unused]] py::module &m) {
  m.attr("urdf_head") = urdf_head;

  m.attr("urdf_tail") = urdf_tail;

  m.attr("hardware_resources") = hardware_resources;

  m.attr("async_hardware_resources") = async_hardware_resources;

  m.attr("hardware_resources_with_different_rw_rates") =
      hardware_resources_with_different_rw_rates;

  m.attr("hardware_resources_with_negative_rw_rates") =
      hardware_resources_with_negative_rw_rates;

  m.attr("unitializable_hardware_resources") =
      uninitializable_hardware_resources;

  m.attr("hardware_resources_missing_state_keys") =
      hardware_resources_missing_state_keys;

  m.attr("hardware_resources_missing_command_keys") =
      hardware_resources_missing_command_keys;

  m.attr("diffbot_urdf") = diffbot_urdf;

  m.attr("minimal_robot_urdf") = minimal_robot_urdf;

  m.attr("minimal_uninitializable_robot_urdf") =
      minimal_uninitializable_robot_urdf;

  m.attr("minimal_robot_missing_state_keys_urdf") =
      minimal_robot_missing_state_keys_urdf;

  m.attr("minimal_robot_missing_command_keys_urdf") =
      minimal_robot_missing_command_keys_urdf;

  m.attr("TEST_ACTUATOR_HARDWARE_NAME") = TEST_ACTUATOR_HARDWARE_NAME;

  m.attr("TEST_ACTUATOR_HARDWARE_TYPE") = TEST_ACTUATOR_HARDWARE_TYPE;

  m.attr("TEST_ACTUATOR_HARDWARE_PLUGIN_NAME") =
      TEST_ACTUATOR_HARDWARE_PLUGIN_NAME;

  m.attr("TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES") =
      TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES;

  m.attr("TEST_ACTUATOR_HARDWARE_STATE_INTERFACES") =
      TEST_ACTUATOR_HARDWARE_STATE_INTERFACES;

  m.attr("TEST_SENSOR_HARDWARE_NAME") = TEST_SENSOR_HARDWARE_NAME;

  m.attr("TEST_SENSOR_HARDWARE_TYPE") = TEST_SENSOR_HARDWARE_TYPE;

  m.attr("TEST_SENSOR_HARDWARE_CLASS_TYPE") = TEST_SENSOR_HARDWARE_PLUGIN_NAME;

  m.attr("TEST_SENSOR_HARDWARE_COMMAND_INTERFACES") =
      TEST_SENSOR_HARDWARE_COMMAND_INTERFACES;

  m.attr("TEST_SENSOR_HARDWARE_STATE_INTERFACES") =
      TEST_SENSOR_HARDWARE_STATE_INTERFACES;

  m.attr("TEST_SYSTEM_HARDWARE_NAME") = TEST_SYSTEM_HARDWARE_NAME;

  m.attr("TEST_SYSTEM_HARDWARE_TYPE") = TEST_SYSTEM_HARDWARE_TYPE;

  m.attr("TEST_SYSTEM_HARDWARE_CLASS_TYPE") = TEST_SYSTEM_HARDWARE_PLUGIN_NAME;

  m.attr("TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES") =
      TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES;

  m.attr("TEST_SYSTEM_HARDWARE_STATE_INTERFACES") =
      TEST_SYSTEM_HARDWARE_STATE_INTERFACES;
}

}  // namespace ros2_control_py::bind_ros2_control_test_assets
