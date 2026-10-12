// pybind11
#include <pybind11/pybind11.h>

// ros2_control_py
#include <controller_interface/chainable_controller_interface_py.hpp>
#include <controller_interface/controller_interface_base_py.hpp>
#include <controller_interface/controller_interface_py.hpp>
#include <controller_interface/helpers_py.hpp>
#include <controller_manager/controller_manager_py.hpp>
#include <controller_manager/controller_spec_py.hpp>
#include <hardware_interface/actuator_interface_py.hpp>
#include <hardware_interface/actuator_py.hpp>
#include <hardware_interface/component_parser_py.hpp>
#include <hardware_interface/controller_info_py.hpp>
#include <hardware_interface/handle_py.hpp>
#include <hardware_interface/hardware_component_info_py.hpp>
#include <hardware_interface/hardware_info_py.hpp>
#include <hardware_interface/lexical_casts_py.hpp>
#include <hardware_interface/loaned_command_interface_py.hpp>
#include <hardware_interface/loaned_state_interface_py.hpp>
#include <hardware_interface/resource_manager_py.hpp>
#include <hardware_interface/sensor_interface_py.hpp>
#include <hardware_interface/sensor_py.hpp>
#include <hardware_interface/system_interface_py.hpp>
#include <hardware_interface/system_py.hpp>
#include <hardware_interface/types/hardware_interface_return_values_py.hpp>
#include <hardware_interface/types/hardware_interface_type_values_py.hpp>
#include <hardware_interface/types/lifecycle_state_names_py.hpp>
#include <joint_limits/joint_limits_py.hpp>
#include <joint_limits/joint_limits_rosparam_py.hpp>
#include <rclcpp/py_ref_py.hpp>
#include <rclcpp/rclcpp_py.hpp>
#include <ros2_control_test_assets/components_urdfs_py.hpp>
#include <ros2_control_test_assets/descriptions_py.hpp>
#include <transmission_interface/accessor_py.hpp>
#include <transmission_interface/differential_transmission_loader_py.hpp>
#include <transmission_interface/differential_transmission_py.hpp>
#include <transmission_interface/exception_py.hpp>
#include <transmission_interface/four_bar_linkage_transmission_loader_py.hpp>
#include <transmission_interface/four_bar_linkage_transmission_py.hpp>
#include <transmission_interface/handle_py.hpp>
#include <transmission_interface/simple_transmission_loader_py.hpp>
#include <transmission_interface/simple_transmission_py.hpp>
#include <transmission_interface/transmission_interface_exception_py.hpp>
#include <transmission_interface/transmission_loader_py.hpp>
#include <transmission_interface/transmission_py.hpp>

namespace py = pybind11;

PYBIND11_MODULE(_impl_ros2_control_py, r2cpy) {
  r2cpy.doc() = R"doc(
            Python bindings for ros2_control functionalities.
            )doc";

  // Provide custom function signatures
  py::options options;
  options.disable_function_signatures();

  // define submodules
  auto joint_limits =
      r2cpy.def_submodule("joint_limits", "Python bindings for joint_limits");
  auto ros2_control_test_assets =
      r2cpy.def_submodule("ros2_control_test_assets",
                          "Python bindings for ros2_control_test_assets");
  auto rclcpp = r2cpy.def_submodule("rclcpp", "Python bindings for rclcpp");
  auto controller_interface = r2cpy.def_submodule(
      "controller_interface", "Python bindings for controller_interface");
  auto controller_manager = r2cpy.def_submodule(
      "controller_manager", "Python bindings for controller_manager");
  auto hardware_interface = r2cpy.def_submodule(
      "hardware_interface", "Python bindings for hardware_interface");
  auto transmission_interface = r2cpy.def_submodule(
      "transmission_interface", "Python bindings for transmission_interface");

  // Construct module classes
  ros2_control_py::bind_impl::init(rclcpp);
  ros2_control_py::bind_joint_limits::init_joint_limits_rosparam(joint_limits);
  ros2_control_py::bind_joint_limits::init_joint_limits(joint_limits);
  ros2_control_py::bind_ros2_control_test_assets::init_components_urdfs(
      ros2_control_test_assets);
  ros2_control_py::bind_ros2_control_test_assets::init_descriptions(
      ros2_control_test_assets);
  ros2_control_py::bind_rclcpp::init_rclcpp(rclcpp);
  ros2_control_py::bind_rclcpp::init_py_ref(rclcpp);
  ros2_control_py::bind_controller_interface::init_helpers(
      controller_interface);
  ros2_control_py::bind_controller_interface::init_controller_interface_base(
      controller_interface);
  ros2_control_py::bind_controller_interface::
      init_chainable_controller_interface(controller_interface);
  ros2_control_py::bind_controller_interface::init_controller_interface(
      controller_interface);
  ros2_control_py::bind_controller_manager::init_controller_spec(
      controller_manager);
  ros2_control_py::bind_controller_manager::init_controller_manager(
      controller_manager);
  ros2_control_py::bind_hardware_interface::init_sensor(hardware_interface);
  ros2_control_py::bind_hardware_interface::init_resource_manager(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_actuator(hardware_interface);
  ros2_control_py::bind_hardware_interface::init_component_parser(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::
      init_types_hardware_interface_return_values(hardware_interface);
  ros2_control_py::bind_hardware_interface::
      init_types_hardware_interface_type_values(hardware_interface);
  ros2_control_py::bind_hardware_interface::init_types_lifecycle_state_names(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_hardware_info(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_lexical_casts(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_sensor_interface(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_loaned_state_interface(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_actuator_interface(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_controller_info(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_handle(hardware_interface);
  ros2_control_py::bind_hardware_interface::init_system(hardware_interface);
  ros2_control_py::bind_hardware_interface::init_system_interface(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_loaned_command_interface(
      hardware_interface);
  ros2_control_py::bind_hardware_interface::init_hardware_component_info(
      hardware_interface);
  ros2_control_py::bind_transmission_interface::
      init_transmission_interface_exception(transmission_interface);
  ros2_control_py::bind_transmission_interface::init_transmission_loader(
      transmission_interface);
  ros2_control_py::bind_transmission_interface::init_exception(
      transmission_interface);
  ros2_control_py::bind_transmission_interface::
      init_four_bar_linkage_transmission_loader(transmission_interface);
  ros2_control_py::bind_transmission_interface::init_simple_transmission_loader(
      transmission_interface);
  ros2_control_py::bind_transmission_interface::init_transmission(
      transmission_interface);
  ros2_control_py::bind_transmission_interface::
      init_four_bar_linkage_transmission(transmission_interface);
  ros2_control_py::bind_transmission_interface::init_handle(
      transmission_interface);
  ros2_control_py::bind_transmission_interface::init_accessor(
      transmission_interface);
  ros2_control_py::bind_transmission_interface::init_simple_transmission(
      transmission_interface);
  ros2_control_py::bind_transmission_interface::init_differential_transmission(
      transmission_interface);
  ros2_control_py::bind_transmission_interface::
      init_differential_transmission_loader(transmission_interface);
}
