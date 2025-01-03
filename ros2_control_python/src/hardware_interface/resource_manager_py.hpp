#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/resource_manager.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface
{

namespace py = pybind11;
using namespace hardware_interface;

inline void init_resource_manager([[maybe_unused]] py::module &m)
{
  py::class_<HardwareReadWriteStatus>(m, "HardwareReadWriteStatus")
      .def(py::init<>())
      .def_readwrite("ok", &HardwareReadWriteStatus::ok)
      .def_readwrite("failed_hardware_names", &HardwareReadWriteStatus::failed_hardware_names);

  py::class_<ResourceManager>(m, "ResourceManager")
      .def(py::init<rclcpp::node_interfaces::NodeClockInterface::SharedPtr ,
           rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr >())
      .def(py::init<const std::string&,
           rclcpp::node_interfaces::NodeClockInterface::SharedPtr ,
           rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr,
           bool, const unsigned int>())
      .def("claim_state_interface", &ResourceManager::claim_state_interface)
      .def("state_interface_keys", &ResourceManager::state_interface_keys)
      .def("available_state_interfaces", &ResourceManager::available_state_interfaces)
      .def("state_interface_is_available", &ResourceManager::state_interface_is_available)
      .def("import_controller_reference_interfaces", &ResourceManager::import_controller_reference_interfaces)
      .def("get_controller_reference_interface_names", &ResourceManager::get_controller_reference_interface_names)
      .def("make_controller_reference_interfaces_available", &ResourceManager::make_controller_reference_interfaces_available)
      .def("make_controller_reference_interfaces_unavailable", &ResourceManager::make_controller_reference_interfaces_unavailable)
      .def("remove_controller_reference_interfaces", &ResourceManager::remove_controller_reference_interfaces)
      .def("cache_controller_to_hardware", &ResourceManager::cache_controller_to_hardware)
      .def("get_cached_controllers_to_hardware", &ResourceManager::get_cached_controllers_to_hardware)
      .def("command_interface_is_claimed", &ResourceManager::command_interface_is_claimed)
      .def("claim_command_interface", &ResourceManager::claim_command_interface)
      .def("command_interface_keys", &ResourceManager::command_interface_keys)
      .def("available_command_interfaces", &ResourceManager::available_command_interfaces)
      .def("command_interface_is_available", &ResourceManager::command_interface_is_available)
      .def("actuator_components_size", &ResourceManager::actuator_components_size)
      .def("sensor_components_size", &ResourceManager::sensor_components_size)
      .def("system_components_size", &ResourceManager::system_components_size)
      .def("import_component", [](ResourceManager& self, std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo& hardware_info) { return self.import_component(std::move(actuator), hardware_info); })
      .def("import_component", [](ResourceManager& self, std::unique_ptr<SensorInterface> sensor, const HardwareInfo& hardware_info) { return self.import_component(std::move(sensor), hardware_info); })
      .def("import_component", [](ResourceManager& self, std::unique_ptr<SystemInterface> system, const HardwareInfo& hardware_info) { return self.import_component(std::move(system), hardware_info); })
      .def("get_components_status", &ResourceManager::get_components_status)
      .def("prepare_command_mode_switch", &ResourceManager::prepare_command_mode_switch)
      .def("perform_command_mode_switch", &ResourceManager::perform_command_mode_switch)
      .def("set_component_state", &ResourceManager::set_component_state)
      .def("read", &ResourceManager::read)
      .def("write", &ResourceManager::write)
      .def("are_components_initialized", &ResourceManager::are_components_initialized)
      .def("command_interface_exists", &ResourceManager::command_interface_exists)
      .def("state_interface_exists", &ResourceManager::state_interface_exists);
}

}
