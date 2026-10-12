#pragma once

// pybind11
#include <pybind11/pybind11.h>

// controller_manager
#include <controller_manager/controller_manager.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_controller_manager {

namespace py = pybind11;
using namespace controller_manager;

class PubControllerManager : public ControllerManager {
 public:
  using ControllerManager::activate_controllers;
  using ControllerManager::activate_controllers_asap;
  using ControllerManager::add_controller_impl;
  using ControllerManager::chained_controllers_configuration_;
  using ControllerManager::configure_controller_service_cb;
  using ControllerManager::deactivate_controllers;
  using ControllerManager::init_services;
  using ControllerManager::list_controller_types_srv_cb;
  using ControllerManager::list_controllers_srv_cb;
  using ControllerManager::list_hardware_components_srv_cb;
  using ControllerManager::list_hardware_interfaces_srv_cb;
  using ControllerManager::load_controller_service_cb;
  using ControllerManager::manage_switch;
  using ControllerManager::reload_controller_libraries_service_cb;
  using ControllerManager::set_hardware_component_state_srv_cb;
  using ControllerManager::switch_chained_mode;
  using ControllerManager::switch_controller_service_cb;
  using ControllerManager::unload_controller_service_cb;
  using ControllerManager::update_loop_counter_;
  using ControllerManager::update_rate_;
};

inline void init_controller_manager([[maybe_unused]] py::module& m) {
  m.def("get_cm_node_options", &get_cm_node_options);
  py::class_<ControllerManager, rclcpp::Node,
             std::shared_ptr<ControllerManager>>(m, "ControllerManager")
      .def(py::init<std::unique_ptr<hardware_interface::ResourceManager>,
                    std::shared_ptr<rclcpp::Executor>>())
      .def(py::init<std::unique_ptr<hardware_interface::ResourceManager>,
                    std::shared_ptr<rclcpp::Executor>, const std::string&>())
      .def(py::init<std::unique_ptr<hardware_interface::ResourceManager>,
                    std::shared_ptr<rclcpp::Executor>, const std::string&,
                    const std::string&>())
      .def(py::init<std::unique_ptr<hardware_interface::ResourceManager>,
                    std::shared_ptr<rclcpp::Executor>, const std::string&,
                    const std::string&, const rclcpp::NodeOptions&>())
      .def(py::init<std::shared_ptr<rclcpp::Executor>>())
      .def(py::init<std::shared_ptr<rclcpp::Executor>, const std::string&>())
      .def(py::init<std::shared_ptr<rclcpp::Executor>, const std::string&,
                    const std::string&>())
      .def(py::init<std::shared_ptr<rclcpp::Executor>, const std::string&,
                    const std::string&, const rclcpp::NodeOptions&>())
      .def("robot_description_callback",
           &ControllerManager::robot_description_callback)
      .def("init_resource_manager", &ControllerManager::init_resource_manager)
      .def("load_controller",
           [](ControllerManager& self, const std::string& controller_name,
              const std::string& controller_type) {
             return self.load_controller(controller_name, controller_type);
           })
      .def("load_controller",
           [](ControllerManager& self, const std::string& controller_name) {
             return self.load_controller(controller_name);
           })
      .def("unload_controller", &ControllerManager::unload_controller)
      .def("get_loaded_controllers", &ControllerManager::get_loaded_controllers)
      .def("configure_controller", &ControllerManager::configure_controller)
      .def(
          "switch_controller",
          [](ControllerManager& self,
             const std::vector<std::string>& start_controllers,
             const std::vector<std::string>& stop_controllers, int strictness) {
            return self.switch_controller(start_controllers, stop_controllers,
                                          strictness);
          })
      .def("switch_controller",
           [](ControllerManager& self,
              const std::vector<std::string>& start_controllers,
              const std::vector<std::string>& stop_controllers, int strictness,
              bool activate_asap) {
             return self.switch_controller(start_controllers, stop_controllers,
                                           strictness, activate_asap);
           })
      .def("switch_controller",
           [](ControllerManager& self,
              const std::vector<std::string>& start_controllers,
              const std::vector<std::string>& stop_controllers, int strictness,
              bool activate_asap, const rclcpp::Duration& timeout) {
             return self.switch_controller(start_controllers, stop_controllers,
                                           strictness, activate_asap, timeout);
           })
      .def("read", &ControllerManager::read)
      .def("update", &ControllerManager::update)
      .def("write", &ControllerManager::write)
      .def("get_update_rate", &ControllerManager::get_update_rate)
      .def("init_services", &PubControllerManager::init_services)
      .def("add_controller_impl", &PubControllerManager::add_controller_impl)
      .def("manage_switch", &PubControllerManager::manage_switch)
      .def("deactivate_controllers",
           &PubControllerManager::deactivate_controllers)
      .def("switch_chained_mode", &PubControllerManager::switch_chained_mode)
      .def("activate_controllers", &PubControllerManager::activate_controllers)
      .def("activate_controllers_asap",
           &PubControllerManager::activate_controllers_asap)
      .def("list_controllers_srv_cb",
           &PubControllerManager::list_controllers_srv_cb)
      .def("list_hardware_interfaces_srv_cb",
           &PubControllerManager::list_hardware_interfaces_srv_cb)
      .def("load_controller_service_cb",
           &PubControllerManager::load_controller_service_cb)
      .def("configure_controller_service_cb",
           &PubControllerManager::configure_controller_service_cb)
      .def("reload_controller_libraries_service_cb",
           &PubControllerManager::reload_controller_libraries_service_cb)
      .def("switch_controller_service_cb",
           &PubControllerManager::switch_controller_service_cb)
      .def("unload_controller_service_cb",
           &PubControllerManager::unload_controller_service_cb)
      .def("list_controller_types_srv_cb",
           &PubControllerManager::list_controller_types_srv_cb)
      .def("list_hardware_components_srv_cb",
           &PubControllerManager::list_hardware_components_srv_cb)
      .def("set_hardware_component_state_srv_cb",
           &PubControllerManager::set_hardware_component_state_srv_cb)
      .def_readwrite("update_loop_counter_",
                     &PubControllerManager::update_loop_counter_)
      .def_readwrite("update_rate_", &PubControllerManager::update_rate_)
      .def_readwrite("chained_controllers_configuration_",
                     &PubControllerManager::chained_controllers_configuration_);
}

}  // namespace ros2_control_py::bind_controller_manager
