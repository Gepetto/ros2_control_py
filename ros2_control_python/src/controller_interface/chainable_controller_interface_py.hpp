#pragma once

// pybind11
#include <pybind11/pybind11.h>

// controller_interface
#include <controller_interface/chainable_controller_interface.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_controller_interface
{

namespace py = pybind11;
using namespace controller_interface;

class PyChainableControllerInterface: public ChainableControllerInterface {
 public:
  using ChainableControllerInterface::ChainableControllerInterface;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override {
    PYBIND11_OVERRIDE_PURE(
        std::vector<hardware_interface::CommandInterface>,
        ChainableControllerInterface,
        on_export_reference_interfaces,
        
    );
  }

  bool on_set_chained_mode(bool chained_mode) override {
    PYBIND11_OVERRIDE(
        bool,
        ChainableControllerInterface,
        on_set_chained_mode,
        chained_mode
    );
  }

  return_type update_reference_from_subscribers(
      const rclcpp::Time & time, const rclcpp::Duration & period) override {
    PYBIND11_OVERRIDE_PURE(
        return_type,
        ChainableControllerInterface,
        update_reference_from_subscribers,
        time, period
        
    );
  }

  return_type update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration& period) override {
    PYBIND11_OVERRIDE_PURE(
        return_type,
        ChainableControllerInterface,
        update_and_write_commands,
        time,
        period
    );
  }

  InterfaceConfiguration command_interface_configuration() const override {
    PYBIND11_OVERRIDE_PURE(
        InterfaceConfiguration,
        ChainableControllerInterface,
        command_interface_configuration,
        
    );
  }

  InterfaceConfiguration state_interface_configuration() const override {
    PYBIND11_OVERRIDE_PURE(
        InterfaceConfiguration,
        ChainableControllerInterface,
        state_interface_configuration,
        
    );
  }

 
  CallbackReturn on_init() override {
    PYBIND11_OVERRIDE_PURE(
        CallbackReturn,
        ChainableControllerInterface,
        on_init,
        
    );
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ChainableControllerInterface,
        on_configure,
        previous_state
    );
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ChainableControllerInterface,
        on_cleanup,
        previous_state
    );
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ChainableControllerInterface,
        on_shutdown,
        previous_state
    );
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ChainableControllerInterface,
        on_activate,
        previous_state
    );
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ChainableControllerInterface,
        on_deactivate,
        previous_state
    );
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ChainableControllerInterface,
        on_error,
        previous_state
    );
  }
};

class PubChainableControllerInterface: public ChainableControllerInterface {
 public:
  using ChainableControllerInterface::on_export_reference_interfaces;
  using ChainableControllerInterface::on_set_chained_mode;
  using ChainableControllerInterface::update_reference_from_subscribers;
  using ChainableControllerInterface::update_and_write_commands;
  using ChainableControllerInterface::reference_interfaces_;
};

inline void init_chainable_controller_interface([[maybe_unused]] py::module &m)
{
  py::class_<ChainableControllerInterface, ControllerInterfaceBase, PyChainableControllerInterface>(m, "ChainableControllerInterface")
      .def(py::init<>())
      .def("update", &ChainableControllerInterface::update)
      .def("is_chainable", &ChainableControllerInterface::is_chainable)
      .def("export_reference_interfaces", &ChainableControllerInterface::export_reference_interfaces)
      .def("set_chained_mode", &ChainableControllerInterface::set_chained_mode)
      .def("is_in_chained_mode", &ChainableControllerInterface::is_in_chained_mode)
      .def("on_export_reference_interfaces", &PubChainableControllerInterface::on_export_reference_interfaces)
      .def("on_set_chained_mode", &PubChainableControllerInterface::on_set_chained_mode)
      .def("update_reference_from_subscribers", &PubChainableControllerInterface::update_reference_from_subscribers)
      .def("update_and_write_commands", &PubChainableControllerInterface::update_and_write_commands)
      .def("command_interface_configuration", &ChainableControllerInterface::command_interface_configuration)
      .def("state_interface_configuration", &ChainableControllerInterface::state_interface_configuration)
      .def("init", &ChainableControllerInterface::init)
      .def("on_init", &ChainableControllerInterface::on_init)
      .def("on_configure", &ChainableControllerInterface::on_configure)
      .def("on_cleanup", &ChainableControllerInterface::on_cleanup)
      .def("on_shutdown", &ChainableControllerInterface::on_shutdown)
      .def("on_activate", &ChainableControllerInterface::on_activate)
      .def("on_deactivate", &ChainableControllerInterface::on_deactivate)
      .def("on_error", &ChainableControllerInterface::on_error)
      .def_readwrite("reference_interfaces_", &PubChainableControllerInterface::reference_interfaces_);
}

}
