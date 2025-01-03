#pragma once

// pybind11
#include <pybind11/pybind11.h>

// controller_interface
#include <controller_interface/controller_interface.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_controller_interface
{

namespace py = pybind11;
using namespace controller_interface;

class PyControllerInterface: public ControllerInterface {
 public:
  using ControllerInterface::ControllerInterface;

  InterfaceConfiguration command_interface_configuration() const override {
    PYBIND11_OVERRIDE_PURE(
        InterfaceConfiguration,
        ControllerInterface,
        command_interface_configuration,
        
    );
  }

  InterfaceConfiguration state_interface_configuration() const override {
    PYBIND11_OVERRIDE_PURE(
        InterfaceConfiguration,
        ControllerInterface,
        state_interface_configuration,
        
    );
  }


  CallbackReturn on_init() override {
    PYBIND11_OVERRIDE_PURE(
        CallbackReturn,
        ControllerInterface,
        on_init,
        
    );
  }

  return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override {
    PYBIND11_OVERRIDE_PURE(
        return_type,
        ControllerInterface,
        update,
        time,
        period
    );
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ControllerInterface,
        on_configure,
        previous_state
    );
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ControllerInterface,
        on_cleanup,
        previous_state
    );
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ControllerInterface,
        on_shutdown,
        previous_state
    );
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ControllerInterface,
        on_activate,
        previous_state
    );
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ControllerInterface,
        on_deactivate,
        previous_state
    );
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(
        CallbackReturn,
        ControllerInterface,
        on_error,
        previous_state
    );
  }
};

inline void init_controller_interface([[maybe_unused]] py::module &m)
{
  py::class_<ControllerInterface, ControllerInterfaceBase, PyControllerInterface>(m, "ControllerInterface")
      .def(py::init<>())
      .def("is_chainable", &ControllerInterface::is_chainable)
      .def("export_reference_interfaces", &ControllerInterface::export_reference_interfaces)
      .def("set_chained_mode", &ControllerInterface::set_chained_mode)
      .def("is_in_chained_mode", &ControllerInterface::is_in_chained_mode)
      .def("command_interface_configuration", &ControllerInterface::command_interface_configuration)
      .def("state_interface_configuration", &ControllerInterface::state_interface_configuration)
      .def("init", &ControllerInterface::init)
      .def("on_init", &ControllerInterface::on_init)
      .def("update", &ControllerInterface::update)
      .def("on_configure", &ControllerInterface::on_configure)
      .def("on_cleanup", &ControllerInterface::on_cleanup)
      .def("on_shutdown", &ControllerInterface::on_shutdown)
      .def("on_activate", &ControllerInterface::on_activate)
      .def("on_deactivate", &ControllerInterface::on_deactivate)
      .def("on_error", &ControllerInterface::on_error);
}

}
