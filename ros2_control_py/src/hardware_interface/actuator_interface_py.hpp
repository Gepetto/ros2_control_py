#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/actuator_interface.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

class PyActuatorInterface : public ActuatorInterface {
 public:
  using ActuatorInterface::ActuatorInterface;

  CallbackReturn on_init(const HardwareInfo& hardware_info) override {
    PYBIND11_OVERRIDE(CallbackReturn, ActuatorInterface, on_init,
                      hardware_info);
  }

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces()
      override {
    PYBIND11_OVERRIDE_PURE(std::vector<StateInterface::ConstSharedPtr>,
                           ActuatorInterface, export_state_interfaces,

    );
  }

  std::vector<CommandInterface::SharedPtr> on_export_command_interfaces()
      override {
    PYBIND11_OVERRIDE_PURE(std::vector<CommandInterface::SharedPtr>,
                           ActuatorInterface, on_export_command_interfaces,

    );
  }

  return_type prepare_command_mode_switch(
      const std::vector<std::string>& arg0,
      const std::vector<std::string>& arg1) override {
    PYBIND11_OVERRIDE(return_type, ActuatorInterface,
                      prepare_command_mode_switch, arg0, arg1);
  }

  return_type perform_command_mode_switch(
      const std::vector<std::string>& arg0,
      const std::vector<std::string>& arg1) override {
    PYBIND11_OVERRIDE(return_type, ActuatorInterface,
                      perform_command_mode_switch, arg0, arg1);
  }

  return_type read(const rclcpp::Time& time,
                   const rclcpp::Duration& period) override {
    PYBIND11_OVERRIDE_PURE(return_type, ActuatorInterface, read, time, period);
  }

  return_type write(const rclcpp::Time& time,
                    const rclcpp::Duration& period) override {
    PYBIND11_OVERRIDE_PURE(return_type, ActuatorInterface, write, time, period);
  }

  std::string get_name() const override {
    PYBIND11_OVERRIDE(std::string, ActuatorInterface, get_name,

    );
  }

  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ActuatorInterface, on_configure,
                      previous_state);
  }

  CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ActuatorInterface, on_cleanup,
                      previous_state);
  }

  CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ActuatorInterface, on_shutdown,
                      previous_state);
  }

  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ActuatorInterface, on_activate,
                      previous_state);
  }

  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ActuatorInterface, on_deactivate,
                      previous_state);
  }

  CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ActuatorInterface, on_error,
                      previous_state);
  }
};

class PubActuatorInterface : public ActuatorInterface {
 public:
  using ActuatorInterface::info_;
  using ActuatorInterface::lifecycle_state_;
};

inline void init_actuator_interface([[maybe_unused]] py::module& m) {
  py::class_<ActuatorInterface,
             rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
             PyActuatorInterface>(m, "ActuatorInterface")
      .def(py::init<>())
      .def("on_init", &ActuatorInterface::on_init)
      .def("on_export_state_interfaces",
           &ActuatorInterface::on_export_state_interfaces)
      .def("on_export_command_interfaces",
           &ActuatorInterface::on_export_command_interfaces)
      .def("prepare_command_mode_switch",
           &ActuatorInterface::prepare_command_mode_switch)
      .def("perform_command_mode_switch",
           &ActuatorInterface::perform_command_mode_switch)
      .def("read", &ActuatorInterface::read)
      .def("write", &ActuatorInterface::write)
      .def("get_name", &ActuatorInterface::get_name)
      .def("get_state", &ActuatorInterface::get_state)
      .def("set_state", &ActuatorInterface::set_state)
      .def("on_configure", &ActuatorInterface::on_configure)
      .def("on_cleanup", &ActuatorInterface::on_cleanup)
      .def("on_shutdown", &ActuatorInterface::on_shutdown)
      .def("on_activate", &ActuatorInterface::on_activate)
      .def("on_deactivate", &ActuatorInterface::on_deactivate)
      .def("on_error", &ActuatorInterface::on_error)
      .def_readwrite("info_", &PubActuatorInterface::info_)
      .def_readwrite("lifecycle_state_",
                     &PubActuatorInterface::lifecycle_state_);
}

}  // namespace ros2_control_py::bind_hardware_interface
