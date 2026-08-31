#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/system_interface.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

class PySystemInterface : public SystemInterface {
 public:
  using SystemInterface::SystemInterface;

  CallbackReturn on_init(const HardwareInfo& hardware_info) override {
    PYBIND11_OVERRIDE(CallbackReturn, SystemInterface, on_init, hardware_info);
  }

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces()
      override {
    PYBIND11_OVERRIDE(std::vector<StateInterface::ConstSharedPtr>,
                      SystemInterface, on_export_state_interfaces,

    );
  }

  std::vector<CommandInterface> export_command_interfaces() override {
    PYBIND11_OVERRIDE_PURE(std::vector<CommandInterface>, SystemInterface,
                           export_command_interfaces,

    );
  }

  return_type prepare_command_mode_switch(
      const std::vector<std::string>& arg0,
      const std::vector<std::string>& arg1) override {
    PYBIND11_OVERRIDE(return_type, SystemInterface, prepare_command_mode_switch,
                      arg0, arg1);
  }

  return_type perform_command_mode_switch(
      const std::vector<std::string>& arg0,
      const std::vector<std::string>& arg1) override {
    PYBIND11_OVERRIDE(return_type, SystemInterface, perform_command_mode_switch,
                      arg0, arg1);
  }

  return_type read(const rclcpp::Time& time,
                   const rclcpp::Duration& period) override {
    PYBIND11_OVERRIDE_PURE(return_type, SystemInterface, read, time, period);
  }

  return_type write(const rclcpp::Time& time,
                    const rclcpp::Duration& period) override {
    PYBIND11_OVERRIDE_PURE(return_type, SystemInterface, write, time, period);
  }

  std::string get_name() const override {
    PYBIND11_OVERRIDE(std::string, SystemInterface, get_name,

    );
  }

  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SystemInterface, on_configure,
                      previous_state);
  }

  CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SystemInterface, on_cleanup,
                      previous_state);
  }

  CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SystemInterface, on_shutdown,
                      previous_state);
  }

  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SystemInterface, on_activate,
                      previous_state);
  }

  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SystemInterface, on_deactivate,
                      previous_state);
  }

  CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SystemInterface, on_error,
                      previous_state);
  }
};

class PubSystemInterface : public SystemInterface {
 public:
  using SystemInterface::info_;
  using SystemInterface::lifecycle_state_;
};

inline void init_system_interface([[maybe_unused]] py::module& m) {
  py::class_<SystemInterface,
             rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
             PySystemInterface>(m, "SystemInterface")
      .def(py::init<>())
      .def("on_init", &SystemInterface::on_init)
      .def("on_export_state_interfaces",
           &SystemInterface::on_export_state_interfaces)
      .def("on_export_command_interfaces",
           &SystemInterface::on_export_command_interfaces)
      .def("prepare_command_mode_switch",
           &SystemInterface::prepare_command_mode_switch)
      .def("perform_command_mode_switch",
           &SystemInterface::perform_command_mode_switch)
      .def("read", &SystemInterface::read)
      .def("write", &SystemInterface::write)
      .def("get_name", &SystemInterface::get_name)
      .def("get_state", &SystemInterface::get_state)
      .def("set_state", &SystemInterface::set_state)
      .def("on_configure", &SystemInterface::on_configure)
      .def("on_cleanup", &SystemInterface::on_cleanup)
      .def("on_shutdown", &SystemInterface::on_shutdown)
      .def("on_activate", &SystemInterface::on_activate)
      .def("on_deactivate", &SystemInterface::on_deactivate)
      .def("on_error", &SystemInterface::on_error)
      .def_readwrite("info_", &PubSystemInterface::info_)
      .def_readwrite("lifecycle_state_", &PubSystemInterface::lifecycle_state_);
}

}  // namespace ros2_control_py::bind_hardware_interface
