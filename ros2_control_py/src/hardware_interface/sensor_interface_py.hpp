#pragma once

// pybind11
#include <pybind11/pybind11.h>

// hardware_interface
#include <hardware_interface/sensor_interface.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;
using namespace hardware_interface;

class PySensorInterface : public SensorInterface {
 public:
  using SensorInterface::SensorInterface;

  CallbackReturn on_init(const HardwareInfo& hardware_info) override {
    PYBIND11_OVERRIDE(CallbackReturn, SensorInterface, on_init, hardware_info);
  }

  std::vector<StateInterface> export_state_interfaces() override {
    PYBIND11_OVERRIDE_PURE(
        std::vector<StateInterface>, SensorInterface, export_state_interfaces,

    );
  }

  return_type read(const rclcpp::Time& time,
                   const rclcpp::Duration& period) override {
    PYBIND11_OVERRIDE_PURE(return_type, SensorInterface, read, time, period);
  }

  std::string get_name() const override {
    PYBIND11_OVERRIDE(std::string, SensorInterface, get_name,

    );
  }

  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SensorInterface, on_configure,
                      previous_state);
  }

  CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SensorInterface, on_cleanup,
                      previous_state);
  }

  CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SensorInterface, on_shutdown,
                      previous_state);
  }

  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SensorInterface, on_activate,
                      previous_state);
  }

  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SensorInterface, on_deactivate,
                      previous_state);
  }

  CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, SensorInterface, on_error,
                      previous_state);
  }

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() {
    PYBIND11_OVERRIDE(std::vector<StateInterface::ConstSharedPtr>,
                      SensorInterface, on_export_state_interfaces);
  }
};

class PubSensorInterface : public SensorInterface {
 public:
  using SensorInterface::info_;
  using SensorInterface::lifecycle_state_;
};

inline void init_sensor_interface([[maybe_unused]] py::module& m) {
  py::class_<SensorInterface,
             rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
             PySensorInterface>(m, "SensorInterface")
      .def(py::init<>())
      .def("on_init", &SensorInterface::on_init)
      .def("on_export_state_interfaces",
           &SensorInterface::on_export_state_interfaces)
      .def("read", &SensorInterface::read)
      .def("get_name", &SensorInterface::get_name)
      .def("get_state", &SensorInterface::get_state)
      .def("set_state", &SensorInterface::set_state)
      .def("on_configure", &SensorInterface::on_configure)
      .def("on_cleanup", &SensorInterface::on_cleanup)
      .def("on_shutdown", &SensorInterface::on_shutdown)
      .def("on_activate", &SensorInterface::on_activate)
      .def("on_deactivate", &SensorInterface::on_deactivate)
      .def("on_error", &SensorInterface::on_error)
      .def_readwrite("info_", &PubSensorInterface::info_)
      .def_readwrite("lifecycle_state_", &PubSensorInterface::lifecycle_state_);
}

}  // namespace ros2_control_py::bind_hardware_interface
