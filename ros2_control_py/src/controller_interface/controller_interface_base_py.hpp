#pragma once

// pybind11
#include <pybind11/pybind11.h>

// controller_interface
#include <controller_interface/controller_interface_base.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_controller_interface {

namespace py = pybind11;
using namespace controller_interface;

class PyControllerInterfaceBase : public ControllerInterfaceBase {
 public:
  using ControllerInterfaceBase::ControllerInterfaceBase;

  InterfaceConfiguration command_interface_configuration() const override {
    PYBIND11_OVERRIDE_PURE(InterfaceConfiguration, ControllerInterfaceBase,
                           command_interface_configuration,

    );
  }

  InterfaceConfiguration state_interface_configuration() const override {
    PYBIND11_OVERRIDE_PURE(InterfaceConfiguration, ControllerInterfaceBase,
                           state_interface_configuration,

    );
  }

  CallbackReturn on_init() override {
    PYBIND11_OVERRIDE_PURE(CallbackReturn, ControllerInterfaceBase, on_init,

    );
  }

  return_type update(const rclcpp::Time& time,
                     const rclcpp::Duration& period) override {
    PYBIND11_OVERRIDE_PURE(return_type, ControllerInterfaceBase, update, time,
                           period);
  }

  bool is_chainable() const override {
    PYBIND11_OVERRIDE_PURE(bool, ControllerInterfaceBase, is_chainable,

    );
  }

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  export_state_interfaces() override {
    PYBIND11_OVERRIDE_PURE(
        std::vector<hardware_interface::StateInterface::ConstSharedPtr>,
        ControllerInterfaceBase, export_reference_interfaces,

    );
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  export_reference_interfaces() override {
    PYBIND11_OVERRIDE_PURE(
        std::vector<hardware_interface::CommandInterface::SharedPtr>,
        ControllerInterfaceBase, export_reference_interfaces,

    );
  }

  bool set_chained_mode(bool chained_mode) override {
    PYBIND11_OVERRIDE_PURE(bool, ControllerInterfaceBase, set_chained_mode,
                           chained_mode);
  }

  bool is_in_chained_mode() const override {
    PYBIND11_OVERRIDE_PURE(bool, ControllerInterfaceBase, is_in_chained_mode,

    );
  }

  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ControllerInterfaceBase, on_configure,
                      previous_state);
  }

  CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ControllerInterfaceBase, on_cleanup,
                      previous_state);
  }

  CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ControllerInterfaceBase, on_shutdown,
                      previous_state);
  }

  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ControllerInterfaceBase, on_activate,
                      previous_state);
  }

  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ControllerInterfaceBase, on_deactivate,
                      previous_state);
  }

  CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn, ControllerInterfaceBase, on_error,
                      previous_state);
  }
};

inline void init_controller_interface_base([[maybe_unused]] py::module& m) {
  py::enum_<return_type>(m, "return_type")
      .value("OK", return_type::OK)
      .value("ERROR", return_type::ERROR)
      .export_values();

  py::enum_<interface_configuration_type>(m, "interface_configuration_type")
      .value("ALL", interface_configuration_type::ALL)
      .value("INDIVIDUAL", interface_configuration_type::INDIVIDUAL)
      .value("NONE", interface_configuration_type::NONE)
      .export_values();
  py::class_<InterfaceConfiguration>(m, "InterfaceConfiguration")
      .def(py::init<>())
      .def_readwrite("type", &InterfaceConfiguration::type)
      .def_readwrite("names", &InterfaceConfiguration::names);

  py::class_<ControllerInterfaceBase,
             rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
             PyControllerInterfaceBase>(m, "ControllerInterfaceBase")
      .def(py::init<>())
      .def("command_interface_configuration",
           &ControllerInterfaceBase::command_interface_configuration)
      .def("state_interface_configuration",
           &ControllerInterfaceBase::state_interface_configuration)
      .def("release_interfaces", &ControllerInterfaceBase::release_interfaces)
      .def("init",
           [](ControllerInterfaceBase& self, const std::string& controller_name,
              const std::string& urdf, unsigned int cm_update_rate,
              const std::string& namespace_,
              const rclcpp::NodeOptions& node_options) {
             return self.init(controller_name, urdf, cm_update_rate, namespace_,
                              node_options);
           })
      .def("configure", &ControllerInterfaceBase::configure)
      .def("on_init", &ControllerInterfaceBase::on_init)
      .def("update", &ControllerInterfaceBase::update)
      .def("trigger_update", &ControllerInterfaceBase::trigger_update)
      .def("get_node",
           [](ControllerInterfaceBase& self) { return self.get_node(); })
      .def("get_update_rate", &ControllerInterfaceBase::get_update_rate)
      .def("is_chainable", &ControllerInterfaceBase::is_chainable)
      .def("export_reference_interfaces",
           &ControllerInterfaceBase::export_reference_interfaces)
      .def("set_chained_mode", &ControllerInterfaceBase::set_chained_mode)
      .def("is_in_chained_mode", &ControllerInterfaceBase::is_in_chained_mode)
      .def("on_configure", &ControllerInterfaceBase::on_configure)
      .def("on_cleanup", &ControllerInterfaceBase::on_cleanup)
      .def("on_shutdown", &ControllerInterfaceBase::on_shutdown)
      .def("on_activate", &ControllerInterfaceBase::on_activate)
      .def("on_deactivate", &ControllerInterfaceBase::on_deactivate)
      .def("on_error", &ControllerInterfaceBase::on_error);
}

}  // namespace ros2_control_py::bind_controller_interface
