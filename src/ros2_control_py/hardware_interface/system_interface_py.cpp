#include <hardware_interface/system_interface.hpp>
#include "system_interface_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

/// Trampoline class for pure virtual ActuatorInterface
class PySystemInterface: public SystemInterface {
 public:
  using SystemInterface::SystemInterface;

  std::vector<StateInterface> export_state_interfaces() override {
    PYBIND11_OVERRIDE_PURE(
        /* Return type */
        std::vector<StateInterface>,
        /* Parent class */
        SystemInterface,
        /* Name of function in C++ (must match Python name) */
        export_state_interfaces
        /* Argument(s) */
    );
  }

  std::vector<CommandInterface> export_command_interfaces() override {
    PYBIND11_OVERRIDE_PURE(
        /* Return type */
        std::vector<CommandInterface>,
        /* Parent class */
        SystemInterface,
        /* Name of function in C++ (must match Python name) */
        export_command_interfaces
        /* Argument(s) */
    );
  }

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    PYBIND11_OVERRIDE_PURE(
        /* Return type */
        return_type,
        /* Parent class */
        SystemInterface,
        /* Name of function in C++ (must match Python name) */
        read,
        /* Argument(s) */
        time,
        period
    );
  }

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    PYBIND11_OVERRIDE_PURE(
        /* Return type */
        return_type,
        /* Parent class */
        SystemInterface,
        /* Name of function in C++ (must match Python name) */
        write,
        /* Argument(s) */
        time,
        period
    );
  }

};

void init_system_interface(py::module &hardware_interface_py)
{
  py::class_<SystemInterface,PySystemInterface>
      (hardware_interface_py,
       "SystemInterface")
      .def(py::init<>())
      .def("on_init",
           &SystemInterface::on_init)
      .def("export_state_interfaces",
           &SystemInterface::export_state_interfaces)
      .def("export_command_interfaces",
           &SystemInterface::export_command_interfaces)
      .def("prepare_command_mode_switch",
           &SystemInterface::prepare_command_mode_switch)
      .def("perform_command_mode_switch",
           &SystemInterface::perform_command_mode_switch)
      .def("read",
           &SystemInterface::read)
      .def("write",
           &SystemInterface::write)
      .def("get_name",
           &SystemInterface::get_name)
      .def("get_state",
           &SystemInterface::get_state)
      .def("set_state",
           &SystemInterface::set_state);
}
}
}