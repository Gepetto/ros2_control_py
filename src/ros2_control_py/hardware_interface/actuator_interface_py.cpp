#include <hardware_interface/actuator_interface.hpp>
#include "actuator_interface_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

/// Trampoline class for pure virtual ActuatorInterface
class PyActuatorInterface: public ActuatorInterface {
 public:
  using ActuatorInterface::ActuatorInterface;

  std::vector<StateInterface> export_state_interfaces() override {
    PYBIND11_OVERRIDE_PURE(
        /* Return type */
        std::vector<StateInterface>,
        /* Parent class */
        ActuatorInterface,
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
        ActuatorInterface,
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
        ActuatorInterface,
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
        ActuatorInterface,
        /* Name of function in C++ (must match Python name) */
        write,
        /* Argument(s) */
        time,
        period
    );
  }



};
void init_actuator_interface(py::module &hardware_interface_py)
{
  py::class_<ActuatorInterface,PyActuatorInterface>
      (hardware_interface_py,
       "ActuatorInterface")
      .def(py::init<>())
      .def("on_init",
           &ActuatorInterface::on_init)
      .def("export_state_interfaces",
           &ActuatorInterface::export_state_interfaces)
      .def("export_command_interfaces",
           &ActuatorInterface::export_command_interfaces)
      .def("prepare_command_mode_switch",
           &ActuatorInterface::prepare_command_mode_switch)
      .def("perform_command_mode_switch",
           &ActuatorInterface::perform_command_mode_switch)
      .def("read",
           &ActuatorInterface::read)
      .def("write",
           &ActuatorInterface::write)
      .def("get_name",
           &ActuatorInterface::get_name)
      .def("get_state",
           &ActuatorInterface::get_state)
      .def("set_state",
           &ActuatorInterface::set_state);

}
}
}
