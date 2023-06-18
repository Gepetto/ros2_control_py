#include <hardware_interface/sensor_interface.hpp>
#include "sensor_interface_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

/// Trampoline class for pure virtual ActuatorInterface
class PySensorInterface: public SensorInterface {
 public:
  using SensorInterface::SensorInterface;

  std::vector<StateInterface> export_state_interfaces() override {
    PYBIND11_OVERRIDE_PURE(
        /* Return type */
        std::vector<StateInterface>,
        /* Parent class */
        SensorInterface,
        /* Name of function in C++ (must match Python name) */
        export_state_interfaces
        /* Argument(s) */
    );
  }

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    PYBIND11_OVERRIDE_PURE(
        /* Return type */
        return_type,
        /* Parent class */
        SensorInterface,
        /* Name of function in C++ (must match Python name) */
        read,
        /* Argument(s) */
        time,
        period
    );
  }

};

void init_sensor_interface(py::module &hardware_interface_py)
{
  py::class_<SensorInterface,PySensorInterface>
      (hardware_interface_py,
       "SensorInterface")
      .def(py::init<>())
      .def("on_init",
           &SensorInterface::on_init)
      .def("export_state_interfaces",
           &SensorInterface::export_state_interfaces)
      .def("read",
           &SensorInterface::read)
      .def("get_name",
           &SensorInterface::get_name)
      .def("get_state",
           &SensorInterface::get_state)
      .def("set_state",
           &SensorInterface::set_state);
}
}
}
