#include <pybind11/pybind11.h>
#include "hardware_interface/hardware_component_info_py.hpp"
#include "hardware_interface/hardware_info_py.hpp"
#include "hardware_interface/actuator_interface_py.hpp"
#include "hardware_interface/actuator_py.hpp"
#include "hardware_interface/sensor_interface_py.hpp"
#include "hardware_interface/sensor_py.hpp"
#include "hardware_interface/system_interface_py.hpp"
#include "hardware_interface/system_py.hpp"

PYBIND11_MODULE(hardware_interface_py, m)
{
  m.doc() = R"(
            Python bindings for ros2_control functionalities.
            )";

  // Provide custom function signatures
  py::options options;
  options.disable_function_signatures();

  // Construct module classes
  ros2_control_py::bind_hardware_interface::init_actuator(m);
  ros2_control_py::bind_hardware_interface::init_actuator_interface(m);
  ros2_control_py::bind_hardware_interface::init_hardware_component_info(m);
  ros2_control_py::bind_hardware_interface::init_hardware_info(m);
  ros2_control_py::bind_hardware_interface::init_sensor_interface(m);
  ros2_control_py::bind_hardware_interface::init_sensor(m);
  ros2_control_py::bind_hardware_interface::init_system_interface(m);
  ros2_control_py::bind_hardware_interface::init_system(m);
}
