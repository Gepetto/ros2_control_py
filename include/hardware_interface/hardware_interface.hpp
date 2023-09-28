// pybind11
#include <pybind11/pybind11.h>
// hardware_interface
#include <hardware_interface/actuator.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/controller_info.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_component_info.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/system.hpp>
#include <hardware_interface/system_interface.hpp>

namespace ros2_control_py::bind_hardware_interface {

namespace py = pybind11;

void init_hardware_interface(py::module &m);

}  // namespace ros2_control_py::bind_hardware_interface
