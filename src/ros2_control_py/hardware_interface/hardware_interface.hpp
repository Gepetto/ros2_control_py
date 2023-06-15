#include <pybind11/pybind11.h>

namespace py = pybind11;

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/hardware_component_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/sensor.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/system.hpp>

namespace ros2_control_py
{
namespace bind_hardware_interface
{
void init_hardware_interface(py::module &m);
}
}
