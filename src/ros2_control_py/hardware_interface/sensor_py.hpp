#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace ros2_control_py
{
namespace bind_hardware_interface
{
void init_sensor(py::module &hardware_interface_py);
}
}
