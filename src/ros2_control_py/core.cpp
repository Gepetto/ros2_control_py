// pybind11
#include <pybind11/pybind11.h>
// hardware_interface
#include <hardware_interface/hardware_interface.hpp>

PYBIND11_MODULE(core, m) {
  m.doc() = R"(
            Python bindings for ros2_control functionalities.
            )";

  // Provide custom function signatures
  pybind11::options options;
  options.disable_function_signatures();

  // Construct module classes
  ros2_control_py::bind_hardware_interface::init_hardware_interface(m);
}
