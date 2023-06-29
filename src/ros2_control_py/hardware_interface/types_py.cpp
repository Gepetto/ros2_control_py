#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include "types_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
void init_types(py::module &hardware_interface_py)
{
  py::enum_<hardware_interface::return_type>(
      hardware_interface_py,
      "return_type")
      .value("OK", hardware_interface::return_type::OK )
      .value("ERROR", hardware_interface::return_type::ERROR  )
      .export_values();

  hardware_interface_py.def("HW_IF_POSITION",
                            []() {
                              return std::string(hardware_interface::HW_IF_POSITION);
                            });
  hardware_interface_py.def("HW_IF_VELOCITY",
                            []() {
                              return std::string(hardware_interface::HW_IF_VELOCITY);
                            });
  hardware_interface_py.def("HW_IF_ACCELERATION",
                            []() {
                              return std::string(hardware_interface::HW_IF_ACCELERATION);
                            });

  hardware_interface_py.def("HW_IF_EFFORT",
                            []() {
                              return std::string(hardware_interface::HW_IF_EFFORT);
                            });

  hardware_interface_py.def("UNKNOWN",
                            []() {
                              return std::string(hardware_interface::lifecycle_state_names::UNKNOWN);
                            }
                           );
  hardware_interface_py.def("UNCONFIGURED",
                            []() {
                              return std::string(hardware_interface::lifecycle_state_names::UNCONFIGURED);
                            }
                            );
  hardware_interface_py.def("INACTIVE",
                            []() {
                              return std::string(hardware_interface::lifecycle_state_names::INACTIVE);
                            }
                            );
  hardware_interface_py.def("ACTIVE",
                            []() {
                              return std::string(hardware_interface::lifecycle_state_names::ACTIVE);
                            }
                            );
  hardware_interface_py.def("FINALIZED",
                            []() {
                              return std::string(hardware_interface::lifecycle_state_names::FINALIZED);
                            }
                            );
}
}
}
