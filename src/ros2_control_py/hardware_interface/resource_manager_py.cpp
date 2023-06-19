#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include "actuator_interface_py.hpp"
#include "resource_manager_py.hpp"

namespace ros2_control_py
{
  namespace bind_hardware_interface
  {
    using namespace hardware_interface;

    void init_resoure_manager(py::module &hardware_interface_py)
    {
      py::class_<ResourceManager>
        (hardware_interface_py,"ResourceManager")
        .def(py::init<>())
        .def(py::init<const std::string &,
             bool, bool>())
        .def("load_urdf",&ResourceManager::load_urdf)
        .def("claim_state_interface",&ResourceManager::claim_state_interface)
        .def("state_interface_keys", &ResourceManager::state_interface_keys)
        .def("available_state_interfaces",
             &ResourceManager::available_state_interfaces)
        .def("state_interface_exists",
             &ResourceManager::state_interface_exists)
        .def("state_interface_is_available",
             &ResourceManager::state_interface_is_available)
        .def("import_controller_reference_interfaces",
             &ResourceManager::import_controller_reference_interfaces)
        .def("get_controller_reference_interface_names",
             &ResourceManager::get_controller_reference_interface_names)
        .def("make_controller_reference_interfaces_available",
             &ResourceManager::make_controller_reference_interfaces_available)
        .def("make_controller_reference_interfaces_unavailable",
             &ResourceManager::make_controller_reference_interfaces_unavailable)
        .def("remove_controller_reference_interfaces",
             &ResourceManager::remove_controller_reference_interfaces)
        .def("command_interface_is_claimed",
             &ResourceManager::command_interface_is_claimed)
        .def("command_interface_keys",
             &ResourceManager::command_interface_keys)
        .def("available_command_interfaces",
             &ResourceManager::command_interface_keys)
        .def("command_interface_exists",
             &ResourceManager::command_interface_exists)
        .def("command_interface_is_available",
             &ResourceManager::command_interface_is_available)
        .def("actuator_components_size",
             &ResourceManager::actuator_components_size)
        .def("sensor_components_size",
             &ResourceManager::sensor_components_size)
        .def("system_components_size",
             &ResourceManager::system_components_size)
        // .def("import_component",
        //      static_cast<void(ResourceManager::*)
        //      ( std::unique_ptr<ActuatorInterface>,
        //        const HardwareInfo & )>
        //      (&ResourceManager::import_component),
        //      "Import a hardware component node listed in the URDF"
        //      )
        .def("get_components_status",
             &ResourceManager::get_components_status)
        .def("prepare_command_mode_switch",
             &ResourceManager::prepare_command_mode_switch)
        .def("set_component_state",
             &ResourceManager::set_component_state)
        .def("read",&ResourceManager::read)
        .def("write",&ResourceManager::write)
        .def("activate_all_components",&ResourceManager::activate_all_components);


    }
  }
}
