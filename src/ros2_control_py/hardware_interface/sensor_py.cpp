#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/sensor.hpp>
#include "sensor_py.hpp"

namespace ros2_control_py
{
namespace bind_hardware_interface
{
using namespace hardware_interface;

void init_sensor(py::module &hardware_interface_py)
{
  py::class_<Sensor>
      (hardware_interface_py,"Sensor")
      .def(py::init<>())
      .def("initialize",&Sensor::initialize)
      .def("configure",&Sensor::configure)
      .def("cleanup",&Sensor::cleanup)
      .def("shutdown",&Sensor::shutdown)
      .def("activate",&Sensor::activate)
      .def("deactivate",&Sensor::deactivate)
      .def("error",&Sensor::error)
      .def("export_state_interfaces",&Sensor::export_state_interfaces)
      .def("get_name",&Sensor::get_name)
      .def("get_state",&Sensor::get_state)
      .def("read",&Sensor::read);
}

}
}
