#pragma once

// pybind11
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
// STL
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <map>
// ros2_control_py
#include <joint_limits/joint_limits_rosparam.hpp>
#include <joint_limits/joint_limits.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>
#include <controller_interface/helpers.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <controller_manager/controller_spec.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/sensor.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/actuator.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/controller_info.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/hardware_component_info.hpp>
#include <transmission_interface/transmission_interface_exception.hpp>
#include <transmission_interface/transmission_loader.hpp>
#include <transmission_interface/exception.hpp>
#include <transmission_interface/four_bar_linkage_transmission_loader.hpp>
#include <transmission_interface/simple_transmission_loader.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/four_bar_linkage_transmission.hpp>
#include <transmission_interface/handle.hpp>
#include <transmission_interface/accessor.hpp>
#include <transmission_interface/simple_transmission.hpp>
#include <transmission_interface/differential_transmission.hpp>
#include <transmission_interface/differential_transmission_loader.hpp>
#include <rclcpp/py_ref_py.hpp>
#include <rclcpp/rclcpp_py.hpp>

namespace PYBIND11_NAMESPACE {
namespace detail {
using namespace transmission_interface;
using namespace ros2_control_test_assets;
using namespace joint_limits;
using namespace hardware_interface::lifecycle_state_names;
using namespace hardware_interface;
using namespace controller_manager;
using namespace controller_interface;
}
}

PYBIND11_MAKE_OPAQUE(std::vector<ActuatorHandle>);
PYBIND11_MAKE_OPAQUE(std::vector<double>);
PYBIND11_MAKE_OPAQUE(std::vector<InterfaceInfo>);
PYBIND11_MAKE_OPAQUE(std::vector<std::string>);
PYBIND11_MAKE_OPAQUE(std::unordered_map<std::string, std::string>);
PYBIND11_MAKE_OPAQUE(std::vector<std::vector<std::string>>);
PYBIND11_MAKE_OPAQUE(std::vector<ComponentInfo>);
PYBIND11_MAKE_OPAQUE(std::unordered_map<std::string, HardwareComponentInfo>);
PYBIND11_MAKE_OPAQUE(std::vector<HardwareInfo>);
PYBIND11_MAKE_OPAQUE(std::vector<JointHandle>);
PYBIND11_MAKE_OPAQUE(std::vector<ControllerSpec>);
PYBIND11_MAKE_OPAQUE(std::vector<JointInfo>);
PYBIND11_MAKE_OPAQUE(std::vector<TransmissionInfo>);
PYBIND11_MAKE_OPAQUE(std::vector<ActuatorInfo>);

namespace PYBIND11_NAMESPACE {
namespace detail {
template <typename T>
struct type_caster<std::unique_ptr<T>> {
 public:
  PYBIND11_TYPE_CASTER(std::unique_ptr<T>, const_name("UniqueT"));

  bool load(handle src, bool) {
    try {
      T* data = src.cast<T*>();
      src.inc_ref();
      value.reset(data);
    } catch (cast_error&) {
      return false;
    }
    return true;
  }

  static handle cast(std::unique_ptr<T> /* src */, return_value_policy /* policy */, handle /* parent */) {
    throw std::runtime_error("invalid unique_ptr to python");
  }
};
template <typename T>
struct type_caster<std::vector<T>> {
 public:
  PYBIND11_TYPE_CASTER(std::vector<T>, const_name("VectorT"));

  bool load(handle src, bool) {
    list l = reinterpret_borrow<list>(src);
    value.clear();
    try {
      for (const handle& el: l)
        value.emplace_back(std::move(el.cast<T&>()));
    } catch (cast_error&) {
      return false;
    }
    return true;
  }

  static handle cast(std::vector<T>&& src, return_value_policy /* policy */, handle /* parent */) {
    list l{0};
    l.inc_ref();
    for (T& el: src)
      l.append(std::move(el));
    src.clear();
    return l;
  }
};
}  // namespace detail
}  // namespace PYBIND11_NAMESPACE

namespace ros2_control_py::bind_impl {

namespace py = pybind11;
using namespace transmission_interface;
using namespace ros2_control_test_assets;
using namespace joint_limits;
using namespace hardware_interface::lifecycle_state_names;
using namespace hardware_interface;
using namespace controller_manager;
using namespace controller_interface;

inline void init(py::module &m)
{
  py::bind_vector<std::vector<ActuatorHandle>>(m, "VectorActuatorHandle");
  py::bind_vector<std::vector<double>>(m, "VectorDouble");
  py::bind_vector<std::vector<InterfaceInfo>>(m, "VectorInterfaceInfo");
  py::bind_vector<std::vector<std::string>>(m, "VectorString");
  py::bind_map<std::unordered_map<std::string, std::string>>(m, "UnorderedMapStringString");
  py::bind_vector<std::vector<std::vector<std::string>>>(m, "VectorString>");
  py::bind_vector<std::vector<ComponentInfo>>(m, "VectorComponentInfo");
  py::bind_map<std::unordered_map<std::string, HardwareComponentInfo>>(m, "UnorderedMapStringHardwareComponentInfo");
  py::bind_vector<std::vector<HardwareInfo>>(m, "VectorHardwareInfo");
  py::bind_vector<std::vector<JointHandle>>(m, "VectorJointHandle");
  py::bind_vector<std::vector<ControllerSpec>>(m, "VectorControllerSpec");
  py::bind_vector<std::vector<JointInfo>>(m, "VectorJointInfo");
  py::bind_vector<std::vector<TransmissionInfo>>(m, "VectorTransmissionInfo");
  py::bind_vector<std::vector<ActuatorInfo>>(m, "VectorActuatorInfo");
}

}
