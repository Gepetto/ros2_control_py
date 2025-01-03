#pragma once

// pybind11
#include <pybind11/pybind11.h>

// transmission_interface
#include <transmission_interface/transmission.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_transmission_interface
{

namespace py = pybind11;
using namespace transmission_interface;

class PyTransmission: public Transmission {
 public:
  using Transmission::Transmission;

  void configure(const std::vector<JointHandle>& joint_handles, const std::vector<ActuatorHandle>& actuator_handles) override {
    PYBIND11_OVERRIDE_PURE(
        void,
        Transmission,
        configure,
        joint_handles,
        actuator_handles
    );
  }

  void actuator_to_joint() override {
    PYBIND11_OVERRIDE_PURE(
        void,
        Transmission,
        actuator_to_joint,
        
    );
  }

  void joint_to_actuator() override {
    PYBIND11_OVERRIDE_PURE(
        void,
        Transmission,
        joint_to_actuator,
        
    );
  }

  std::size_t num_actuators() const override {
    PYBIND11_OVERRIDE_PURE(
        std::size_t,
        Transmission,
        num_actuators,
        
    );
  }

  std::size_t num_joints() const override {
    PYBIND11_OVERRIDE_PURE(
        std::size_t,
        Transmission,
        num_joints,
        
    );
  }
};

inline void init_transmission([[maybe_unused]] py::module &m)
{
  py::class_<Transmission, PyTransmission>(m, "Transmission")
      .def(py::init<>())
      .def("configure", &Transmission::configure)
      .def("actuator_to_joint", &Transmission::actuator_to_joint)
      .def("joint_to_actuator", &Transmission::joint_to_actuator)
      .def("num_actuators", &Transmission::num_actuators)
      .def("num_joints", &Transmission::num_joints);
}

}
