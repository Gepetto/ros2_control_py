#pragma once

// pybind11
#include <pybind11/pybind11.h>

// transmission_interface
#include <transmission_interface/transmission_loader.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_transmission_interface
{

namespace py = pybind11;
using namespace transmission_interface;

class PyTransmissionLoader: public TransmissionLoader {
 public:
  using TransmissionLoader::TransmissionLoader;

  std::shared_ptr<Transmission> load(const hardware_interface::TransmissionInfo& transmission_info) override {
    PYBIND11_OVERRIDE_PURE(
        std::shared_ptr<Transmission>,
        TransmissionLoader,
        load,
        transmission_info
    );
  }
};

inline void init_transmission_loader([[maybe_unused]] py::module &m)
{
  py::class_<TransmissionLoader, PyTransmissionLoader>(m, "TransmissionLoader")
      .def(py::init<>())
      .def("load", &TransmissionLoader::load);
}

}
