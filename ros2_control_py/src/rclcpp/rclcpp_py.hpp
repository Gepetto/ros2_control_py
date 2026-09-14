#pragma once

// pybind11
#include <pybind11/pybind11.h>
// rclcpp
#include <rclcpp/executors.hpp>
#include <rclcpp/node_interfaces/node_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "controller_manager/controller_manager_py.hpp"

namespace ros2_control_py::bind_rclcpp {

namespace py = pybind11;
using namespace rclcpp_lifecycle::node_interfaces;
using namespace rclcpp_lifecycle;
using namespace rclcpp::executors;
using namespace rclcpp::node_interfaces;
using namespace rclcpp;

class PyLifecycleNodeInterface : public LifecycleNodeInterface {
 public:
  using LifecycleNodeInterface::LifecycleNodeInterface;

  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn,
                      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
                      on_configure, previous_state);
  }

  CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn,
                      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
                      on_cleanup, previous_state);
  }

  CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn,
                      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
                      on_shutdown, previous_state);
  }

  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn,
                      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
                      on_activate, previous_state);
  }

  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn,
                      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
                      on_deactivate, previous_state);
  }

  CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override {
    PYBIND11_OVERRIDE(CallbackReturn,
                      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
                      on_error, previous_state);
  }
};

class PyNodeBaseInterface : public NodeBaseInterface {
 public:
  using NodeBaseInterface::NodeBaseInterface;

  const char* get_name() const override {
    PYBIND11_OVERRIDE_PURE(
        const char*, rclcpp::node_interfaces::NodeBaseInterface, get_name,

    );
  }

  const char* get_namespace() const override {
    PYBIND11_OVERRIDE_PURE(
        const char*, rclcpp::node_interfaces::NodeBaseInterface, get_namespace,

    );
  }
};

class PyExecutor : public Executor {
 public:
  using Executor::Executor;

  void spin() override {
    PYBIND11_OVERRIDE_PURE(void, rclcpp::Executor, spin,

    );
  }

  void add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
                bool notify) override {
    PYBIND11_OVERRIDE(void, rclcpp::Executor, add_node, node_ptr, notify);
  }

  void add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify) override {
    PYBIND11_OVERRIDE(void, rclcpp::Executor, add_node, node_ptr, notify);
  }
};

class PySingleThreadedExecutor : public SingleThreadedExecutor {
 public:
  using SingleThreadedExecutor::SingleThreadedExecutor;

  void spin() override {
    PYBIND11_OVERRIDE_PURE(void, SingleThreadedExecutor, spin,

    );
  }

  void add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
                bool notify) override {
    PYBIND11_OVERRIDE(void, SingleThreadedExecutor, add_node, node_ptr, notify);
  }

  void add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify) override {
    PYBIND11_OVERRIDE(void, SingleThreadedExecutor, add_node, node_ptr, notify);
  }
};

inline void init_rclcpp([[maybe_unused]] py::module& m) {
  py::class_<Time>(m, "Time")
      .def(py::init<>())
      .def(py::init<uint32_t, uint32_t>())
      .def(py::init<uint64_t>())
      .def("nanoseconds", &Time::nanoseconds)
      .def("__eq__",
           [](const Time& lhs, const Time& rhs) { return lhs == rhs; })
      .def("__ne__",
           [](const Time& lhs, const Time& rhs) { return lhs != rhs; })
      .def("__ge__",
           [](const Time& lhs, const Time& rhs) { return lhs >= rhs; })
      .def("__gt__", [](const Time& lhs, const Time& rhs) { return lhs > rhs; })
      .def("__le__",
           [](const Time& lhs, const Time& rhs) { return lhs <= rhs; })
      .def("__lt__", [](const Time& lhs, const Time& rhs) { return lhs < rhs; })
      .def("__add__",
           [](const Time& lhs, const Duration& rhs) { return lhs + rhs; })
      .def("__sub__",
           [](const Time& lhs, const Time& rhs) { return lhs - rhs; })
      .def("__sub__",
           [](const Time& lhs, const Duration& rhs) { return lhs - rhs; });

  py::class_<Duration>(m, "Duration")
      .def(py::init<uint32_t, uint32_t>())
      .def("nanoseconds", &Duration::nanoseconds)
      .def("__eq__",
           [](const Duration& lhs, const Duration& rhs) { return lhs == rhs; })
      .def("__ne__",
           [](const Duration& lhs, const Duration& rhs) { return lhs != rhs; })
      .def("__ge__",
           [](const Duration& lhs, const Duration& rhs) { return lhs >= rhs; })
      .def("__gt__",
           [](const Duration& lhs, const Duration& rhs) { return lhs > rhs; })
      .def("__le__",
           [](const Duration& lhs, const Duration& rhs) { return lhs <= rhs; })
      .def("__lt__",
           [](const Duration& lhs, const Duration& rhs) { return lhs < rhs; })
      .def("__add__",
           [](const Duration& lhs, const Duration& rhs) { return lhs + rhs; })
      .def("__sub__",
           [](const Duration& lhs, const Duration& rhs) { return lhs - rhs; })
      .def("__mul__",
           [](const Duration& lhs, double rhs) { return lhs * rhs; });

  m.def("init", []() { rclcpp::init(0, nullptr); });

  m.def("init", [](std::vector<std::string>& argv) {
    if (argv.empty()) {
      rclcpp::init(0, nullptr);
      return;
    }
    std::vector<const char*> vec(argv.size() + 1);
    for (size_t i = 0; i < argv.size(); ++i) vec[i] = argv[i].data();
    vec[argv.size()] = nullptr;
    rclcpp::init(argv.size(), vec.data());
  });

  m.def("shutdown", []() { return shutdown(); });

  m.def("shutdown",
        [](rclcpp::Context::SharedPtr context) { return shutdown(context); });

  m.def("shutdown",
        [](rclcpp::Context::SharedPtr context, const std::string& reason) {
          return shutdown(context, reason);
        });

  m.def("ok", []() { return ok(); });

  m.def("ok", [](rclcpp::Context::SharedPtr context) { return ok(context); });
  py::enum_<LifecycleNodeInterface::CallbackReturn>(m, "CallbackReturn")
      .value("SUCCESS", LifecycleNodeInterface::CallbackReturn::SUCCESS)
      .value("FAILURE", LifecycleNodeInterface::CallbackReturn::FAILURE)
      .value("ERROR", LifecycleNodeInterface::CallbackReturn::ERROR)
      .export_values();
  py::class_<LifecycleNodeInterface, PyLifecycleNodeInterface>(
      m, "LifecycleNodeInterface")
      .def(py::init<>())
      .def("on_configure", &LifecycleNodeInterface::on_configure)
      .def("on_cleanup", &LifecycleNodeInterface::on_cleanup)
      .def("on_shutdown", &LifecycleNodeInterface::on_shutdown)
      .def("on_activate", &LifecycleNodeInterface::on_activate)
      .def("on_deactivate", &LifecycleNodeInterface::on_deactivate)
      .def("on_error", &LifecycleNodeInterface::on_error);

  py::class_<State>(m, "State")
      .def(py::init<>())
      .def(py::init<uint8_t, const std::string&>())
      .def("id", &State::id)
      .def("label", &State::label);

  py::class_<NodeBaseInterface, PyNodeBaseInterface>(m, "NodeBaseInterface")
      .def("get_name", &NodeBaseInterface::get_name)
      .def("get_namespace", &NodeBaseInterface::get_namespace);

  py::class_<NodeBase, std::shared_ptr<NodeBase>>(m, "NodeBase")
      .def("get_name", &NodeBase::get_name)
      .def("get_namespace", &NodeBase::get_namespace);

  py::class_<Node, std::shared_ptr<Node>>(m, "Node")
      .def(py::init<const std::string&, const std::string&>())
      .def(py::init<const std::string&>())
      .def("get_name", &Node::get_name)
      .def("get_namespace", &Node::get_namespace)
      .def("now", &Node::now);

  py::class_<Executor, PyExecutor, std::shared_ptr<Executor>>(m, "Executor")
      .def("spin", &Executor::spin)
      .def("add_node",
           [](Executor& self,
              ros2_control_py::bind_controller_manager::ControllerManager::
                  SharedPtr node_ptr,
              bool notify) { return self.add_node(node_ptr, notify); })
      .def("add_node",
           [](Executor& self,
              rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr) {
             return self.add_node(node_ptr);
           })
      .def("add_node",
           [](Executor& self, std::shared_ptr<rclcpp::Node> node_ptr,
              bool notify) { return self.add_node(node_ptr, notify); })
      .def("add_node",
           [](Executor& self, std::shared_ptr<rclcpp::Node> node_ptr) {
             return self.add_node(node_ptr);
           });

  py::class_<SingleThreadedExecutor, Executor, PySingleThreadedExecutor,
             std::shared_ptr<SingleThreadedExecutor>>(m,
                                                      "SingleThreadedExecutor")
      .def(py::init<>())
      .def("spin_some",
           [](SingleThreadedExecutor& self) { return self.spin_some(); })
      .def("spin_some",
           [](SingleThreadedExecutor& self, std::int64_t max_duration) {
             self.spin_some(std::chrono::nanoseconds(max_duration));
           })
      .def("spin_some",
           [](SingleThreadedExecutor& self, double max_duration) {
             self.spin_some(std::chrono::nanoseconds(
                 static_cast<std::int64_t>(max_duration)));
           })
      .def("spin_all",
           [](SingleThreadedExecutor& self, std::int64_t max_duration) {
             self.spin_all(std::chrono::nanoseconds(max_duration));
           })
      .def("spin_all",
           [](SingleThreadedExecutor& self, double max_duration) {
             self.spin_all(std::chrono::nanoseconds(
                 static_cast<std::int64_t>(max_duration)));
           })
      .def("spin", &SingleThreadedExecutor::spin)
      .def("add_node",
           [](SingleThreadedExecutor& self,
              ros2_control_py::bind_controller_manager::ControllerManager::
                  SharedPtr node_ptr) { return self.add_node(node_ptr); })
      .def("add_node",
           [](SingleThreadedExecutor& self,
              rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
              bool notify) { return self.add_node(node_ptr, notify); })
      .def("add_node", [](SingleThreadedExecutor& self,
                          std::shared_ptr<rclcpp::Node> node_ptr, bool notify) {
        return self.add_node(node_ptr, notify);
      });
}

}  // namespace ros2_control_py::bind_rclcpp
