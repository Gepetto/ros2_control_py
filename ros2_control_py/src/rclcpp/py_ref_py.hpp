#pragma once

// pybind11
#include <pybind11/pybind11.h>
// rclcpp
#include <rclcpp/executors.hpp>
#include <rclcpp/node_interfaces/node_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

template <typename T>
class Ref {
 public:
  Ref() : value_{std::make_shared<T>()} {}
  Ref(const T& value) : value_{std::make_shared<T>(value)} {}

  operator T*() const { return value_.get(); }
  operator T&() const { return *value_; }

  T& operator*() const { return *value_; }
  T* operator->() const { return value_.get(); }

  T get_value() const { return *value_; }
  Ref& set_value(double value) {
    *value_ = value;
    return *this;
  }

  std::string repr() const {
    std::ostringstream oss;
    oss << *value_;
    return std::move(oss).str();
  }

 private:
  std::shared_ptr<T> value_;
};

template <typename T>
class RefProp {
 public:
  RefProp(const T& default_value) : default_value_{default_value} {}

  Ref<T> get(pybind11::object instance, pybind11::object) {
    std::size_t id = reinterpret_cast<std::size_t>(instance.ptr());
    auto it = values_.find(id);
    if (it == values_.end())
      it = values_.insert({id, Ref<T>(default_value_)}).first;
    return it->second;
  }

  void set(pybind11::object instance, const T& value) {
    std::size_t id = reinterpret_cast<std::size_t>(instance.ptr());
    auto it = values_.find(id);
    if (it != values_.end())
      *it->second = value;
    else
      values_.insert({id, Ref<T>(value)});
  }

  void del(pybind11::object) {}

 private:
  T default_value_;
  std::unordered_map<std::size_t, Ref<T>> values_;
};

namespace ros2_control_py::bind_rclcpp {

namespace py = pybind11;

inline void init_py_ref([[maybe_unused]] py::module& m) {
  py::class_<Ref<double>>(m, "FloatRef")
      .def(py::init<double>())
      .def("__repr__", &Ref<double>::repr)
      .def("__float__", &Ref<double>::get_value)
      .def("get_value", &Ref<double>::get_value)
      .def("set_value", &Ref<double>::set_value)
      .def("set_value",
           [](Ref<double>& py_self, long long value) {
             return py_self.set_value(value);
           })
      .def("set_value",
           [](Ref<double>& py_self, const Ref<double>& value) {
             return py_self.set_value(value);
           })
      .def("__matmul__", &Ref<double>::set_value)
      .def("__matmul__",
           [](Ref<double>& lhs, const Ref<double>& rhs) {
             return lhs.set_value(rhs);
           })
      .def("__matmul__",
           [](Ref<double>& lhs, long long rhs) { return lhs.set_value(rhs); })
      .def("__imatmul__", &Ref<double>::set_value)
      .def("__imatmul__",
           [](Ref<double>& lhs, const Ref<double>& rhs) {
             return lhs.set_value(rhs);
           })
      .def("__imatmul__",
           [](Ref<double>& lhs, long long rhs) { return lhs.set_value(rhs); })
      .def("__add__", [](const Ref<double>& lhs,
                         const Ref<double>& rhs) { return *lhs + *rhs; })
      .def("__add__",
           [](const Ref<double>& lhs, double rhs) { return *lhs + rhs; })
      .def("__add__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs + rhs; })
      .def("__radd__",
           [](const Ref<double>& rhs, double lhs) { return lhs + *rhs; })
      .def("__radd__",
           [](const Ref<double>& rhs, long long lhs) { return lhs + *rhs; })
      .def("__iadd__",
           [](Ref<double>& lhs, const Ref<double>& rhs) {
             *lhs += *rhs;
             return lhs;
           })
      .def("__iadd__",
           [](Ref<double>& lhs, double rhs) {
             *lhs += rhs;
             return lhs;
           })
      .def("__iadd__",
           [](Ref<double>& lhs, long long rhs) {
             *lhs += rhs;
             return lhs;
           })
      .def("__sub__", [](const Ref<double>& lhs,
                         const Ref<double>& rhs) { return *lhs - *rhs; })
      .def("__sub__",
           [](const Ref<double>& lhs, double rhs) { return *lhs - rhs; })
      .def("__sub__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs - rhs; })
      .def("__rsub__",
           [](const Ref<double>& rhs, double lhs) { return lhs - *rhs; })
      .def("__rsub__",
           [](const Ref<double>& rhs, long long lhs) { return lhs - *rhs; })
      .def("__isub__",
           [](Ref<double>& lhs, const Ref<double>& rhs) {
             *lhs -= *rhs;
             return lhs;
           })
      .def("__isub__",
           [](Ref<double>& lhs, double rhs) {
             *lhs -= rhs;
             return lhs;
           })
      .def("__isub__",
           [](Ref<double>& lhs, long long rhs) {
             *lhs -= rhs;
             return lhs;
           })
      .def("__mul__", [](const Ref<double>& lhs,
                         const Ref<double>& rhs) { return *lhs * *rhs; })
      .def("__mul__",
           [](const Ref<double>& lhs, double rhs) { return *lhs * rhs; })
      .def("__mul__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs * rhs; })
      .def("__rmul__",
           [](const Ref<double>& rhs, double lhs) { return lhs * *rhs; })
      .def("__rmul__",
           [](const Ref<double>& rhs, long long lhs) { return lhs * *rhs; })
      .def("__imul__",
           [](Ref<double>& lhs, const Ref<double>& rhs) {
             *lhs *= *rhs;
             return lhs;
           })
      .def("__imul__",
           [](Ref<double>& lhs, double rhs) {
             *lhs *= rhs;
             return lhs;
           })
      .def("__imul__",
           [](Ref<double>& lhs, long long rhs) {
             *lhs *= rhs;
             return lhs;
           })
      .def("__truediv__", [](const Ref<double>& lhs,
                             const Ref<double>& rhs) { return *lhs / *rhs; })
      .def("__truediv__",
           [](const Ref<double>& lhs, double rhs) { return *lhs / rhs; })
      .def("__truediv__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs / rhs; })
      .def("__rtruediv__",
           [](const Ref<double>& rhs, double lhs) { return lhs / *rhs; })
      .def("__rtruediv__",
           [](const Ref<double>& rhs, long long lhs) { return lhs / *rhs; })
      .def("__itruediv__",
           [](Ref<double>& lhs, const Ref<double>& rhs) {
             *lhs /= *rhs;
             return lhs;
           })
      .def("__itruediv__",
           [](Ref<double>& lhs, double rhs) {
             *lhs /= rhs;
             return lhs;
           })
      .def("__itruediv__",
           [](Ref<double>& lhs, long long rhs) {
             *lhs /= rhs;
             return lhs;
           })
      .def("__eq__", [](const Ref<double>& lhs,
                        const Ref<double>& rhs) { return *lhs == *rhs; })
      .def("__eq__",
           [](const Ref<double>& lhs, double rhs) { return *lhs == rhs; })
      .def("__eq__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs == rhs; })
      .def("__req__",
           [](const Ref<double>& rhs, double lhs) { return lhs == *rhs; })
      .def("__req__",
           [](const Ref<double>& rhs, long long lhs) { return lhs == *rhs; })
      .def("__ne__", [](const Ref<double>& lhs,
                        const Ref<double>& rhs) { return *lhs != *rhs; })
      .def("__ne__",
           [](const Ref<double>& lhs, double rhs) { return *lhs != rhs; })
      .def("__ne__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs != rhs; })
      .def("__rne__",
           [](const Ref<double>& rhs, double lhs) { return lhs != *rhs; })
      .def("__rne__",
           [](const Ref<double>& rhs, long long lhs) { return lhs != *rhs; })
      .def("__ge__", [](const Ref<double>& lhs,
                        const Ref<double>& rhs) { return *lhs >= *rhs; })
      .def("__ge__",
           [](const Ref<double>& lhs, double rhs) { return *lhs >= rhs; })
      .def("__ge__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs >= rhs; })
      .def("__rge__",
           [](const Ref<double>& rhs, double lhs) { return lhs >= *rhs; })
      .def("__rge__",
           [](const Ref<double>& rhs, long long lhs) { return lhs >= *rhs; })
      .def("__gt__", [](const Ref<double>& lhs,
                        const Ref<double>& rhs) { return *lhs > *rhs; })
      .def("__gt__",
           [](const Ref<double>& lhs, double rhs) { return *lhs > rhs; })
      .def("__gt__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs > rhs; })
      .def("__rgt__",
           [](const Ref<double>& rhs, double lhs) { return lhs > *rhs; })
      .def("__rgt__",
           [](const Ref<double>& rhs, long long lhs) { return lhs > *rhs; })
      .def("__le__", [](const Ref<double>& lhs,
                        const Ref<double>& rhs) { return *lhs <= *rhs; })
      .def("__le__",
           [](const Ref<double>& lhs, double rhs) { return *lhs <= rhs; })
      .def("__le__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs <= rhs; })
      .def("__rle__",
           [](const Ref<double>& rhs, double lhs) { return lhs <= *rhs; })
      .def("__rle__",
           [](const Ref<double>& rhs, long long lhs) { return lhs <= *rhs; })
      .def("__lt__", [](const Ref<double>& lhs,
                        const Ref<double>& rhs) { return *lhs < *rhs; })
      .def("__lt__",
           [](const Ref<double>& lhs, double rhs) { return *lhs < rhs; })
      .def("__lt__",
           [](const Ref<double>& lhs, long long rhs) { return *lhs < rhs; })
      .def("__rlt__",
           [](const Ref<double>& rhs, double lhs) { return lhs < *rhs; })
      .def("__rlt__",
           [](const Ref<double>& rhs, long long lhs) { return lhs < *rhs; });

  py::class_<RefProp<double>>(m, "FloatRefProp")
      .def(py::init<double>())
      .def("__get__", &RefProp<double>::get)
      .def("__set__", &RefProp<double>::set)
      .def("__set__", [](RefProp<double>& py_self, py::object instance,
                         long long value) { py_self.set(instance, value); })
      .def("__set__", [](RefProp<double>& py_self, py::object instance,
                         Ref<double> value) { py_self.set(instance, value); })
      .def("__delete__", &RefProp<double>::del);
}

}  // namespace ros2_control_py::bind_rclcpp
