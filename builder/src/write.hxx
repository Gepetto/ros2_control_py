#pragma once

// hpp
#include "write.hpp"

/// @brief output a `Cls` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Cls& cls);
/// @brief output a `Memb` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Memb& memb);
/// @brief output a `Enum` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Enum& enu);
/// @brief output a `Var` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Var& var);
/// @brief output a `Func` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Func& func);
/// @brief write source files for module `mod` and header `header`
inline void write_module_header(const Module& mod, const Header& header);

void write_module(const Module& mod) {
  for (const Header& header : ptr_iter(mod.headers))
    write_module_header(mod, header);

  std::ofstream ofs{mod.src, std::ios::out | std::ios::trunc};
  ASSERT(ofs, "could not open " << mod.src);
  ofs << R"(// pybind11
#include <pybind11/pybind11.h>

// )" << mod.name
      << '\n';
  for (const Header& header : ptr_iter(mod.headers))
    ofs << "#include <" << mod.name << "/" << header.name << "_py.hpp>\n";
  ofs << R"(
namespace py = pybind11;

PYBIND11_MODULE()"
      << mod.name << R"(, m)
{
  m.doc() = R"doc(
            Python bindings for ros2_control functionalities.
            )doc";

  // Provide custom function signatures
  py::options options;
  options.disable_function_signatures();

  // Construct module classes
)";
  for (const Header& header : ptr_iter(mod.headers)) {
    ofs << "  ros2_control_py::bind_" << mod.name << "::init_"
        << header.proper_name << "(m);\n";
  }
  ofs << "}\n";
}

inline std::ostream& operator<<(std::ostream& os, const Cls& cls) {
  os << "  py::class_<" << (cls.is_outsider ? cls.complete_name : cls.name);
  if (cls.mother)
    os << ", "
       << (cls.mother->is_outsider ? cls.mother->complete_name
                                   : cls.mother->name);
  if (cls.has_virtual) os << ", " << cls.tramp_name;
  os << ">(m, \"" << cls.name << "\")";
  std::vector<Ctor> ctors = cls.ctors;
  if (ctors.empty()) ctors.emplace_back(std::vector<std::string>{});
  for (const Ctor& ctor : ctors)
    os << "\n      .def(py::init<" << Sep{ctor.args, ", "} << ">())";
  for (const Memb& memb : ptr_iter(cls.membs)) {
    if (memb.is_overloaded)
      os << "\n      .def(\"" << memb.name << "\", []("
         << (memb.is_public ? (cls.is_outsider ? cls.complete_name : cls.name)
                            : cls.pub_name)
         << "& py_self" << (memb.args.empty() ? "" : ", ")
         << Sep{memb.args, ", "} << ") { return py_self." << memb.name << '('
         << Sep{memb.args_names, ", "} << "); })";
    else
      os << "\n      .def(\"" << memb.name << "\", &"
         << (memb.is_public ? (cls.is_outsider ? cls.complete_name : cls.name)
                            : cls.pub_name)
         << "::" << memb.name << ")";
  }
  for (const Attr& attr : ptr_iter(cls.attrs))
    os << "\n      .def_readwrite(\"" << attr.name << "\", &"
       << (attr.is_public ? (cls.is_outsider ? cls.complete_name : cls.name)
                          : cls.pub_name)
       << "::" << attr.name << ')';
  os << ";\n";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const Memb& memb) {
  ASSERT(memb.is_virtual && !memb.is_final,
         memb.cls << "::" << memb.name
                  << " is not a virtual member but it was used as one");
  return os << "\n  " << memb.ret_type << ' ' << memb.name << '('
            << Sep{memb.args, ", "} << ") " << (memb.is_const ? "const " : "")
            << R"(override {
    PYBIND11_OVERRIDE)"
            << (memb.is_pure ? "_PURE" : "") << R"((
        )" << memb.ret_type
            << R"(,
        )" << memb.cls
            << R"(,
        )" << memb.name
            << ",\n        " << Sep{memb.args_names, ",\n        "} << R"(
    );
  }
)";
}

inline std::ostream& operator<<(std::ostream& os, const Enum& enu) {
  ASSERT(!enu.items.empty(), "empty enum");
  os << "  py::enum_<" << enu.name << ">(m, \"" << enu.name << "\")\n";
  for (const std::string& item : enu.items)
    os << "      .value(\"" << item << "\", " << enu.name << "::" << item
       << ")\n";
  return os << "      .export_values();\n";
}

inline std::ostream& operator<<(std::ostream& os, const Var& var) {
  return os << "  m.attr(\"" << var.name << "\") = " << var.name << ";\n";
}

inline std::ostream& operator<<(std::ostream& os, const Func& func) {
  if (func.is_overloaded)
    return os << "\n      .def(\"" << func.name << "\", []("
              << Sep{func.args, ", "} << ") { return " << func.name << '('
              << Sep{func.args_names, ", "} << "); })";
  return os << "  m.def(\"" << func.name << "\", &" << func.name << ");\n";
}

void write_module_header(const Module& mod, const Header& header) {
  fs::path path = mod.src_dir / (header.name + "_py.hpp");
  fs::create_directories(path.parent_path());
  ASSERT_DIR(path.parent_path());
  std::ofstream ofs{path, std::ios::out | std::ios::trunc};
  ASSERT(ofs, "Could not open " << path);
  ofs << R"(#pragma once

// pybind11
#include <pybind11/pybind11.h>
)";
  if (header.name != mod.py_utils->name)
    ofs << "\n// " << mod.name << "\n#include <" << mod.name << '/'
        << header.name << ".hpp>\n// py_utils\n#include <" << mod.name << '/'
        << mod.py_utils->name << "_py.hpp>\n";
  else {
    ofs << "#include <pybind11/stl_bind.h>\n// STL\n#include "
           "<stdexcept>\n#include "
           "<vector>\n#include <unordered_map>\n#include <map>\n// "
        << mod.name << '\n';
    for (const Header& h : ptr_iter(mod.headers))
      if (&h != &header)
        ofs << "#include <" << mod.name << '/' << h.name << ".hpp>\n";
    ofs << "\nnamespace PYBIND11_NAMESPACE {\nnamespace detail {\n";
    for (const std::string& ns : header.namespaces)
      ofs << "using namespace " << ns << ";\n";
    ofs << "}\n}\n";
    if (!header.stl_bind.empty()) ofs << '\n';
    for (const auto& [type, cpp_type, u, v] : header.stl_bind)
      ofs << "PYBIND11_MAKE_OPAQUE(std::" << cpp_type << '<' << u
          << (v.empty() ? "" : ", " + v) << ">);\n";
    ofs << R"(
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
)";
  }
  ofs << R"(
namespace ros2_control_py::bind_)"
      << mod.name << R"(
{

namespace py = pybind11;
)";
  for (const std::string& ns : header.namespaces)
    ofs << "using namespace " << ns << ";\n";
  for (const Cls& cls : ptr_iter(header.classes)) {
    if (!cls.has_virtual) continue;
    ofs << "\nclass " << cls.tramp_name << ": public "
        << (cls.is_outsider ? cls.complete_name : cls.name)
        << " {\n public:\n  using " << cls.name << "::" << cls.name << ";\n";
    for (const Memb& vmemb : ptr_iter(cls.find_vmembs())) ofs << vmemb;
    ofs << "};\n";
  }
  for (const Cls& cls : ptr_iter(header.classes)) {
    if (!cls.has_protected) continue;
    ofs << "\nclass " << cls.pub_name << ": public "
        << (cls.is_outsider ? cls.complete_name : cls.name) << " {\n public:\n";
    for (const Memb& pmemb : ptr_iter(cls.find_pmembs()))
      ofs << "  using " << cls.name << "::" << pmemb.name << ";\n";
    for (const Attr& pattr : ptr_iter(cls.find_pattrs()))
      ofs << "  using " << cls.name << "::" << pattr.name << ";\n";
    ofs << "};\n";
  }
  if (header.name == mod.py_utils->name) {
    ofs << R"(
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
  Ref& set_value(double value) { *value_ = value; return *this; }

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

  Ref<T> get(py::object instance, py::object) {
    std::size_t id = reinterpret_cast<std::size_t>(instance.ptr());
    auto it = values_.find(id);
    if (it == values_.end())
      it = values_.insert({id, Ref<T>(default_value_)}).first;
    return it->second;
  }

  void set(py::object instance, const T& value) {
    std::size_t id = reinterpret_cast<std::size_t>(instance.ptr());
    auto it = values_.find(id);
    if (it != values_.end())
      *it->second = value;
    else
      values_.insert({id, Ref<T>(value)});
  }

  void del(py::object) {}

 private:
  T default_value_;
  std::unordered_map<std::size_t, Ref<T>> values_;
};
)";
  }
  ofs << R"(
inline void init_)"
      << header.proper_name << R"((py::module &m)
{
)";
  if (header.name == mod.py_utils->name) {
    ofs << R"(  py::class_<Ref<double>>(m, "FloatRef")
      .def(py::init<double>())
      .def("__repr__", &Ref<double>::repr)
      .def("__float__", &Ref<double>::get_value)
      .def("get_value", &Ref<double>::get_value)
      .def("set_value", &Ref<double>::set_value)
      .def("set_value", [](Ref<double>& py_self, long long value) { return py_self.set_value(value); })
      .def("set_value", [](Ref<double>& py_self, const Ref<double>& value) { return py_self.set_value(value); })
      .def("__matmul__", &Ref<double>::set_value)
      .def("__matmul__", [](Ref<double>& lhs, const Ref<double>& rhs) { return lhs.set_value(rhs); })
      .def("__matmul__", [](Ref<double>& lhs, long long rhs) { return lhs.set_value(rhs); })
      .def("__imatmul__", &Ref<double>::set_value)
      .def("__imatmul__", [](Ref<double>& lhs, const Ref<double>& rhs) { return lhs.set_value(rhs); })
      .def("__imatmul__", [](Ref<double>& lhs, long long rhs) { return lhs.set_value(rhs); })
      .def("__add__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs + *rhs; })
      .def("__add__", [](const Ref<double>& lhs, double rhs) { return *lhs + rhs; })
      .def("__add__", [](const Ref<double>& lhs, long long rhs) { return *lhs + rhs; })
      .def("__radd__", [](const Ref<double>& rhs, double lhs) { return lhs + *rhs; })
      .def("__radd__", [](const Ref<double>& rhs, long long lhs) { return lhs + *rhs; })
      .def("__iadd__", [](Ref<double>& lhs, const Ref<double>& rhs) { *lhs += *rhs; return lhs; })
      .def("__iadd__", [](Ref<double>& lhs, double rhs) { *lhs += rhs; return lhs; })
      .def("__iadd__", [](Ref<double>& lhs, long long rhs) { *lhs += rhs; return lhs; })
      .def("__sub__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs - *rhs; })
      .def("__sub__", [](const Ref<double>& lhs, double rhs) { return *lhs - rhs; })
      .def("__sub__", [](const Ref<double>& lhs, long long rhs) { return *lhs - rhs; })
      .def("__rsub__", [](const Ref<double>& rhs, double lhs) { return lhs - *rhs; })
      .def("__rsub__", [](const Ref<double>& rhs, long long lhs) { return lhs - *rhs; })
      .def("__isub__", [](Ref<double>& lhs, const Ref<double>& rhs) { *lhs -= *rhs; return lhs; })
      .def("__isub__", [](Ref<double>& lhs, double rhs) { *lhs -= rhs; return lhs; })
      .def("__isub__", [](Ref<double>& lhs, long long rhs) { *lhs -= rhs; return lhs; })
      .def("__mul__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs * *rhs; })
      .def("__mul__", [](const Ref<double>& lhs, double rhs) { return *lhs * rhs; })
      .def("__mul__", [](const Ref<double>& lhs, long long rhs) { return *lhs * rhs; })
      .def("__rmul__", [](const Ref<double>& rhs, double lhs) { return lhs * *rhs; })
      .def("__rmul__", [](const Ref<double>& rhs, long long lhs) { return lhs * *rhs; })
      .def("__imul__", [](Ref<double>& lhs, const Ref<double>& rhs) { *lhs *= *rhs; return lhs; })
      .def("__imul__", [](Ref<double>& lhs, double rhs) { *lhs *= rhs; return lhs; })
      .def("__imul__", [](Ref<double>& lhs, long long rhs) { *lhs *= rhs; return lhs; })
      .def("__truediv__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs / *rhs; })
      .def("__truediv__", [](const Ref<double>& lhs, double rhs) { return *lhs / rhs; })
      .def("__truediv__", [](const Ref<double>& lhs, long long rhs) { return *lhs / rhs; })
      .def("__rtruediv__", [](const Ref<double>& rhs, double lhs) { return lhs / *rhs; })
      .def("__rtruediv__", [](const Ref<double>& rhs, long long lhs) { return lhs / *rhs; })
      .def("__itruediv__", [](Ref<double>& lhs, const Ref<double>& rhs) { *lhs /= *rhs; return lhs; })
      .def("__itruediv__", [](Ref<double>& lhs, double rhs) { *lhs /= rhs; return lhs; })
      .def("__itruediv__", [](Ref<double>& lhs, long long rhs) { *lhs /= rhs; return lhs; })
      .def("__eq__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs == *rhs; })
      .def("__eq__", [](const Ref<double>& lhs, double rhs) { return *lhs == rhs; })
      .def("__eq__", [](const Ref<double>& lhs, long long rhs) { return *lhs == rhs; })
      .def("__req__", [](const Ref<double>& rhs, double lhs) { return lhs == *rhs; })
      .def("__req__", [](const Ref<double>& rhs, long long lhs) { return lhs == *rhs; })
      .def("__ne__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs != *rhs; })
      .def("__ne__", [](const Ref<double>& lhs, double rhs) { return *lhs != rhs; })
      .def("__ne__", [](const Ref<double>& lhs, long long rhs) { return *lhs != rhs; })
      .def("__rne__", [](const Ref<double>& rhs, double lhs) { return lhs != *rhs; })
      .def("__rne__", [](const Ref<double>& rhs, long long lhs) { return lhs != *rhs; })
      .def("__ge__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs >= *rhs; })
      .def("__ge__", [](const Ref<double>& lhs, double rhs) { return *lhs >= rhs; })
      .def("__ge__", [](const Ref<double>& lhs, long long rhs) { return *lhs >= rhs; })
      .def("__rge__", [](const Ref<double>& rhs, double lhs) { return lhs >= *rhs; })
      .def("__rge__", [](const Ref<double>& rhs, long long lhs) { return lhs >= *rhs; })
      .def("__gt__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs > *rhs; })
      .def("__gt__", [](const Ref<double>& lhs, double rhs) { return *lhs > rhs; })
      .def("__gt__", [](const Ref<double>& lhs, long long rhs) { return *lhs > rhs; })
      .def("__rgt__", [](const Ref<double>& rhs, double lhs) { return lhs > *rhs; })
      .def("__rgt__", [](const Ref<double>& rhs, long long lhs) { return lhs > *rhs; })
      .def("__le__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs <= *rhs; })
      .def("__le__", [](const Ref<double>& lhs, double rhs) { return *lhs <= rhs; })
      .def("__le__", [](const Ref<double>& lhs, long long rhs) { return *lhs <= rhs; })
      .def("__rle__", [](const Ref<double>& rhs, double lhs) { return lhs <= *rhs; })
      .def("__rle__", [](const Ref<double>& rhs, long long lhs) { return lhs <= *rhs; })
      .def("__lt__", [](const Ref<double>& lhs, const Ref<double>& rhs) { return *lhs < *rhs; })
      .def("__lt__", [](const Ref<double>& lhs, double rhs) { return *lhs < rhs; })
      .def("__lt__", [](const Ref<double>& lhs, long long rhs) { return *lhs < rhs; })
      .def("__rlt__", [](const Ref<double>& rhs, double lhs) { return lhs < *rhs; })
      .def("__rlt__", [](const Ref<double>& rhs, long long lhs) { return lhs < *rhs; });

  py::class_<RefProp<double>>(m, "FloatRefProp")
      .def(py::init<double>())
      .def("__get__", &RefProp<double>::get)
      .def("__set__", &RefProp<double>::set)
      .def("__set__", [](RefProp<double>& py_self, py::object instance, long long value) { py_self.set(instance, value); })
      .def("__set__", [](RefProp<double>& py_self, py::object instance, Ref<double> value) { py_self.set(instance, value); })
      .def("__delete__", &RefProp<double>::del);

)";
    if (mod.name == "hardware_interface") {
      ofs << R"(  py::enum_<CallbackReturn>(m, "CallbackReturn")
      .value("SUCCESS", CallbackReturn::SUCCESS)
      .value("FAILURE", CallbackReturn::FAILURE)
      .value("ERROR", CallbackReturn::ERROR)
      .export_values();

  py::class_<rclcpp_lifecycle::State>(m, "State")
      .def(py::init<>())
      .def(py::init<uint8_t, const std::string&>())
      .def("id", &rclcpp_lifecycle::State::id)
      .def("label", &rclcpp_lifecycle::State::label);

  py::class_<rclcpp::Time>(m, "Time")
      .def(py::init<>())
      .def(py::init<uint32_t, uint32_t>())
      .def(py::init<uint64_t>())
      .def("nanoseconds", &rclcpp::Time::nanoseconds)
      .def("__eq__", [](const rclcpp::Time& lhs, const rclcpp::Time& rhs) { return lhs == rhs; })
      .def("__ne__", [](const rclcpp::Time& lhs, const rclcpp::Time& rhs) { return lhs != rhs; })
      .def("__ge__", [](const rclcpp::Time& lhs, const rclcpp::Time& rhs) { return lhs >= rhs; })
      .def("__gt__", [](const rclcpp::Time& lhs, const rclcpp::Time& rhs) { return lhs > rhs; })
      .def("__le__", [](const rclcpp::Time& lhs, const rclcpp::Time& rhs) { return lhs <= rhs; })
      .def("__lt__", [](const rclcpp::Time& lhs, const rclcpp::Time& rhs) { return lhs < rhs; })
      .def("__add__", [](const rclcpp::Time& lhs, const rclcpp::Duration& rhs) { return lhs + rhs; })
      .def("__sub__", [](const rclcpp::Time& lhs, const rclcpp::Time& rhs) { return lhs - rhs; })
      .def("__sub__", [](const rclcpp::Time& lhs, const rclcpp::Duration& rhs) { return lhs - rhs; });

  py::class_<rclcpp::Duration>(m, "Duration")
      .def(py::init<uint32_t, uint32_t>())
      .def("nanoseconds", &rclcpp::Duration::nanoseconds)
      .def("__eq__", [](const rclcpp::Duration& lhs, const rclcpp::Duration& rhs) { return lhs == rhs; })
      .def("__ne__", [](const rclcpp::Duration& lhs, const rclcpp::Duration& rhs) { return lhs != rhs; })
      .def("__ge__", [](const rclcpp::Duration& lhs, const rclcpp::Duration& rhs) { return lhs >= rhs; })
      .def("__gt__", [](const rclcpp::Duration& lhs, const rclcpp::Duration& rhs) { return lhs > rhs; })
      .def("__le__", [](const rclcpp::Duration& lhs, const rclcpp::Duration& rhs) { return lhs <= rhs; })
      .def("__lt__", [](const rclcpp::Duration& lhs, const rclcpp::Duration& rhs) { return lhs < rhs; })
      .def("__add__", [](const rclcpp::Duration& lhs, const rclcpp::Duration& rhs) { return lhs + rhs; })
      .def("__sub__", [](const rclcpp::Duration& lhs, const rclcpp::Duration& rhs) { return lhs - rhs; })
      .def("__mul__", [](const rclcpp::Duration& lhs, double rhs) { return lhs * rhs; });

)";
    }
  }
  for (const auto& [type, cpp_type, u, v] : header.stl_bind) {
    std::string complete_type =
        "std::" + cpp_type + "<" + u + (v.empty() ? "" : ", " + v) + ">";
    ofs << "  py::bind_" << type << '<' << complete_type << ">(m, \""
        << make_pascal_name(cpp_type, u, v) << "\");\n";
  }
  if (!header.stl_bind.empty()) ofs << '\n';
  ofs << Sep{header.vars, "\n"} << Sep{ptr_iter(header.funcs), "\n"}
      << Sep{header.enums, "\n"} << Sep{ptr_iter(header.classes), "\n"} << R"(}

}
)";
}
