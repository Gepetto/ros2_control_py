#pragma once

// STL
#include <filesystem>
#include <ostream>
// ros2_control_pŷ_builder
#include "structs.hpp"
#include "utils.hpp"

namespace fs = std::filesystem;

/// @brief output a `Cls` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Cls& cls);
/// @brief output a `VMemb` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const VMemb& vmemb);
/// @brief output a `Enum` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Enum& enu);
/// @brief output a `Var` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Var& var);
/// @brief write a hpp file in `inc_hi_dir` for header `header`
inline void write_named_hi_py_hpp(const fs::path& inc_hi_dir,
                                  const Header& header);
/// @brief write a cpp file in `hi_py` calling bindings from headers `headers`
inline void write_hi_py_cpp(const fs::path& hi_py,
                            const std::vector<Header>& headers);

// Impl

inline std::ostream& operator<<(std::ostream& os, const Cls& cls) {
  if (!cls.vmembs.empty()) {
    Cls py_cls{"Py" + cls.name, cls.name};
    py_cls.attrs = cls.attrs;
    py_cls.ctors = cls.ctors;
    py_cls.membs = cls.membs;
    py_cls.membs.reserve(py_cls.membs.size() + cls.vmembs.size());
    for (const VMemb& vmemb : cls.vmembs) py_cls.membs.emplace_back(vmemb.name);
    return os << py_cls;
  }
  os << "  py::class_<" << cls.name
     << (cls.mother.empty() ? "" : ", " + cls.mother)
     << ">(hardware_interface_py, \"" << cls.name << "\")";
  std::vector<Ctor> ctors = cls.ctors;
  if (ctors.empty()) ctors.emplace_back(std::vector<std::string>{});
  for (const Ctor& ctor : ctors)
    os << "\n      .def(py::init<" << Sep(ctor.args, ", ") << ">())";
  for (const Memb& memb : cls.membs)
    os << "\n      .def(\"" << memb.name << "\", &" << cls.name
       << "::" << memb.name << ")";
  for (const Attr& attr : cls.attrs)
    os << "\n      .def_readwrite(\"" << attr.name << "\", &" << cls.name
       << "::" << attr.name << ")";
  os << ";\n";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const VMemb& vmemb) {
  os << "\n  " << vmemb.ret_type << ' ' << vmemb.name << '('
     << Sep(vmemb.args, ", ") << R"() override {
    PYBIND11_OVERRIDE_PURE(
        )"
     << vmemb.ret_type << R"(,
        )"
     << vmemb.cls << R"(,
        )"
     << vmemb.name;
  for (const std::string& arg : vmemb.args_names) os << ",\n        " << arg;
  return os << R"(
    );
  }
)";
}

inline std::ostream& operator<<(std::ostream& os, const Enum& enu) {
  ASSERT(!enu.items.empty(), "empty enum");
  os << "  py::enum_<" << enu.name << ">(hardware_interface_py, \"" << enu.name
     << "\")\n";
  for (const std::string& item : enu.items)
    os << "      .value(\"" << item << "\", " << enu.name << "::" << item
       << ")\n";
  return os << "      .export_values();\n";
}

inline std::ostream& operator<<(std::ostream& os, const Var& var) {
  return os << "  hardware_interface_py.def(\"" << var.name
            << "\", []() { return std::string{" << var.name << "}; });";
}

void write_named_hi_py_hpp(const fs::path& inc_hi_dir, const Header& header) {
  fs::path path = inc_hi_dir / (header.name + "_py.hpp");
  std::ofstream ofs{path, std::ios::out | std::ios::trunc};
  ASSERT(ofs, "Could not open " << path);
  ofs << R"(// pybind11
#include <pybind11/pybind11.h>
// hardware_interface
#include <hardware_interface/)"
      << header.name << R"(.hpp>

namespace ros2_control_py::bind_hardware_interface
{

namespace py = pybind11;
)";
  for (const std::string& ns : header.namespaces)
    ofs << "using namespace " << ns << ";\n";
  for (const Cls& cls : header.classes) {
    if (cls.vmembs.empty()) continue;
    ofs << "\nclass Py" << cls.name << ": public " << cls.name
        << "{\n public:\n  using " << cls.name << "::" << cls.name << ";\n";
    for (const VMemb& vmemb : cls.vmembs) ofs << vmemb;
    ofs << "};\n";
  }
  std::string proper = header.name;
  std::replace(proper.begin(), proper.end(), '/', '_');
  ofs << R"(
inline void init_)"
      << proper << R"((py::module &hardware_interface_py)
{
)" << Sep(header.vars, "\n")
      << Sep(header.enums, "\n") << Sep(header.classes, "\n") << R"(}

}
)";
}

void write_hi_py_cpp(const fs::path& hi_py,
                     const std::vector<Header>& headers) {
  std::ofstream ofs{hi_py, std::ios::out | std::ios::trunc};
  ASSERT(ofs, "could not open " << hi_py);
  ofs << R"(// pybind11
#include <pybind11/pybind11.h>
// parse_control_resources_from_urdf
#include <hardware_interface/component_parser.hpp>

// hardware_interface_py
)";
  for (const Header& header : headers)
    ofs << "#include <hardware_interface/" << header.name << "_py.hpp>\n";
  ofs << R"(
namespace py = pybind11;

PYBIND11_MODULE(hardware_interface_py, m)
{
  m.doc() = R"doc(
            Python bindings for ros2_control functionalities.
            )doc";

  // Provide custom function signatures
  py::options options;
  options.disable_function_signatures();

  // Construct module classes
)";
  for (const Header& header : headers) {
    std::string proper = header.name;
    std::replace(proper.begin(), proper.end(), '/', '_');
    ofs << "  ros2_control_py::bind_hardware_interface::init_" << proper
        << "(m);\n";
  }
  ofs << R"(
  m.def("parse_control_resources_from_urdf", hardware_interface::parse_control_resources_from_urdf);
}
)";
}
