#pragma once

// STL
#include <ostream>
// boost
#include <boost/filesystem.hpp>
// ros2_control_pŷ_builder
#include "structs.hpp"
#include "utils.hpp"

namespace fs = boost::filesystem;

/// @brief output a `Cls` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Cls& cls);
/// @brief output a `VMemb` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const VMemb& vmemb);
/// @brief output a `Enum` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Enum& enu);
/// @brief output a `Var` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Var& var);
/// @brief output a `Func` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Func& func);
/// @brief write source files for module `mod` and header `header`
inline void write_module_header(const Module& mod, const Header& header);
/// @brief write source files for module `mod`
inline void write_module(const Module& mod);

// Impl

inline std::ostream& operator<<(std::ostream& os, const Cls& cls) {
  if (!cls.vmembs.empty()) {
    Cls py_cls{cls.name, "Py" + cls.name};
    py_cls.attrs = cls.attrs;
    py_cls.ctors = cls.ctors;
    py_cls.membs = cls.membs;
    py_cls.membs.reserve(py_cls.membs.size() + cls.vmembs.size());
    for (const VMemb& vmemb : cls.vmembs) py_cls.membs.emplace_back(vmemb.name);
    return os << py_cls;
  }
  os << "  py::class_<" << cls.name
     << (cls.mother.empty() ? "" : ", " + cls.mother) << ">(m, \"" << cls.name
     << "\")";
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
  os << "  py::enum_<" << enu.name << ">(m, \"" << enu.name << "\")\n";
  for (const std::string& item : enu.items)
    os << "      .value(\"" << item << "\", " << enu.name << "::" << item
       << ")\n";
  return os << "      .export_values();\n";
}

inline std::ostream& operator<<(std::ostream& os, const Var& var) {
  return os << "  m.def(\"" << var.name << "\", []() { return std::string{"
            << var.name << "}; });\n";
}

inline std::ostream& operator<<(std::ostream& os, const Func& func) {
  return os << "  m.def(\"" << func.name << "\", &" << func.name << ");\n";
}

void write_module_header(const Module& mod, const Header& header) {
  fs::path path = mod.src_dir / (header.name + "_py.hpp");
  std::ofstream ofs{path, std::ios::out | std::ios::trunc};
  ASSERT(ofs, "Could not open " << path);
  ofs << R"(// pybind11
#include <pybind11/pybind11.h>
// )" << mod.name
      << R"(
#include <)"
      << mod.name << R"(/)" << header.name << R"(.hpp>

namespace ros2_control_py::bind_)"
      << mod.name << R"(
{

namespace py = pybind11;
)";
  for (const std::string& ns : header.namespaces)
    ofs << "using namespace " << ns << ";\n";
  for (const Cls& cls : header.classes) {
    if (cls.vmembs.empty()) continue;
    ofs << "\nclass Py" << cls.name << ": public " << cls.name
        << " {\n public:\n  using " << cls.name << "::" << cls.name << ";\n";
    for (const VMemb& vmemb : cls.vmembs) ofs << vmemb;
    ofs << "};\n";
  }
  ofs << R"(
inline void init_)"
      << header.proper_name << R"((py::module &m)
{
)" << Sep(header.vars, "\n")
      << Sep(header.funcs, "\n") << Sep(header.enums, "\n")
      << Sep(header.classes, "\n") << R"(}

}
)";
}

void write_module(const Module& mod) {
  for (const Header& header : mod.headers) write_module_header(mod, header);

  std::ofstream ofs{mod.src, std::ios::out | std::ios::trunc};
  ASSERT(ofs, "could not open " << mod.src);
  ofs << R"(// pybind11
#include <pybind11/pybind11.h>

// )" << mod.name
      << '\n';
  for (const Header& header : mod.headers)
    ofs << "#include \"" << mod.name << "/" << header.name << "_py.hpp\"\n";
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
  for (const Header& header : mod.headers) {
    ofs << "  ros2_control_py::bind_" << mod.name << "::init_"
        << header.proper_name << "(m);\n";
  }
  ofs << "}\n";
}
