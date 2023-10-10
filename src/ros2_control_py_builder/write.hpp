#pragma once

// STL
#include <boost/filesystem/operations.hpp>
#include <ostream>
// boost
#include <boost/filesystem.hpp>
// ros2_control_pŷ_builder
#include "structs.hpp"
#include "utils.hpp"

namespace fs = boost::filesystem;

/// @brief output a `Cls` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Cls& cls);
/// @brief output a `Memb` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Memb& memb);
/// @brief output a `Memb` to write an hpp
inline std::ostream& operator<<(std::ostream& os, const Memb& vmemb);
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
  os << "  py::class_<" << cls.name;
  if (cls.mother) os << ", " << cls.mother->name;
  if (cls.has_virtual) os << ", " << cls.tramp_name;
  os << ">(m, \"" << cls.name << "\")";
  std::vector<Ctor> ctors = cls.ctors;
  if (ctors.empty()) ctors.emplace_back(std::vector<std::string>{});
  for (const Ctor& ctor : ctors)
    os << "\n      .def(py::init<" << Sep{ctor.args, ", "} << ">())";
  for (const Memb& memb : ptr_iter(cls.membs)) {
    if (memb.is_overloaded)
      os << "\n      .def(\"" << memb.name << "\", []("
         << (memb.is_public ? cls.name : cls.pub_name) << "& py_self"
         << (memb.args.empty() ? "" : ", ") << Sep{memb.args, ", "}
         << ") { return py_self." << memb.name << '('
         << Sep{memb.args_names, ", "} << "); })";
    else
      os << "\n      .def(\"" << memb.name << "\", &"
         << (memb.is_public ? cls.name : cls.pub_name) << "::" << memb.name
         << ")";
  }
  for (const Attr& attr : ptr_iter(cls.attrs))
    os << "\n      .def_readwrite(\"" << attr.name << "\", &"
       << (attr.is_public ? cls.name : cls.pub_name) << "::" << attr.name
       << ")";
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
  return os << "  m.def(\"" << var.name << "\", []() { return " << var.name
            << "; });\n";
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
  for (const Cls& cls : ptr_iter(header.classes)) {
    if (!cls.has_virtual) continue;
    ofs << "\nclass " << cls.tramp_name << ": public " << cls.name
        << " {\n public:\n  using " << cls.name << "::" << cls.name << ";\n";
    for (const Memb& vmemb : ptr_iter(cls.find_vmembs())) ofs << vmemb;
    ofs << "};\n";
  }
  for (const Cls& cls : ptr_iter(header.classes)) {
    if (!cls.has_protected) continue;
    ofs << "\nclass " << cls.pub_name << ": public " << cls.name
        << " {\n public:\n";
    for (const Memb& pmemb : ptr_iter(cls.find_pmembs()))
      ofs << "  using " << cls.name << "::" << pmemb.name << ";\n";
    for (const Attr& pattr : ptr_iter(cls.find_pattrs()))
      ofs << "  using " << cls.name << "::" << pattr.name << ";\n";
    ofs << "};\n";
  }
  ofs << R"(
inline void init_)"
      << header.proper_name << R"((py::module &m)
{
)" << Sep{header.vars, "\n"}
      << Sep{ptr_iter(header.funcs), "\n"} << Sep{header.enums, "\n"}
      << Sep{ptr_iter(header.classes), "\n"} << R"(}

}
)";
}

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
    ofs << "#include \"" << mod.name << "/" << header.name << "_py.hpp\"\n";
  ofs << R"(
namespace py = pybind11;

PYBIND11_MODULE()"
      << mod.name + "_py"
      << R"(, m)
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
