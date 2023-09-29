// STL
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
// CppParser
#include <cppast.h>
#include <cppcompound-info-accessor.h>
#include <cppparser.h>

#define ASSERT(Assert, ...)                \
  do {                                     \
    if (Assert) break;                     \
    std::cerr << __VA_ARGS__ << std::endl; \
    std::exit(1);                          \
  } while (false)

namespace fs = std::filesystem;

void remove_attributes(std::string& contents) {
  auto it = contents.begin();
  while (it != contents.end()) {
    it = std::search_n(it, contents.end(), 2, '[');
    auto end = std::search_n(it, contents.end(), 2, ']');
    if (end != contents.end()) it = contents.erase(it, end + 2);
  }
}

std::string read_file(const std::string& filename) {
  std::string contents;
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in) {
    in.seekg(0, std::ios::end);
    size_t size = static_cast<size_t>(in.tellg());
    contents.resize(size + 3);  // For adding last 2 nulls and a new line.
    in.seekg(0, std::ios::beg);
    in.read(contents.data(), size);
    in.close();
    auto len = stripChar(contents.data(), size, '\r');
    assert(len <= size);
    remove_attributes(contents);
    contents.resize(len + 3);
    contents[len] = '\n';
    contents[len + 1] = '\0';
    contents[len + 2] = '\0';
  }
  return contents;
}

CppCompoundPtr parse_file(CppParser& parser, const std::string& filename) {
  std::string stm = read_file(filename);
  CppCompoundPtr cppCompound = parser.parseStream(stm.data(), stm.size());
  if (!cppCompound) return cppCompound;
  cppCompound->name(filename);
  return cppCompound;
}

void write_named_hi_py_hpp(const fs::path& inc_hi_dir,
                           const std::string& name) {
  fs::path path = inc_hi_dir / (name + "_py.hpp");
  std::ofstream ofs{path, std::ios::out | std::ios::trunc};
  ASSERT(ofs, "Could not open " << path);
  ofs << R"(// pybind11
#include <pybind11/pybind11.h>
// hardware_interface
#include <hardware_interface/)"
      << name << R"(.hpp>

namespace ros2_control_py::bind_hardware_interface
{

using namespace hardware_interface;
namespace py = pybind11;

inline void init_)"
      << name << R"((py::module &hardware_interface_py)
{
)";
  // TODO
  ofs << R"(}

}
)";
}

void write_hi_py_cpp(const fs::path& hi_py,
                     const std::vector<std::string>& names) {
  std::ofstream ofs{hi_py, std::ios::out | std::ios::trunc};
  ASSERT(ofs, "could not open " << hi_py);
  ofs << R"(// pybind11
#include <pybind11/pybind11.h>
// parse_control_resources_from_urdf
#include <hardware_interface/component_parser.hpp>

// hardware_interface_py
)";
  for (const std::string& name : names)
    ofs << "#include <hardware_interface/" << name << "_py.hpp>\n";
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
  for (const std::string& name : names)
    ofs << "  ros2_control_py::bind_hardware_interface::init_" << name
        << "(m);\n";
  ofs << R"(
  m.def("parse_control_resources_from_urdf", hardware_interface::parse_control_resources_from_urdf);
}
)";
}

int main(int argc, char** argv) {
  ASSERT(argc == 3, "Invalid number of command line arguments, expected 2 got "
                        << argc - 1);

  fs::path hi_dir = argv[1];
  fs::path dst_dir = argv[2];
  ASSERT(fs::is_directory(hi_dir), hi_dir << " is not a valid directory");
  ASSERT(fs::is_directory(dst_dir), dst_dir << " is not a valid file");

  CppParser parser;
  parser.addIgnorableMacro("HARDWARE_INTERFACE_PUBLIC");

  std::vector<std::string> names;

  for (auto entry : fs::directory_iterator{hi_dir}) {
    const fs::path& path = entry.path();
    if (!entry.is_regular_file() || path.extension() != ".hpp" ||
        path.filename() == "macros.hpp" ||
        path.filename() == "component_parser.hpp")
      continue;

    std::string filename = path.filename().string();
    names.emplace_back(filename.substr(0, filename.rfind('.')));

    const CppCompoundPtr ast = parse_file(parser, path.string());
    ASSERT(ast, "Could not parse " << path);
    std::cerr << "Parsed " << path << std::endl;
    for (const CppObjPtr& obj_ns : ast->members()) {
      CppConstCompoundEPtr ns = obj_ns;
      if (!ns || !isNamespace(ns) || ns->name() != "hardware_interface")
        continue;
      for (const CppObjPtr& obj_cls : ns->members()) {
        CppConstCompoundEPtr cls = obj_cls;
        if (!cls || (!isClass(cls) && !isStruct(cls))) continue;
        std::cerr << "Found class: " << cls->name() << std::endl;
      }
    }
  }

  fs::path src_dir = dst_dir / "src";
  fs::path hi_py = src_dir / "hardware_interface_py.cpp";
  fs::path inc_hi_dir = dst_dir / "include" / "hardware_interface";

  fs::create_directories(src_dir);
  fs::create_directories(inc_hi_dir);

  for (const std::string& name : names) write_named_hi_py_hpp(inc_hi_dir, name);

  write_hi_py_cpp(hi_py, names);

  return 0;
}
