// STL
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>

// CppParser
#include <cppast.h>
#include <cppcompound-info-accessor.h>
#include <cppconst.h>
#include <cppobj-info-accessor.h>
#include <cppparser.h>
#include <cppwriter.h>

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

template <typename T, typename U>
class Sep;

template <typename T, typename U>
std::ostream& operator<<(std::ostream& os, const Sep<T, U>& sep);

template <typename T, typename U>
class Sep {
 public:
  Sep(const T& iterable, const U& separator)
      : iterable_{iterable}, separator_{separator} {}

  friend std::ostream& operator<< <>(std::ostream&, const Sep<T, U>&);

 private:
  const T& iterable_;
  const U& separator_;
};

template <typename T, typename U>
std::ostream& operator<<(std::ostream& os, const Sep<T, U>& sep) {
  auto it = std::begin(sep.iterable_);
  if (it == std::end(sep.iterable_)) return os;
  os << *it;
  for (++it; it != std::end(sep.iterable_); ++it) os << sep.separator_ << *it;
  return os;
}

struct Attr {
  Attr(const std::string& name) : name{name} {}

  std::string name;
};

struct Ctor {
  Ctor(std::vector<std::string>&& args) : args{std::move(args)} {}

  std::vector<std::string> args;
};

struct Memb {
  Memb(const std::string& name) : name{name} {}

  std::string name;
};

struct Cls {
  Cls(const std::string& name, const std::string& mother)
      : name{name}, mother{mother} {}

  std::string name;
  std::string mother;
  std::vector<Attr> attrs;
  std::vector<Memb> membs;
  std::vector<Ctor> ctors;
};

std::ostream& operator<<(std::ostream& os, const Cls& cls) {
  os << "  py::class_<" << cls.name
     << (cls.mother.empty() ? "" : ", " + cls.mother)
     << ">(hardware_interface_py, \"" << cls.name << "\")";
  for (const Ctor& ctor : cls.ctors)
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

void write_named_hi_py_hpp(const fs::path& inc_hi_dir, const std::string& name,
                           const std::vector<Cls>& classes) {
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
)" << Sep(classes, "\n")
      << R"(}

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
  CppWriter writer;

  std::vector<std::string> headers;
  std::unordered_map<std::string, std::vector<Cls>> classes;

  for (auto entry : fs::directory_iterator{hi_dir}) {
    const fs::path& path = entry.path();
    if (!entry.is_regular_file() || path.extension() != ".hpp" ||
        path.filename() == "macros.hpp" ||
        path.filename() == "component_parser.hpp")
      continue;

    std::string filename = path.filename().string();
    headers.emplace_back(filename.substr(0, filename.rfind('.')));
    classes.insert({headers.back(), {}});

    const CppCompoundPtr ast = parse_file(parser, path.string());
    ASSERT(ast, "Could not parse " << path);
    // std::cerr << "Parsed " << path << std::endl;
    for (const CppObjPtr& obj_ns : ast->members()) {
      CppConstCompoundEPtr ns = obj_ns;
      if (!ns || !isNamespace(ns) || ns->name() != "hardware_interface")
        continue;
      for (const CppObjPtr& obj_cls : ns->members()) {
        CppConstCompoundEPtr cls = obj_cls;
        if (!cls || (!isClass(cls) && !isStruct(cls))) continue;
        // std::cerr << "Found class: " << cls->name() << std::endl;
        const CppInheritanceListPtr& parents = cls->inheritanceList();
        ASSERT(!parents || parents->size() <= 1,
               "Too many parents for " << cls->name());
        bool has_mother = parents && !parents->empty() &&
                          parents->front().inhType == CppAccessType::kPublic;
        Cls& cls_rep = classes[headers.back()].emplace_back(
            cls->name(), has_mother ? parents->front().baseName : "");
        for (const CppObjPtr& obj_memb : cls->members()) {
          if (!isPublic(obj_memb)) continue;
          CppConstVarEPtr attr = obj_memb;
          if (attr) {
            cls_rep.attrs.emplace_back(attr->name());
          }
          CppConstructorEPtr ctor = obj_memb;
          if (ctor && !ctor->isCopyConstructor() &&
              !ctor->isMoveConstructor() && !cls->hasPureVirtual()) {
            std::vector<std::string> args;
            const CppParamVector* params = ctor->params();
            bool valid = true;
            if (params) {
              for (const CppObjPtr& param : *params) {
                CppConstVarEPtr var = param;
                ASSERT(var, "that was not a var");
                std::ostringstream oss;
                writer.emitVarType(var->varType(), oss);
                if (oss.str() == "Deleter&&" ||
                    oss.str().find("std::unique_ptr<") != std::string::npos) {
                  valid = false;
                  break;
                }
                args.emplace_back(std::move(oss).str());
              }
            }
            if (valid) cls_rep.ctors.emplace_back(std::move(args));
          }
          CppConstFunctionEPtr memb = obj_memb;
          if (memb && memb->name_.find("operator") == std::string::npos &&
              memb->name_ != "get_full_name" &&
              memb->name_ != "import_component") {
            cls_rep.membs.emplace_back(memb->name_);
          }
        }
      }
    }
  }

  fs::path src_dir = dst_dir / "src";
  fs::path hi_py = src_dir / "hardware_interface_py.cpp";
  fs::path inc_hi_dir = dst_dir / "include" / "hardware_interface";

  fs::create_directories(src_dir);
  fs::create_directories(inc_hi_dir);

  for (const auto& [name, classes] : classes)
    write_named_hi_py_hpp(inc_hi_dir, name, classes);

  write_hi_py_cpp(hi_py, headers);

  return 0;
}
