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
    return 1;                              \
  } while (false)

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

int main(int argc, char** argv) {
  namespace fs = std::filesystem;
  ASSERT(argc == 3, "Invalid number of command line arguments, expected 2 got "
                        << argc - 1);

  fs::path src_dir = argv[1];
  fs::path dst = argv[2];
  fs::path dst_dir = fs::path{dst}.parent_path();
  ASSERT(fs::is_directory(src_dir), src_dir << " is not a valid directory");
  ASSERT((fs::is_regular_file(dst) || !fs::exists(dst)) &&
             dst.extension() == ".cpp",
         dst << " is not a valid file");

  CppParser parser;
  parser.addIgnorableMacro("HARDWARE_INTERFACE_PUBLIC");

  std::vector<std::string> names;

  for (auto entry : fs::recursive_directory_iterator{src_dir}) {
    fs::path const& path = entry.path();
    if (!entry.is_regular_file() || path.extension() != ".hpp" ||
        path.filename() == "macros.hpp" ||
        path.filename() == "component_parser.hpp")
      continue;

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

  fs::create_directories(dst_dir);
  std::ofstream ofs{dst, std::ios::out | std::ios::trunc};
  if (!ofs) std::cerr << "error!" << std::endl;
  ofs << std::endl;

  return 0;
}
