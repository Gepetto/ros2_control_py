// ros2_control_py_builder
#include "parse.hpp"
#include "utils.hpp"
#include "write.hpp"

int main(int argc, char** argv) {
  ASSERT(argc > 3,
         "Invalid number of command line arguments, expected at least 3 got "
             << argc - 1);

  fs::path dst_dir = argv[1];
  fs::path inc_dir = argv[2];
  fs::path src_dir = dst_dir / "src";
  fs::create_directories(src_dir);

  ASSERT_DIR(src_dir);
  ASSERT_DIR(inc_dir);

  std::vector<Module> modules;
  for (int i = 3; i < argc; ++i)
    modules.emplace_back(inc_dir, src_dir, argv[i]);

  for (const Module& mod : modules) {
    fs::create_directories(mod.src_dir);
    ASSERT_DIR(mod.src_dir);
    ASSERT_DIR(mod.inc_dir);
  }

  for (Module& mod : modules) {
    for (const fs::directory_entry& entry :
         fs::recursive_directory_iterator{mod.inc_dir}) {
      const fs::path& path = entry.path();
      if (!fs::is_regular_file(entry) || path.extension() != ".hpp" ||
          path.filename() == "macros.hpp")
        continue;

      std::string name = path.lexically_relative(mod.inc_dir).string();
      parse_header(mod, path, name);
    }
  }

  for (const Module& mod : modules) write_module(mod);

  return 0;
}
