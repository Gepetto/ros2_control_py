// STL
#include <unordered_set>
// ros2_control_py_builder
#include "parse.hpp"
#include "utils.hpp"
#include "write.hpp"

void init_cls_mothers(Module& mod) {
  std::size_t size = 0;
  for (Header& header : ptr_iter(mod.headers)) size += header.classes.size();
  std::unordered_map<std::string, std::shared_ptr<Cls>> classes;
  classes.reserve(size);

  for (Header& header : ptr_iter(mod.headers))
    for (std::shared_ptr<Cls> cls : header.classes)
      ASSERT(classes.insert({cls->name, cls}).second,
             "Duplicate class " << cls->name << " in " << header.name);

  for (Header& header : ptr_iter(mod.headers)) {
    for (Cls& cls : ptr_iter(header.classes)) {
      if (cls.init) continue;
      auto it = classes.find(cls.mother_just_name);
      if (it == classes.end()) {
        std::cerr << cls.name << " did not find " << cls.mother_just_name
                  << std::endl;
        cls.init = true;
        continue;
      }
      cls.mother = it->second;
      if (header.name != cls.mother->header.name)
        header.required.emplace_back(cls.mother->header.name);
      if (!cls.using_mother_ctor) continue;
      for (const Ctor& ctor : cls.mother->ctors)
        if (std::find(cls.ctors.cbegin(), cls.ctors.cend(), ctor) ==
            cls.ctors.cend())
          cls.ctors.emplace_back(ctor);
    }
  }
}

void init_cls_vmembs(Module& mod) {
  bool did_all = false;
  while (!did_all) {
    did_all = true;
    for (Header& header : ptr_iter(mod.headers)) {
      for (Cls& cls : ptr_iter(header.classes)) {
        if (cls.init) continue;
        if (!cls.mother->init) {
          did_all = false;
          continue;
        }
        cls.init = true;
        if (!cls.mother->has_virtual) continue;
        for (Memb const& vmemb : ptr_iter(cls.mother->find_vmembs())) {
          auto ovrd = cls.find_override(vmemb);
          if (ovrd) {
            if (!ovrd->is_virtual) ovrd->is_virtual = true;
            continue;
          }
          cls.membs.emplace_back(std::make_shared<Memb>(vmemb.clone(cls.name)));
          cls.has_virtual = true;
          if (vmemb.is_pure) cls.has_pure = true;
          if (!vmemb.is_public) cls.has_protected = true;
        }
      }
    }
  }
}

void init_overloads(Module& mod) {
  for (Header& header : ptr_iter(mod.headers)) {
    for (Func const& func : ptr_iter(header.funcs)) {
      if (func.is_overloaded) continue;
      auto overloads = header.find_overloads(func.name);
      if (overloads.size() <= 1) continue;
      for (Func& overload : ptr_iter(overloads)) overload.is_overloaded = true;
    }
    for (Cls& cls : ptr_iter(header.classes)) {
      std::unordered_set<std::shared_ptr<Memb>> to_remove;
      for (Memb const& memb : ptr_iter(cls.membs)) {
        if (memb.is_overloaded) continue;
        auto overloads = cls.find_overloads(memb.name);
        if (overloads.size() <= 1) continue;
        for (std::shared_ptr<Memb> overload : overloads) {
          overload->is_overloaded = true;
          auto covrd = cls.find_mutable_overload(*overload);
          if (covrd) to_remove.insert(overload);
        }
      }
      cls.membs.erase(std::remove_if(cls.membs.begin(), cls.membs.end(),
                                     [&to_remove](std::shared_ptr<Memb> memb) {
                                       return to_remove.find(memb) !=
                                              to_remove.end();
                                     }),
                      cls.membs.end());
    }
  }
}

void init_header_order(Module& mod) {
  std::vector<std::shared_ptr<Header>> headers;
  auto old_headers = ptr_iter(mod.headers);
  while (!mod.headers.empty()) {
    for (auto it = old_headers.begin(); it != old_headers.end();) {
      bool valid = true;
      for (const std::string& req : it->required) {
        if (std::find_if(headers.cbegin(), headers.cend(),
                         [req](std::shared_ptr<const Header> other) {
                           return other->name == req;
                         }) == headers.cend()) {
          valid = false;
          break;
        }
      }
      if (!valid) {
        ++it;
        continue;
      }
      headers.emplace_back(std::move(*it.base()));
      it.from_base(mod.headers.erase(it.base()));
    }
  }
  mod.headers = std::move(headers);
}

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

    init_cls_mothers(mod);
    init_cls_vmembs(mod);
    init_overloads(mod);
    init_header_order(mod);
  }

  for (const Module& mod : modules) write_module(mod);

  return 0;
}
