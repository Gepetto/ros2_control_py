// STL
#include <memory>
#include <unordered_map>
#include <unordered_set>
// ros2_control_py_builder
#include "parse.hpp"
#include "utils.hpp"
#include "write.hpp"

/// @brief init py_utils header for a module
void init_py_utils(Module& mod);
/// @brief find each classes' mother
void init_cls_mothers(Module& mod);
/// @brief inherit virtual members from parent
void init_cls_vmembs(Module& mod);
/// @brief find same name functions
void init_overloads(Module& mod);
/// @brief resolve header dependencies
void init_header_order(Module& mod);

int main(int argc, char** argv) {
  ASSERT(argc > 4,
         "Invalid number of command line arguments, expected at least 4 got "
             << argc - 1);

  const fs::path src_dir = argv[1];
  const fs::path inc_dir = argv[2];
  const std::string ros_distro = argv[3];

  fs::create_directories(src_dir);
  ASSERT_DIR(src_dir);
  ASSERT_DIR(inc_dir);
  ASSERT(ros_distro == "humble" || ros_distro == "rolling",
         "Unsupported ros distro " << ros_distro);

  const bool long_includes = ros_distro != "humble";

  std::vector<Module> modules;
  for (int i = 4; i < argc; ++i)
    modules.emplace_back(long_includes ? inc_dir / argv[i] : inc_dir, src_dir,
                         argv[i]);

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

    init_py_utils(mod);
    init_cls_mothers(mod);
    init_cls_vmembs(mod);
    init_overloads(mod);
    init_header_order(mod);
  }

  for (const Module& mod : modules) write_module(mod);

  return 0;
}

void init_py_utils(Module& mod) {
  for (Header& header : ptr_iter(mod.headers)) {
    header.required.emplace_back(mod.py_utils->name);
    auto set_remove = [](Header& header, const std::string& name) {
      auto it = std::find_if(header.stl_bind.cbegin(), header.stl_bind.cend(),
                             [name](const auto& bind) {
                               return std::get<2>(bind) == name ||
                                      std::get<3>(bind) == name;
                             });
      if (it != header.stl_bind.cend()) header.stl_bind.erase(it);
    };
    set_remove(header, "hardware_interface::CommandInterface");
    set_remove(header, "hardware_interface::StateInterface");
    set_remove(header, "CommandInterface");
    set_remove(header, "StateInterface");
    set_remove(header, "hardware_interface::LoanedCommandInterface");
    set_remove(header, "hardware_interface::LoanedStateInterface");
    set_remove(header, "LoanedCommandInterface");
    set_remove(header, "LoanedStateInterface");
    mod.py_utils->stl_bind.merge(header.stl_bind);
    header.stl_bind.clear();
    for (const std::string& ns : header.namespaces)
      mod.py_utils->namespaces.insert(ns);
  }
  mod.headers.emplace_back(mod.py_utils);

  if (mod.name != "hardware_interface") return;
  auto lni =
      std::make_shared<Cls>(*mod.py_utils, "rclcpp_lifecycle::node_interfaces",
                            "LifecycleNodeInterface", "", nullptr);
  const auto names = {"on_configure", "on_cleanup",    "on_shutdown",
                      "on_activate",  "on_deactivate", "on_error"};
  for (const auto& name : names)
    lni->membs.emplace_back(
        new Memb{name,
                 lni->complete_name,
                 "CallbackReturn",
                 {"const rclcpp_lifecycle::State& previous_state"},
                 {"const rclcpp_lifecycle::State&"},
                 {"previous_state"},
                 false,
                 true,
                 false,
                 false,
                 true});
  lni->has_virtual = true;
  lni->is_outsider = true;
  mod.py_utils->classes.emplace_back(lni);
}

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
        std::cerr << "warning: class " << cls.name << " did not find mother "
                  << cls.mother_just_name << std::endl;
        cls.init = true;
        continue;
      }
      cls.mother = it->second;
      if (header.name != cls.mother->header.name)
        header.required.emplace_back(cls.mother->header.name);
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
        if (cls.using_mother_ctor) {
          for (const Ctor& ctor : cls.mother->ctors)
            if (std::find(cls.ctors.cbegin(), cls.ctors.cend(), ctor) ==
                cls.ctors.cend())
              cls.ctors.emplace_back(ctor);
        }
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
