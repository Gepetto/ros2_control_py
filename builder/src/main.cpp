// STL
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// ros2_control_py_builder
#include "parse.hpp"
#include "structs.hpp"
#include "utils.hpp"
#include "write.hpp"

/// @brief init impl header for modules
StlBinderHeader init_impl(std::vector<std::shared_ptr<Module>>& mods);
/// @brief add module for rclcpp
void init_rclcpp_module(std::vector<std::shared_ptr<Module>>& mod,
                        const fs::path& inc_dir, const fs::path& src_dir);
/// @brief find each classes' mother
void init_cls_mothers(std::vector<std::shared_ptr<Module>>& mod);
/// @brief inherit virtual members from parent
void init_cls_vmembs(std::vector<std::shared_ptr<Module>>& mods);
/// @brief find same name functions
void init_overloads(std::vector<std::shared_ptr<Module>>& mod);
/// @brief resolve header dependencies
void init_header_order(std::vector<std::shared_ptr<Module>>& mod);

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

  std::vector<std::shared_ptr<Module>> modules;
  for (int i = 4; i < argc; ++i)
    modules.emplace_back(std::make_shared<Module>(
        long_includes ? inc_dir / argv[i] : inc_dir, src_dir, argv[i]));

  for (const Module& mod : ptr_iter(modules)) {
    fs::create_directories(mod.src_dir);
    ASSERT_DIR(mod.src_dir);
    ASSERT_DIR(mod.inc_dir);
  }

  for (Module& mod : ptr_iter(modules)) {
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

  StlBinderHeader stl_binder = init_impl(modules);
  init_rclcpp_module(modules, inc_dir, src_dir);
  init_cls_mothers(modules);
  init_cls_vmembs(modules);
  init_overloads(modules);
  init_header_order(modules);

  write_impl(src_dir, modules, stl_binder);

  return 0;
}

StlBinderHeader init_impl(std::vector<std::shared_ptr<Module>>& mods) {
  StlBinderHeader stl_binder;
  for (Module& mod : ptr_iter(mods)) {
    for (Header& header : ptr_iter(mod.headers)) {
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
      stl_binder.stl_bind.merge(header.stl_bind);
      header.stl_bind.clear();
      for (const std::string& ns : header.namespaces)
        stl_binder.namespaces.insert(ns);
    }
  }
  return stl_binder;
}

void init_rclcpp_module(std::vector<std::shared_ptr<Module>>& mod,
                        const fs::path& inc_dir, const fs::path& src_dir) {
  const std::string name = "rclcpp";
  Module& rclcpp_mod = *mod.emplace_back(
      std::make_shared<Module>(inc_dir / name, src_dir, name, false));
  Header& rclcpp =
      *rclcpp_mod.headers.emplace_back(new Header(rclcpp_mod, "rclcpp"));
  rclcpp.namespaces.insert("rclcpp");
  rclcpp.namespaces.insert("rclcpp::node_interfaces");
  rclcpp.namespaces.insert("rclcpp::executors");
  rclcpp.namespaces.insert("rclcpp_lifecycle");
  rclcpp.namespaces.insert("rclcpp_lifecycle::node_interfaces");
  Cls& lni = *rclcpp.classes.emplace_back(
      new Cls{rclcpp, "rclcpp_lifecycle::node_interfaces",
              "LifecycleNodeInterface", "", nullptr});
  lni.has_virtual = true;
  const auto names = {"on_configure", "on_cleanup",    "on_shutdown",
                      "on_activate",  "on_deactivate", "on_error"};
  for (const auto& name : names)
    lni.membs.emplace_back(
        new Memb{name,
                 lni.complete_name,
                 "CallbackReturn",
                 {"const rclcpp_lifecycle::State& previous_state"},
                 {"const rclcpp_lifecycle::State&"},
                 {"previous_state"},
                 false,
                 true,
                 false,
                 false,
                 true});
  Cls& state = *rclcpp.classes.emplace_back(
      new Cls{rclcpp, "rclcpp_lifecycle", "State", "", nullptr});
  state.ctors = {{{}}, {{"uint8_t", "const std::string&"}}};
  state.membs = {
      std::shared_ptr<Memb>(new Memb{"id",
                                     state.complete_name,
                                     "uint8_t",
                                     {},
                                     {},
                                     {},
                                     true,
                                     false,
                                     false,
                                     false,
                                     true}),
      std::shared_ptr<Memb>(new Memb{"label",
                                     state.complete_name,
                                     "std::string",
                                     {},
                                     {},
                                     {},
                                     true,
                                     false,
                                     false,
                                     false,
                                     true}),
  };
  Cls& node_base_interface = *rclcpp.classes.emplace_back(new Cls{
      rclcpp, "rclcpp::node_interfaces", "NodeBaseInterface", "", nullptr});
  node_base_interface.has_virtual = true;
  node_base_interface.has_no_ctor = true;
  node_base_interface.membs = {
      std::shared_ptr<Memb>(new Memb{"get_name",
                                     node_base_interface.complete_name,
                                     "const char*",
                                     {},
                                     {},
                                     {},
                                     true,
                                     true,
                                     true,
                                     false,
                                     true}),
      std::shared_ptr<Memb>(new Memb{"get_namespace",
                                     node_base_interface.complete_name,
                                     "const char*",
                                     {},
                                     {},
                                     {},
                                     true,
                                     true,
                                     true,
                                     false,
                                     true}),
  };
  Cls& node_base = *rclcpp.classes.emplace_back(
      new Cls{rclcpp, "rclcpp::node_interfaces", "NodeBase", "", nullptr});
  node_base.has_no_ctor = true;
  node_base.is_shared_from_this = true;
  node_base.membs = {
      std::shared_ptr<Memb>(new Memb{"get_name",
                                     node_base.complete_name,
                                     "const char*",
                                     {},
                                     {},
                                     {},
                                     true,
                                     true,
                                     false,
                                     false,
                                     true}),
      std::shared_ptr<Memb>(new Memb{"get_namespace",
                                     node_base.complete_name,
                                     "const char*",
                                     {},
                                     {},
                                     {},
                                     true,
                                     true,
                                     false,
                                     false,
                                     true}),
  };
  Cls& node = *rclcpp.classes.emplace_back(
      new Cls{rclcpp, "rclcpp", "Node", "", nullptr});
  node.is_shared_from_this = true;
  node.ctors = {
      {{"const std::string&", "const std::string&"}},
      {{"const std::string&"}},
  };
  node.membs = {
      std::shared_ptr<Memb>(new Memb{"get_name",
                                     node.complete_name,
                                     "const char*",
                                     {},
                                     {},
                                     {},
                                     true,
                                     false,
                                     false,
                                     false,
                                     true}),
      std::shared_ptr<Memb>(new Memb{"get_namespace",
                                     node.complete_name,
                                     "const char*",
                                     {},
                                     {},
                                     {},
                                     true,
                                     false,
                                     false,
                                     false,
                                     true}),
      std::shared_ptr<Memb>(new Memb{"now",
                                     node.complete_name,
                                     "rclcpp::Time",
                                     {},
                                     {},
                                     {},
                                     false,
                                     false,
                                     false,
                                     false,
                                     true}),
  };
  Cls& executor = *rclcpp.classes.emplace_back(
      new Cls{rclcpp, "rclcpp", "Executor", "", nullptr});
  executor.has_virtual = true;
  executor.has_no_ctor = true;
  executor.membs = {
      std::shared_ptr<Memb>{new Memb{"spin",
                                     executor.complete_name,
                                     "void",
                                     {},
                                     {},
                                     {},
                                     false,
                                     true,
                                     true,
                                     false,
                                     true}},
      std::shared_ptr<Memb>{new Memb{
          "add_node",
          executor.complete_name,
          "void",
          {"rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr",
           "bool notify"},
          {"rclcpp::node_interfaces::NodeBaseInterface::SharedPtr", "bool"},
          {"node_ptr", "notify"},
          false,
          true,
          false,
          false,
          true}},
      std::shared_ptr<Memb>{new Memb{
          "add_node",
          executor.complete_name,
          "void",
          {"rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr"},
          {"rclcpp::node_interfaces::NodeBaseInterface::SharedPtr"},
          {"node_ptr"},
          false,
          false,
          false,
          false,
          true}},
      std::shared_ptr<Memb>{
          new Memb{"add_node",
                   executor.complete_name,
                   "void",
                   {"std::shared_ptr<rclcpp::Node> node_ptr", "bool notify"},
                   {"std::shared_ptr<rclcpp::Node>", "bool"},
                   {"node_ptr", "notify"},
                   false,
                   true,
                   false,
                   false,
                   true}},
      std::shared_ptr<Memb>{new Memb{"add_node",
                                     executor.complete_name,
                                     "void",
                                     {"std::shared_ptr<rclcpp::Node> node_ptr"},
                                     {"std::shared_ptr<rclcpp::Node>"},
                                     {"node_ptr"},
                                     false,
                                     false,
                                     false,
                                     false,
                                     true}},
  };
  Cls& static_executor = *rclcpp.classes.emplace_back(
      new Cls{rclcpp, "rclcpp::executors", "StaticSingleThreadedExecutor",
              "Executor", nullptr});
  static_executor.ctors = {{{}}};
  static_executor.membs = {
      std::shared_ptr<Memb>{new Memb{"spin_some",
                                     static_executor.complete_name,
                                     "void",
                                     {},
                                     {},
                                     {},
                                     false,
                                     false,
                                     false,
                                     false,
                                     true}},
  };
  Cls& single_executor = *rclcpp.classes.emplace_back(
      new Cls{rclcpp, "rclcpp::executors", "SingleThreadedExecutor", "Executor",
              nullptr});
  single_executor.ctors = {{{}}};
  single_executor.membs = {
      std::shared_ptr<Memb>{new Memb{"spin_some",
                                     single_executor.complete_name,
                                     "void",
                                     {},
                                     {},
                                     {},
                                     false,
                                     false,
                                     false,
                                     false,
                                     true}},
  };
  Cls& multi_executor = *rclcpp.classes.emplace_back(
      new Cls{rclcpp, "rclcpp::executors", "MultiThreadedExecutor", "Executor",
              nullptr});
  multi_executor.ctors = {{{}}};
  multi_executor.membs = {
      std::shared_ptr<Memb>{new Memb{"spin_some",
                                     multi_executor.complete_name,
                                     "void",
                                     {},
                                     {},
                                     {},
                                     false,
                                     false,
                                     false,
                                     false,
                                     true}},
  };
  Enum& cbr =
      rclcpp.enums.emplace_back("LifecycleNodeInterface::CallbackReturn");
  cbr.items = {"SUCCESS", "FAILURE", "ERROR"};
  rclcpp_mod.headers.emplace_back(new Header(rclcpp_mod, "py_ref"));
}

void init_cls_mothers(std::vector<std::shared_ptr<Module>>& mods) {
  std::size_t size = 0;
  for (Module& mod : ptr_iter(mods))
    for (Header& header : ptr_iter(mod.headers)) size += header.classes.size();
  std::unordered_map<std::string, std::shared_ptr<Cls>> classes;
  classes.reserve(size);

  for (Module& mod : ptr_iter(mods))
    for (Header& header : ptr_iter(mod.headers))
      for (std::shared_ptr<Cls> cls : header.classes)
        ASSERT(classes.insert({cls->name, cls}).second,
               "Duplicate class " << cls->name << " in " << header.name
                                  << " and "
                                  << classes[cls->name]->header.name);

  for (Module& mod : ptr_iter(mods)) {
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
        if (header != cls.mother->header) {
          if (header.mod == cls.mother->header.mod)
            header.required.emplace_back(cls.mother->header.name);
          else
            header.mod.required.emplace_back(cls.mother->header.mod.name);
        }
      }
    }
  }
}

void init_cls_vmembs(std::vector<std::shared_ptr<Module>>& mods) {
  bool did_all = false;
  while (!did_all) {
    did_all = true;
    for (Module& mod : ptr_iter(mods)) {
      for (Header& header : ptr_iter(mod.headers)) {
        for (Cls& cls : ptr_iter(header.classes)) {
          if (cls.init) continue;
          if (!cls.mother->init) {
            did_all = false;
            continue;
          }
          cls.init = true;
          cls.is_shared_from_this = cls.mother->is_shared_from_this;
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
            cls.membs.emplace_back(
                std::make_shared<Memb>(vmemb.clone(cls.name)));
            cls.has_virtual = true;
            if (vmemb.is_pure) cls.has_pure = true;
            if (!vmemb.is_public) cls.has_protected = true;
          }
        }
      }
    }
  }
}

void init_overloads(std::vector<std::shared_ptr<Module>>& mods) {
  for (Module& mod : ptr_iter(mods)) {
    for (Header& header : ptr_iter(mod.headers)) {
      for (Func const& func : ptr_iter(header.funcs)) {
        if (func.is_overloaded) continue;
        auto overloads = header.find_overloads(func.name);
        if (overloads.size() <= 1) continue;
        for (Func& overload : ptr_iter(overloads))
          overload.is_overloaded = true;
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
        cls.membs.erase(
            std::remove_if(cls.membs.begin(), cls.membs.end(),
                           [&to_remove](std::shared_ptr<Memb> memb) {
                             return to_remove.find(memb) != to_remove.end();
                           }),
            cls.membs.end());
      }
    }
  }
}

void init_header_order(std::vector<std::shared_ptr<Module>>& mods) {
  // header => header
  for (Module& mod : ptr_iter(mods)) {
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
  // module => module
  std::vector<std::shared_ptr<Module>> modules;
  auto old_modules = ptr_iter(mods);
  while (!mods.empty()) {
    for (auto it = old_modules.begin(); it != old_modules.end();) {
      bool valid = true;
      for (const std::string& req : it->required) {
        if (std::find_if(modules.cbegin(), modules.cend(),
                         [req](std::shared_ptr<const Module> other) {
                           return other->name == req;
                         }) == modules.cend()) {
          valid = false;
          break;
        }
      }
      if (!valid) {
        ++it;
        continue;
      }
      modules.emplace_back(std::move(*it.base()));
      it.from_base(mods.erase(it.base()));
    }
  }
  mods = std::move(modules);
}
