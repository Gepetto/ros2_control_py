#pragma once

// STL
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
// ros2_control_py_builder
#include "utils.hpp"

struct Var {
  Var(const std::string& name) : name{name} {};

  std::string name;
};

struct Func {
  Func(const std::string& name, const std::string& ret_type,
       std::vector<std::string>&& args, std::vector<std::string>&& args_type,
       std::vector<std::string>&& args_names)
      : name{name},
        ret_type{ret_type},
        args{std::move(args)},
        args_type{std::move(args_type)},
        args_names{std::move(args_names)} {}

  friend inline bool operator==(const Func& lhs, const Func& rhs) noexcept {
    return lhs.name == rhs.name && lhs.args_type == rhs.args_type;
  }
  friend inline bool operator!=(const Func& lhs, const Func& rhs) noexcept {
    return !(lhs == rhs);
  }

  const std::string name;
  const std::string ret_type;
  const std::vector<std::string> args;
  const std::vector<std::string> args_type;
  const std::vector<std::string> args_names;
  bool is_overloaded{false};
};

struct Attr : public Var {
  Attr(const std::string& name, bool is_public)
      : Var{name}, is_public{is_public} {}

  bool is_public;
};

struct Ctor {
  Ctor(std::vector<std::string>&& args) noexcept : args{std::move(args)} {}

  friend inline bool operator==(const Ctor& lhs, const Ctor& rhs) noexcept {
    return lhs.args == rhs.args;
  }
  friend inline bool operator!=(const Ctor& lhs, const Ctor& rhs) noexcept {
    return !(lhs == rhs);
  }

  std::vector<std::string> args;
};

struct Memb : public Func {
  explicit Memb(const std::string& name, const std::string& cls,
                const std::string& ret_type, std::vector<std::string>&& args,
                std::vector<std::string>&& args_type,
                std::vector<std::string>&& args_names, bool is_const,
                bool is_virtual, bool is_pure, bool is_final, bool is_public)
      : Func{name, ret_type, std::move(args), std::move(args_type),
             std::move(args_names)},
        cls{cls},
        is_const{is_const},
        is_virtual{is_virtual},
        is_pure{is_pure},
        is_final{is_final},
        is_public{is_public} {}

  inline Memb clone(const std::string& new_cls) const {
    auto new_args = args;
    auto new_args_type = args_type;
    auto new_args_names = args_names;
    return Memb{name,
                new_cls,
                ret_type,
                std::move(new_args),
                std::move(new_args_type),
                std::move(new_args_names),
                is_const,
                is_virtual,
                is_pure,
                is_final,
                is_public};
  }

  friend inline bool operator==(const Memb& lhs, const Memb& rhs) noexcept {
    return lhs.is_const == rhs.is_const &&
           (static_cast<const Func&>(lhs) == static_cast<const Func&>(rhs));
  }
  friend inline bool operator!=(const Memb& lhs, const Memb& rhs) noexcept {
    return !(lhs == rhs);
  }

  const std::string cls;
  const bool is_const;
  bool is_virtual;
  const bool is_pure;
  const bool is_final;
  const bool is_public;
};

struct Header;

struct Cls {
  Cls(const Header& header, const std::string& ns, const std::string& name,
      const std::string& mother_name,
      std::shared_ptr<const Cls> mother = nullptr)
      : header{header},
        complete_name{ns + "::" + name},
        name{name},
        tramp_name("Py" + name),
        pub_name("Pub" + name),
        mother_name{mother_name},
        mother_just_name{just_name(mother_name)},
        mother{std::move(mother)},
        init{mother_name.empty()} {}

  auto find_overloads(const std::string& name) noexcept {
    std::vector<std::shared_ptr<Memb>> found;
    for (auto it = membs.begin(); it != membs.end();) {
      it = std::find_if(it, membs.end(),
                        [name](std::shared_ptr<const Memb> memb) {
                          return memb->name == name;
                        });
      if (it != membs.end()) found.emplace_back(*it++);
    }
    return found;
  }

  std::shared_ptr<Memb> find_override(const Memb& memb) noexcept {
    auto it = std::find_if(
        membs.begin(), membs.end(),
        [memb](std::shared_ptr<const Memb> other) { return *other == memb; });
    return it != membs.cend() ? *it : nullptr;
  }

  std::shared_ptr<const Memb> find_mutable_overload(
      const Memb& memb) const noexcept {
    if (!memb.is_const) return nullptr;
    auto it = std::find_if(membs.cbegin(), membs.cend(),
                           [memb](std::shared_ptr<const Memb> other) {
                             return !other->is_const &&
                                    other->name == memb.name &&
                                    other->args_type == memb.args_type;
                           });
    return it != membs.cend() ? *it : nullptr;
  }

  auto find_vmembs() const noexcept {
    std::vector<std::shared_ptr<const Memb>> found;
    for (auto it = membs.cbegin(); it != membs.cend();) {
      it = std::find_if(it, membs.cend(), [](std::shared_ptr<const Memb> memb) {
        return memb->is_virtual && !memb->is_final;
      });
      if (it != membs.cend()) found.emplace_back(*it++);
    }
    return found;
  }

  auto find_pmembs() const noexcept {
    std::vector<std::shared_ptr<const Memb>> found;
    for (auto it = membs.cbegin(); it != membs.cend();) {
      it = std::find_if(it, membs.cend(), [](std::shared_ptr<const Memb> memb) {
        return !memb->is_public;
      });
      if (it != membs.cend()) found.emplace_back(*it++);
    }
    return found;
  }

  auto find_pattrs() const noexcept {
    std::vector<std::shared_ptr<const Attr>> found;
    for (auto it = attrs.cbegin(); it != attrs.cend();) {
      it = std::find_if(it, attrs.cend(), [](std::shared_ptr<const Attr> attr) {
        return !attr->is_public;
      });
      if (it != attrs.cend()) found.emplace_back(*it++);
    }
    return found;
  }

  const Header& header;
  const std::string complete_name;
  const std::string name;
  const std::string tramp_name;
  const std::string pub_name;
  const std::string mother_name;
  const std::string mother_just_name;
  std::shared_ptr<const Cls> mother{};
  std::vector<std::shared_ptr<Attr>> attrs{};
  std::vector<std::shared_ptr<Memb>> membs{};
  std::vector<Ctor> ctors{};
  bool init;
  bool using_mother_ctor{false};
  bool has_virtual{false};
  bool has_pure{false};
  bool has_protected{false};
  bool is_outsider{false};
  bool is_shared_from_this{false};
  bool has_no_ctor{false};
};

struct Enum {
  Enum(const std::string& name) : name{name} {};

  const std::string name;
  std::vector<std::string> items{};
};

struct Module;

struct Header {
  Header(const Module& mod, const std::string& name)
      : mod{mod}, name{name}, proper_name{get_proper_name(name)} {}

  std::shared_ptr<Cls> find_cls(const std::string& name) noexcept {
    std::string jname = just_name(name);
    auto it = std::find_if(
        classes.begin(), classes.end(),
        [&name](std::shared_ptr<const Cls> cls) { return cls->name == name; });
    return it != classes.end() ? *it : nullptr;
  }

  auto find_overloads(const std::string& name) noexcept {
    std::vector<std::shared_ptr<Func>> found;
    for (auto it = funcs.begin(); it != funcs.end();) {
      it = std::find_if(it, funcs.end(),
                        [name](std::shared_ptr<const Func> func) {
                          return func->name == name;
                        });
      if (it != funcs.end()) found.emplace_back(*it++);
    }
    return found;
  }

  static std::string get_proper_name(std::string name) noexcept {
    std::replace(name.begin(), name.end(), '/', '_');
    return name;
  }

  friend inline bool operator==(const Header& lhs, const Header& rhs) noexcept {
    return &lhs == &rhs;
  }
  friend inline bool operator!=(const Header& lhs, const Header& rhs) noexcept {
    return !(lhs == rhs);
  }

  const Module& mod;
  const std::string name;
  const std::string proper_name;
  std::unordered_set<std::string> namespaces{};
  std::vector<std::shared_ptr<Cls>> classes{};
  std::vector<Enum> enums{};
  std::vector<Var> vars{};
  std::vector<std::shared_ptr<Func>> funcs{};
  std::vector<std::string> required{};
  std::unordered_set<
      std::tuple<std::string, std::string, std::string, std::string>>
      stl_bind{};
};

struct Module {
  Module(fs::path inc_dir, fs::path src_dir, const std::string& name,
         bool py_utils = true)
      : inc_dir{inc_dir / name},
        src_dir{src_dir / name},
        src{src_dir / (name + "_py.cpp")},
        name{name},
        py_utils{py_utils ? new Header{*this, "py_utils"} : nullptr} {}

  friend inline bool operator==(const Module& lhs, const Module& rhs) noexcept {
    return &lhs == &rhs;
  }
  friend inline bool operator!=(const Module& lhs, const Module& rhs) noexcept {
    return !(lhs == rhs);
  }

  const fs::path inc_dir;
  const fs::path src_dir;
  const fs::path src;
  const std::string name;
  std::vector<std::shared_ptr<Header>> headers{};
  const std::shared_ptr<Header> py_utils;
  mutable std::vector<std::string> required{};
};

struct StlBinderHeader {
  std::unordered_set<std::string> namespaces{};
  std::unordered_set<
      std::tuple<std::string, std::string, std::string, std::string>>
      stl_bind{};
};
