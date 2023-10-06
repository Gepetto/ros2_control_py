#pragma once

// STL
#include <string>
#include <vector>
// boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

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

struct VMemb {
  VMemb(const std::string& name, const std::string& ret_type,
        const std::string cls, const std::vector<std::string>&& args,
        const std::vector<std::string>&& args_names)
      : name{name},
        ret_type{ret_type},
        cls{cls},
        args{std::move(args)},
        args_names{std::move(args_names)} {}

  std::string name;
  std::string ret_type;
  std::string cls;
  std::vector<std::string> const args;
  std::vector<std::string> const args_names;
};

struct Cls {
  Cls(const std::string& name, const std::string& mother)
      : name{name}, mother{mother} {}

  std::string const name;
  std::string const mother;
  std::vector<Attr> attrs;
  std::vector<Memb> membs;
  std::vector<Ctor> ctors;
  std::vector<VMemb> vmembs;
};

struct Enum {
  Enum(const std::string& name) : name{name} {};

  std::string const name;
  std::vector<std::string> items;
};

struct Var {
  Var(const std::string& name) : name{name} {};

  std::string const name;
};

struct Func {
  Func(const std::string& name) : name{name} {};

  std::string const name;
};

struct Header {
  Header(const std::string& name)
      : name{name}, proper_name{get_proper_name(name)} {}

  static std::string get_proper_name(std::string name) {
    std::replace(name.begin(), name.end(), '/', '_');
    return name;
  }

  std::string const name;
  std::string const proper_name;
  std::vector<std::string> namespaces;
  std::vector<Cls> classes;
  std::vector<Enum> enums;
  std::vector<Var> vars;
  std::vector<Func> funcs;
};

struct Module {
  Module(fs::path inc_dir, fs::path src_dir, const std::string& name)
      : inc_dir{inc_dir / name},
        src_dir{src_dir / name},
        src{src_dir / (name + "_py.cpp")},
        name{name} {}

  fs::path const inc_dir;
  fs::path const src_dir;
  fs::path const src;
  std::string name;
  std::vector<Header> headers;
};
