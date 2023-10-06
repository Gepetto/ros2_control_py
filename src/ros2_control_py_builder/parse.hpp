#pragma once

// STL
#include <algorithm>
// boost
#include <boost/filesystem.hpp>
// CppParser
#include <cppast.h>
#include <cppcompound-info-accessor.h>
#include <cppconst.h>
#include <cppfunc-info-accessor.h>
#include <cppobj-info-accessor.h>
#include <cppparser.h>
#include <cppwriter.h>
// ros2_control_py_builder
#include "structs.hpp"
#include "utils.hpp"

namespace fs = boost::filesystem;

inline void parse_class_attr(Cls& cls, CppConstVarEPtr attr) {
  cls.attrs.emplace_back(attr->name());
}

inline void parse_class_ctor(Cls& cls, CppConstructorEPtr ctor,
                             CppWriter& writer) {
  std::vector<std::string> args;
  const CppParamVector* params = ctor->params();
  bool valid = true;
  if (params) {
    for (const CppObjPtr& param : *params) {
      CppConstVarEPtr var = param;
      ASSERT(var, "that was not a var");
      std::string arg = str_of_cpp(writer, var->varType());
      if (arg == "Deleter&&" ||
          arg.find("std::unique_ptr<") != std::string::npos) {
        valid = false;
        break;
      }
      args.emplace_back(std::move(arg));
    }
  }
  if (valid) cls.ctors.emplace_back(std::move(args));
}

inline void parse_class_memb(Cls& cls, CppConstFunctionEPtr memb,
                             CppWriter& writer) {
  {
    if (isPureVirtual(memb.get())) {
      std::vector<std::string> args;
      std::vector<std::string> args_names;
      const CppParamVector* params = memb->params();
      if (params) {
        for (const CppObjPtr& param : *params) {
          CppConstVarEPtr var = param;
          ASSERT(var, "that was not a var");
          std::string arg = str_of_cpp(writer, var);
          arg.pop_back();
          arg.pop_back();
          args.emplace_back(std::move(arg));
          args_names.emplace_back(var->name());
        }
      }
      cls.vmembs.emplace_back(memb->name_,
                              str_of_cpp(writer, memb->retType_.get()),
                              cls.name, std::move(args), std::move(args_names));
    } else
      cls.membs.emplace_back(memb->name_);
  }
}

inline void parse_class_using(Cls& cls, Header& header) {
  auto it = std::find_if(
      header.classes.begin(), header.classes.end(),
      [cls](const Cls& other) { return other.name == cls.mother; });
  ASSERT(it != header.classes.end(), "Could not find parent class");
  cls.ctors.insert(cls.ctors.end(), it->ctors.begin(), it->ctors.end());
}

inline void parse_class(Header& header, CppWriter& writer,
                        CppConstCompoundEPtr cls) {
  const CppInheritanceListPtr& parents = cls->inheritanceList();
  ASSERT(!parents || parents->size() <= 1,
         "Too many parents for " << cls->name());
  bool has_mother = parents && !parents->empty() &&
                    parents->front().inhType == CppAccessType::kPublic;
  Cls& cls_rep = header.classes.emplace_back(
      cls->name(), has_mother ? parents->front().baseName : "");
  for (const CppObjPtr& obj_memb : cls->members()) {
    if (!isPublic(obj_memb)) continue;
    CppConstVarEPtr attr = obj_memb;
    if (attr) {
      parse_class_attr(cls_rep, attr);
      continue;
    }
    CppConstructorEPtr ctor = obj_memb;
    if (ctor && !ctor->isCopyConstructor() && !ctor->isMoveConstructor()) {
      parse_class_ctor(cls_rep, ctor, writer);
      continue;
    }
    CppConstFunctionEPtr memb = obj_memb;
    if (memb && memb->name_.find("operator") == std::string::npos &&
        memb->name_ != "get_full_name" && memb->name_ != "import_component") {
      parse_class_memb(cls_rep, memb, writer);
      continue;
    }
    CppConstUsingDeclEPtr use = obj_memb;
    if (use && use->name_ == cls_rep.mother + "::" + cls_rep.mother) {
      parse_class_using(cls_rep, header);
      continue;
    }
  }
}

inline void parse_enum(Header& header, CppEnumEPtr enu) {
  header.enums.emplace_back(enu->name_);
  for (const auto& item : *enu->itemList_)
    header.enums.back().items.emplace_back(item->name_);
}

inline void parse_var(Header& header, CppVarEPtr var) {
  header.vars.emplace_back(var->name());
}

inline void parse_func(Header& header, CppFunctionEPtr func) {
  header.funcs.emplace_back(func->name_);
}

inline void parse_namespace(Header& header, CppWriter& writer,
                            CppConstCompoundEPtr ns, std::string name) {
  header.namespaces.emplace_back(name);
  for (const CppObjPtr& obj : ns->members()) {
    CppVarEPtr var = obj;
    if (var) {
      parse_var(header, var);
      continue;
    }
    CppFunctionEPtr func = obj;
    if (func) {
      parse_func(header, func);
      continue;
    }
    CppEnumEPtr enu = obj;
    if (enu && enu->itemList_) {
      parse_enum(header, enu);
      continue;
    }
    CppConstCompoundEPtr compound = obj;
    if (!compound) continue;
    if (isNamespace(compound))
      parse_namespace(header, writer, compound, name + "::" + compound->name());
    else if (isClass(compound) || isStruct(compound))
      parse_class(header, writer, compound);
  }
}

inline void parse_header(Module& mod, fs::path path, const std::string& name) {
  CppParser parser;
  std::string const upper_name = to_upper(mod.name);
  parser.addIgnorableMacro(upper_name + "_PUBLIC");
  parser.addIgnorableMacro(upper_name + "_LOCAL");
  parser.addIgnorableMacro(upper_name + "_EXPORT");
  parser.addIgnorableMacro(upper_name + "_IMPORT");
  CppWriter writer;

  mod.headers.emplace_back(name.substr(0, name.rfind('.')));

  const CppCompoundPtr ast = parse_file(parser, path.string());
  ASSERT(ast, "Could not parse " << path);
  // std::cerr << "Parsed " << path << std::endl;
  for (const CppObjPtr& obj_ns : ast->members()) {
    CppConstCompoundEPtr ns = obj_ns;
    if (!ns || !isNamespace(ns) || ns->name() != mod.name) continue;
    parse_namespace(mod.headers.back(), writer, ns, ns->name());
  }
}
