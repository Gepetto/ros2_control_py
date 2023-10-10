#pragma once

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
  if (attr->varType()->typeModifier().refType_ != CppRefType::kNoRef) {
    std::cerr << "skipped ref attr " << cls.name << "::" << attr->name()
              << std::endl;
    return;
  }
  if (!isPublic(attr)) return;
  cls.attrs.emplace_back(std::make_shared<Attr>(attr->name(), isPublic(attr)));
  if (!cls.attrs.back()->is_public) cls.has_protected = true;
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
  if (memb->templateParamList()) {
    std::cerr << cls.name << ": skipped template member " << memb->name_
              << std::endl;
    return;
  }
  std::string ret_type = str_of_cpp(writer, memb->retType_.get());
  std::vector<std::string> args;
  std::vector<std::string> args_type;
  std::vector<std::string> args_names;
  const CppParamVector* params = memb->params();
  if (params) {
    size_t i = 0;
    for (const CppObjPtr& param : *params) {
      CppConstVarEPtr var = param;
      ASSERT(var, "that was not a var");
      /*if (var->varType()->typeModifier().refType_ == CppRefType::kByRef)
        std::cerr << "warning: arg by ref in " << cls.name
                  << "::" << memb->name_ << std::endl;*/
      if (var->varType()->typeModifier().refType_ == CppRefType::kRValRef) {
        std::cerr << "warning: skipped " << cls.name << "::" << memb->name_
                  << " because of rvalue argument" << std::endl;
        return;
      }
      std::string type = str_of_cpp(writer, var->varType());
      std::string name =
          !var->name().empty() ? var->name() : "arg" + std::to_string(i++);
      args.emplace_back(type + " " + name);
      args_type.emplace_back(std::move(type));
      args_names.emplace_back(std::move(name));
    }
  }
  cls.membs.emplace_back(std::make_shared<Memb>(
      memb->name_, cls.name, ret_type, std::move(args), std::move(args_type),
      std::move(args_names), isConst(memb.get()), isVirtual(memb.get()),
      isPureVirtual(memb.get()), isFinal(memb.get()), isPublic(memb.get())));
  if (cls.membs.back()->is_virtual && !cls.membs.back()->is_final)
    cls.has_virtual = true;
  if (cls.membs.back()->is_pure) cls.has_pure = true;
  if (!cls.membs.back()->is_public) cls.has_protected = true;
}

inline void parse_class_using(Cls& cls) { cls.using_mother_ctor = true; }

inline void parse_class(Header& header, CppWriter& writer,
                        CppConstCompoundEPtr cls) {
  const CppInheritanceListPtr& parents = cls->inheritanceList();
  ASSERT(!parents || parents->size() <= 1,
         "Too many parents for " << cls->name());
  bool has_mother = parents && !parents->empty() &&
                    parents->front().inhType == CppAccessType::kPublic;
  auto cls_rep = std::make_shared<Cls>(
      header, cls->name(), has_mother ? parents->front().baseName : "");
  header.classes.emplace_back(cls_rep);
  for (const CppObjPtr& obj_memb : cls->members()) {
    if (!isPublic(obj_memb) && !isProtected(obj_memb)) continue;
    CppConstVarEPtr attr = obj_memb;
    if (attr) {
      parse_class_attr(*cls_rep, attr);
      continue;
    }
    CppConstFunctionEPtr memb = obj_memb;
    if (memb && memb->name_.find("operator") == std::string::npos &&
        memb->name_ != "get_full_name" && memb->name_ != "import_component") {
      parse_class_memb(*cls_rep, memb, writer);
      continue;
    }
    if (!isPublic(obj_memb)) continue;
    CppConstructorEPtr ctor = obj_memb;
    if (ctor && !ctor->isCopyConstructor() && !ctor->isMoveConstructor()) {
      parse_class_ctor(*cls_rep, ctor, writer);
      continue;
    }
    CppConstUsingDeclEPtr use = obj_memb;
    if (use && use->name_ == cls_rep->mother_just_name +
                                 "::" + cls_rep->mother_just_name) {
      parse_class_using(*cls_rep);
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
  if (var->templateParamList()) {
    std::cerr << header.name << ": skipped template var " << var->name()
              << std::endl;
    return;
  }
  header.vars.emplace_back(var->name());
}

inline void parse_func(Header& header, CppWriter& writer,
                       CppFunctionEPtr func) {
  if (func->templateParamList()) {
    std::cerr << header.name << ": skipped template function " << func->name_
              << std::endl;
    return;
  }
  std::string ret_type = str_of_cpp(writer, func->retType_.get());
  std::vector<std::string> args;
  std::vector<std::string> args_type;
  std::vector<std::string> args_names;
  const CppParamVector* params = func->params();
  if (params) {
    size_t i = 0;
    for (const CppObjPtr& param : *params) {
      CppConstVarEPtr var = param;
      ASSERT(var, "that was not a var");
      std::string type = str_of_cpp(writer, var->varType());
      std::string name =
          !var->name().empty() ? var->name() : "arg" + std::to_string(i++);
      args.emplace_back(type + " " + name);
      args_type.emplace_back(std::move(type));
      args_names.emplace_back(std::move(name));
    }
  }
  header.funcs.emplace_back(
      std::make_shared<Func>(func->name_, ret_type, std::move(args),
                             std::move(args_type), std::move(args_names)));
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
      parse_func(header, writer, func);
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

  mod.headers.emplace_back(
      std::make_shared<Header>(name.substr(0, name.rfind('.'))));

  const CppCompoundPtr ast = parse_file(parser, path.string());
  ASSERT(ast, "Could not parse " << path);
  // std::cerr << "Parsed " << path << std::endl;
  for (const CppObjPtr& obj_ns : ast->members()) {
    CppConstCompoundEPtr ns = obj_ns;
    if (!ns || !isNamespace(ns) || ns->name() != mod.name) continue;
    parse_namespace(*mod.headers.back(), writer, ns, ns->name());
  }
}
