#pragma once

// hpp
#include "parse.hpp"

// CppParser
#include <cppast.h>
#include <cppcompound-info-accessor.h>
#include <cppconst.h>
#include <cppfunc-info-accessor.h>
#include <cppobj-info-accessor.h>
#include <cppparser.h>
#include <cppwriter.h>
// ros2_control_builder
#include "utils.hpp"

inline void find_stl(Header& header, const std::string& type_name,
                     const std::string& type, const std::string& cpp_type);
inline void find_stls(Header& header, const std::string& type_name);
inline void parse_class_attr(Cls& cls, CppConstVarEPtr attr);
inline void parse_class_ctor(Cls& cls, CppConstructorEPtr ctor);
inline void parse_class_memb(Cls& cls, CppConstFunctionEPtr memb);
inline void parse_class_using(Cls& cls);
inline void parse_class(Header& header, CppConstCompoundEPtr cls,
                        const std::string& ns);
inline void parse_enum(Header& header, CppEnumEPtr enu);
inline void parse_var(Header& header, CppVarEPtr var);
inline void parse_func(Header& header, CppFunctionEPtr func);
inline void parse_namespace(Header& header, CppConstCompoundEPtr ns,
                            std::string name);

inline void parse_header(Module& mod, fs::path path, const std::string& name) {
  CppParser parser;
  std::string const upper_name = to_upper(mod.name);
  parser.addIgnorableMacro(upper_name + "_PUBLIC");
  parser.addIgnorableMacro(upper_name + "_LOCAL");
  parser.addIgnorableMacro(upper_name + "_EXPORT");
  parser.addIgnorableMacro(upper_name + "_IMPORT");

  mod.headers.emplace_back(
      std::make_shared<Header>(name.substr(0, name.rfind('.'))));

  const CppCompoundPtr ast = parse_file(parser, path.string());
  ASSERT(ast, "Could not parse " << path);
  // std::cerr << "Parsed " << path << std::endl;
  for (const CppObjPtr& obj_ns : ast->members()) {
    CppConstCompoundEPtr ns = obj_ns;
    if (!ns || !isNamespace(ns) || ns->name() != mod.name) continue;
    parse_namespace(*mod.headers.back(), ns, ns->name());
  }
}

inline void find_stl(Header& header, const std::string& type_name,
                     const std::string& type, const std::string& cpp_type) {
  std::string beg_str = "std::" + cpp_type + "<";
  std::size_t beg = type_name.find(beg_str);
  if (beg == std::string::npos) return;
  beg += beg_str.size();
  std::string u = type_name.substr(beg, type_name.rfind('>') - beg);
  std::string v = "";
  if (type == "map") {
    std::size_t i = u.find(',');
    v = u.substr(i + 1);
    if (!v.empty() && v[0] == ' ') v.erase(v.begin());
    u.erase(u.begin() + i, u.end());
  }
  header.stl_bind.insert({type, cpp_type, u, v});
}

inline void find_stls(Header& header, const std::string& type_name) {
  find_stl(header, type_name, "vector", "vector");
  find_stl(header, type_name, "map", "unordered_map");
  find_stl(header, type_name, "map", "unordered_multimap");
  find_stl(header, type_name, "map", "map");
  find_stl(header, type_name, "map", "multimap");
}

inline void parse_class_attr(Cls& cls, CppConstVarEPtr attr) {
  if (attr->templateParamList()) {
    std::cerr << "warning: class " << cls.name << " skipped template attr "
              << attr->name() << std::endl;
    return;
  }
  if (attr->varType()->typeModifier().refType_ != CppRefType::kNoRef) {
    std::cerr << "warning: class " << cls.name << " skipped ref attr "
              << attr->name() << std::endl;
    return;
  }
  std::string const type_name = str_of_cpp(attr->varType());
  if (type_name.find("std::vector<") != std::string::npos &&
      (type_name.find("CommandInterface") != std::string::npos ||
       type_name.find("StateInterface") != std::string::npos)) {
    std::cerr << "warning: class " << cls.name << " skipped vector attr "
              << attr->name() << std::endl;
    return;
  }
  find_stls(cls.header, type_name);
  cls.attrs.emplace_back(std::make_shared<Attr>(attr->name(), isPublic(attr)));
  if (!cls.attrs.back()->is_public) cls.has_protected = true;
}

inline void parse_class_ctor(Cls& cls, CppConstructorEPtr ctor) {
  if (ctor->templateParamList()) {
    std::cerr << "warning: class " << cls.name << " skipped template ctor"
              << std::endl;
    return;
  }
  std::vector<std::string> args;
  const CppParamVector* params = ctor->params();
  bool valid = true;
  std::size_t defaulted = 0;
  if (params) {
    for (const CppObjPtr& param : *params) {
      CppConstVarEPtr var = param;
      ASSERT(var, "that was not a var");
      if (var->assignValue()) ++defaulted;
      std::string const arg = str_of_cpp(var->varType());
      find_stls(cls.header, arg);
      if (arg == "Deleter&&") {
        valid = false;
        break;
      }
      args.emplace_back(std::move(arg));
    }
  }
  if (!valid) return;
  for (; defaulted > 0; --defaulted) {
    std::vector<std::string> args_cpy{args.cbegin(), args.cend() - defaulted};
    cls.ctors.emplace_back(std::move(args_cpy));
  }
  cls.ctors.emplace_back(std::move(args));
}

inline void parse_class_memb(Cls& cls, CppConstFunctionEPtr memb) {
  if (memb->templateParamList()) {
    std::cerr << "warning: class " << cls.name << " skipped template member "
              << memb->name_ << std::endl;
    return;
  }
  std::string const ret_type = str_of_cpp(memb->retType_.get());
  find_stls(cls.header, ret_type);
  std::vector<std::string> args;
  std::vector<std::string> args_type;
  std::vector<std::string> args_names;
  std::size_t defaulted = 0;
  const CppParamVector* params = memb->params();
  if (params) {
    std::size_t i = 0;
    for (const CppObjPtr& param : *params) {
      CppConstVarEPtr var = param;
      ASSERT(var, "that was not a var");
      if (var->assignValue()) ++defaulted;
      /*if (var->varType()->typeModifier().refType_ == CppRefType::kByRef)
        std::cerr << "warning: arg by ref in " << cls.name
                  << "::" << memb->name_ << std::endl;*/
      if (var->varType()->typeModifier().refType_ == CppRefType::kRValRef) {
        std::cerr << "warning: class " << cls.name
                  << " skipped memb with rvalue arg " << memb->name_
                  << std::endl;
        return;
      }
      std::string type = str_of_cpp(var->varType());
      find_stls(cls.header, type);
      std::string name =
          !var->name().empty() ? var->name() : "arg" + std::to_string(i++);
      args.emplace_back(type + " " + name);
      args_type.emplace_back(std::move(type));
      args_names.emplace_back(std::move(name));
    }
  }
  for (; defaulted > 0; --defaulted) {
    std::vector<std::string> args_cpy{args.cbegin(), args.cend() - defaulted};
    std::vector<std::string> args_type_cpy{args_type.cbegin(),
                                           args_type.cend() - defaulted};
    std::vector<std::string> args_names_cpy{args_names.cbegin(),
                                            args_names.cend() - defaulted};
    cls.membs.emplace_back(std::make_shared<Memb>(
        memb->name_, cls.name, ret_type, std::move(args_cpy),
        std::move(args_type_cpy), std::move(args_names_cpy),
        isConst(memb.get()), isVirtual(memb.get()), isPureVirtual(memb.get()),
        isFinal(memb.get()), isPublic(memb.get())));
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

inline void parse_class(Header& header, CppConstCompoundEPtr cls,
                        const std::string& ns) {
  const CppInheritanceListPtr& parents = cls->inheritanceList();
  ASSERT(!parents || parents->size() <= 1,
         "Too many parents for " << cls->name());
  bool has_mother = parents && !parents->empty() &&
                    parents->front().inhType == CppAccessType::kPublic;
  auto cls_rep = std::make_shared<Cls>(
      header, ns, cls->name(), has_mother ? parents->front().baseName : "");
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
      parse_class_memb(*cls_rep, memb);
      continue;
    }
    if (!isPublic(obj_memb)) continue;
    CppConstructorEPtr ctor = obj_memb;
    if (ctor && !ctor->isCopyConstructor() && !ctor->isMoveConstructor()) {
      parse_class_ctor(*cls_rep, ctor);
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
    std::cerr << "warning: header " << header.name << " skipped template var "
              << var->name() << std::endl;
    return;
  }
  std::string const type_name = str_of_cpp(var->varType());
  find_stls(header, type_name);
  header.vars.emplace_back(var->name());
}

inline void parse_func(Header& header, CppFunctionEPtr func) {
  if (func->templateParamList()) {
    std::cerr << "warning: header " << header.name
              << " skipped template function " << func->name_ << std::endl;
    return;
  }
  std::string ret_type = str_of_cpp(func->retType_.get());
  find_stls(header, ret_type);
  std::vector<std::string> args;
  std::vector<std::string> args_type;
  std::vector<std::string> args_names;
  std::size_t defaulted = 0;
  const CppParamVector* params = func->params();
  if (params) {
    size_t i = 0;
    for (const CppObjPtr& param : *params) {
      CppConstVarEPtr var = param;
      ASSERT(var, "that was not a var");
      if (var->assignValue()) ++defaulted;
      std::string type = str_of_cpp(var->varType());
      find_stls(header, type);
      std::string name =
          !var->name().empty() ? var->name() : "arg" + std::to_string(i++);
      args.emplace_back(type + " " + name);
      args_type.emplace_back(std::move(type));
      args_names.emplace_back(std::move(name));
    }
  }
  for (; defaulted > 0; --defaulted) {
    std::vector<std::string> args_cpy{args.cbegin(), args.cend() - defaulted};
    std::vector<std::string> args_type_cpy{args_type.cbegin(),
                                           args_type.cend() - defaulted};
    std::vector<std::string> args_names_cpy{args_names.cbegin(),
                                            args_names.cend() - defaulted};
    header.funcs.emplace_back(std::make_shared<Func>(
        func->name_, ret_type, std::move(args_cpy), std::move(args_type_cpy),
        std::move(args_names_cpy)));
  }
  header.funcs.emplace_back(
      std::make_shared<Func>(func->name_, ret_type, std::move(args),
                             std::move(args_type), std::move(args_names)));
}

inline void parse_namespace(Header& header, CppConstCompoundEPtr ns,
                            std::string name) {
  header.namespaces.insert(name);
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
      parse_namespace(header, compound, name + "::" + compound->name());
    else if (isClass(compound) || isStruct(compound))
      parse_class(header, compound, name);
  }
}
