#pragma once

// STL
#include <algorithm>
#include <unordered_map>
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

inline void parse_class_using(
    Cls& cls, std::vector<std::string>& headers,
    std::unordered_map<std::string, std::vector<Cls>>& classes) {
  auto& cls_vec = classes[headers.back()];
  auto it = std::find_if(
      cls_vec.begin(), cls_vec.end(),
      [cls](const Cls& other) { return other.name == cls.mother; });
  ASSERT(it != cls_vec.end(), "Could not find parent class");
  cls.ctors.insert(cls.ctors.end(), it->ctors.begin(), it->ctors.end());
}

inline void parse_class(
    std::vector<std::string>& headers,
    std::unordered_map<std::string, std::vector<Cls>>& classes,
    CppWriter& writer, CppConstCompoundEPtr cls) {
  const CppInheritanceListPtr& parents = cls->inheritanceList();
  ASSERT(!parents || parents->size() <= 1,
         "Too many parents for " << cls->name());
  bool has_mother = parents && !parents->empty() &&
                    parents->front().inhType == CppAccessType::kPublic;
  Cls& cls_rep = classes[headers.back()].emplace_back(
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
      parse_class_using(cls_rep, headers, classes);
      continue;
    }
  }
}

inline void parse_namespace(
    std::vector<std::string>& headers,
    std::unordered_map<std::string, std::vector<Cls>>& classes,
    CppWriter& writer, CppConstCompoundEPtr ns) {
  for (const CppObjPtr& obj_cls : ns->members()) {
    CppConstCompoundEPtr compound = obj_cls;
    if (!compound) continue;
    if (isNamespace(compound)) {
      parse_namespace(headers, classes, writer, compound);
      continue;
    }
    if (isClass(compound) || isStruct(compound)) {
      parse_class(headers, classes, writer, compound);
      continue;
    }
  }
}

inline void parse_header(
    std::vector<std::string>& headers,
    std::unordered_map<std::string, std::vector<Cls>>& classes, fs::path path,
    const std::string& name) {
  CppParser parser;
  parser.addIgnorableMacro("HARDWARE_INTERFACE_PUBLIC");
  CppWriter writer;

  headers.emplace_back(name.substr(0, name.rfind('.')));
  classes.insert({headers.back(), {}});

  const CppCompoundPtr ast = parse_file(parser, path.string());
  ASSERT(ast, "Could not parse " << path);
  // std::cerr << "Parsed " << path << std::endl;
  for (const CppObjPtr& obj_ns : ast->members()) {
    CppConstCompoundEPtr ns = obj_ns;
    if (!ns || !isNamespace(ns) || ns->name() != "hardware_interface") continue;
    parse_namespace(headers, classes, writer, ns);
  }
}
