#pragma once

// hpp
#include "cppast.h"
#include "utils.hpp"

// STL
#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
// CppParser
#include <cppwriter.h>

inline void remove_attributes(std::string& contents) {
  // remove attributes aka [[...]]
  auto it = contents.cbegin();
  while (it != contents.cend()) {
    it = std::search_n(it, contents.cend(), 2, '[');
    auto end = std::search_n(it, contents.cend(), 2, ']');
    if (end != contents.cend()) it = contents.erase(it, end + 2);
  }
  // remove digit separators aka d'ddd'ddd but not char digits aka 'd'
  it = contents.cbegin();
  while (it != contents.cend()) {
    it = std::adjacent_find(it, contents.cend(), [](char a, char b) {
      return std::isdigit(a) && b == '\'';
    });
    if (it == contents.cend()) continue;
    it += 2;
    if (it == contents.cend() || !std::isdigit(*it)) continue;
    it = contents.erase(it - 1);
  }
  // remove raw strings aka R"()"
  auto mit = contents.begin();
  while (mit != contents.end()) {
    mit = std::adjacent_find(mit, contents.end(), [](char a, char b) {
      return a == 'R' && b == '"';
    });
    if (mit == contents.end()) continue;
    mit += 2;
    if (mit == contents.end()) continue;
    auto name_end = std::find(mit, contents.end(), '(');
    std::string name = std::string{mit, name_end};
    auto valid_dchar = [](char c) {
      return (!std::isalnum(c) && !std::ispunct(c)) || (c == ')' || c == '\\');
    };
    if (name.size() > 16 ||
        std::find_if(name.cbegin(), name.cend(), valid_dchar) != name.cend())
      continue;
    std::size_t end =
        contents.find(")" + name + "\"", name_end - contents.cbegin());
    if (end == std::string::npos) continue;
    *(mit - 2) = '"';  // replpace R with "
    mit = contents.erase(
        mit - 1, contents.cbegin() + end + name.size() + 1);  // R|"name()name|"
  }
}

inline std::string read_file(const std::string& filename) {
  std::string contents;
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in) {
    in.seekg(0, std::ios::end);
    size_t size = static_cast<size_t>(in.tellg());
    contents.resize(size + 3);  // For adding last 2 nulls and a new line.
    in.seekg(0, std::ios::beg);
    in.read(contents.data(), size);
    in.close();
    auto len = stripChar(contents.data(), size, '\r');
    assert(len <= size);
    remove_attributes(contents);
    contents.resize(len + 3);
    contents[len] = '\n';
    contents[len + 1] = '\0';
    contents[len + 2] = '\0';
  }
  return contents;
}

inline CppCompoundPtr parse_file(CppParser& parser,
                                 const std::string& filename) {
  std::string stm = read_file(filename);
  CppCompoundPtr cppCompound = parser.parseStream(stm.data(), stm.size());
  if (!cppCompound) return cppCompound;
  cppCompound->name(filename);
  return cppCompound;
}

inline std::string str_of_cpp(const CppObj* cppObj) {
  std::ostringstream oss;
  CppWriter{}.emit(cppObj, oss);
  std::string str = std::move(oss).str();
  if (!str.empty() && str.back() == '*') {
    std::string type = str.substr(0, str.size() - 1);
    if (type == "double") return "Ref<" + type + ">";
  }
  return str;
}

inline std::string str_of_cpp(const CppObjPtr& cppObjPtr) {
  return str_of_cpp(cppObjPtr.get());
}

inline std::string to_upper(std::string str) {
  std::transform(str.cbegin(), str.cend(), str.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  return str;
}

inline std::string to_pascal_case(std::string_view str) {
  std::string r;
  r.resize(str.size());
  auto rit = r.begin();
  for (auto it = str.cbegin(); it != str.cend(); ++it) {
    auto end = std::find(it, str.cend(), '_');
    if (it == end) continue;
    *rit++ = std::toupper(static_cast<unsigned char>(*it++));
    if (it != end) {
      std::copy(it, end, rit);
      rit += end - it;
    }
    it = end;
    if (it == str.cend()) break;
  }
  if (rit != r.cend()) r.erase(rit, r.cend());
  return r;
}

template <typename... Args>
inline std::string make_pascal_name(const Args&... args) {
  std::string r;
  (r.append(to_pascal_case(just_name(args))), ...);
  return r;
}

inline std::string just_name(const std::string& name) {
  return just_name(std::string{name});
}

inline std::string just_name(std::string&& name) {
  auto it = std::search_n(name.crbegin(), name.crend(), 2, ':');
  if (it != name.crend()) name.erase(name.cbegin(), it.base());
  return name;
}
