#pragma once

// STL
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
// CppParser
#include <cppparser.h>
#include <cppwriter.h>

/// @brief if not `Assert` print the other args and exit failure
#define ASSERT(Assert, ...)                \
  do {                                     \
    if (Assert) break;                     \
    std::cerr << __VA_ARGS__ << std::endl; \
    std::exit(EXIT_FAILURE);               \
  } while (false)
/// @brief if not fs::is_directori((Dir)) fails
#define ASSERT_DIR(Dir) \
  ASSERT(fs::is_directory((Dir)), (Dir) << " is not a valid directory")

/// @brief wrapper over a `const T&` and a `const U&`, display all elements of a
/// `T` with a 'U' as a separator
/// @expample std::cout << Sep{std::vector<int>{1, 2, 3}, ", "} << std::endl; //
/// output: `1, 2, 3`
template <typename T, typename U>
class Sep;

/// @brief removes `[[.*]]` sequences from the string because the parser does
/// not handle them well
inline void remove_attributes(std::string& contents);
/// @brief same as readFile from CppParser but calls remove_attributes
inline std::string read_file(const std::string& filename);
/// @brief same as parseFile from CppParser but calls read_file instead of
/// readFile
inline CppCompoundPtr parse_file(CppParser& parser,
                                 const std::string& filename);
/// @brief ostream writer for `Sep<T, U>`
template <typename T, typename U>
std::ostream& operator<<(std::ostream& os, const Sep<T, U>& sep);
/// @brief wrapper to get a string repr of a CppObj
inline std::string str_of_cpp(CppWriter& writer, const CppObj* cppObj);
/// @brief std::string to upper (not in place)
inline std::string to_upper(std::string str);

// Impl

inline void remove_attributes(std::string& contents) {
  // remove attributes aka [[...]]
  auto it = contents.begin();
  while (it != contents.end()) {
    it = std::search_n(it, contents.end(), 2, '[');
    auto end = std::search_n(it, contents.end(), 2, ']');
    if (end != contents.end()) it = contents.erase(it, end + 2);
  }
  // digit separators aka d'ddd'ddd but not char digits aka 'd'
  it = contents.begin();
  while (it != contents.end()) {
    it = std::adjacent_find(it, contents.end(), [](char a, char b) {
      return std::isdigit(a) && b == '\'';
    });
    if (it == contents.end()) continue;
    it += 2;
    if (it == contents.end() || !std::isdigit(*it)) continue;
    it = contents.erase(it - 1);
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

template <typename T, typename U>
class Sep {
 public:
  Sep(const T& iterable, const U& separator)
      : iterable_{iterable}, separator_{separator} {}

  friend std::ostream& operator<< <>(std::ostream&, const Sep<T, U>&);

 private:
  const T& iterable_;
  const U& separator_;
};

template <typename T, typename U>
std::ostream& operator<<(std::ostream& os, const Sep<T, U>& sep) {
  auto it = std::begin(sep.iterable_);
  if (it == std::end(sep.iterable_)) return os;
  os << *it;
  for (++it; it != std::end(sep.iterable_); ++it) os << sep.separator_ << *it;
  return os;
}

inline std::string str_of_cpp(CppWriter& writer, const CppObj* cppObj) {
  std::ostringstream oss;
  writer.emit(cppObj, oss);
  return std::move(oss).str();
}

inline std::string to_upper(std::string str) {
  std::transform(str.cbegin(), str.cend(), str.begin(),
                 [](char c) { return std::toupper(c); });
  return str;
}
