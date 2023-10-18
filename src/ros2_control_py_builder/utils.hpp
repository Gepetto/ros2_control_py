#pragma once

// STL
#include <algorithm>
#include <cctype>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
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
inline std::string just_name(const std::string& name);
inline std::string just_name(std::string&& name);

// Impl

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

inline std::string to_upper(std::string str) {
  std::transform(str.cbegin(), str.cend(), str.begin(),
                 [](char c) { return std::toupper(c); });
  return str;
}

inline std::string to_pascal_case(std::string_view str) {
  std::string r;
  r.resize(str.size());
  auto rit = r.begin();
  for (auto it = str.cbegin(); it != str.cend(); ++it) {
    auto end = std::find(it, str.cend(), '_');
    if (it == end) continue;
    *rit++ = std::toupper(*it++);
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
  return std::string{std::move(name)};
}

template <typename T>
class PtrIt {
 public:
  explicit PtrIt(T it) : it_{it} {}

  decltype(std::declval<T>()->operator*()) operator*() const {
    return it_->operator*();
  }
  auto operator->() const { return std::addressof(this->operator*()); }

  PtrIt operator++() { return PtrIt{++it_}; }
  PtrIt operator++(int) { return PtrIt{it_++}; }

  PtrIt operator--() { return PtrIt{--it_}; }
  PtrIt operator--(int) { return PtrIt{it_--}; }

  PtrIt operator+(std::ptrdiff_t i) const { return PtrIt{it_} += i; }
  PtrIt& operator+=(std::ptrdiff_t i) {
    it_ += i;
    return *this;
  }

  PtrIt operator-(std::ptrdiff_t i) const { return PtrIt{it_} -= i; }
  PtrIt& operator-=(std::ptrdiff_t i) {
    it_ -= i;
    return *this;
  }

  bool operator==(const PtrIt& other) const { return it_ == other.it_; }
  bool operator!=(const PtrIt& other) const { return !(*this == other); }

  T base() const { return it_; }
  void from_base(T it) { it_ = it; }

 private:
  T it_;
};

template <typename T>
class CPtrIterable {
 public:
  explicit CPtrIterable(const T& iterable) noexcept : iterable_{iterable} {}

  auto begin() const { return PtrIt{std::begin(iterable_)}; }
  auto end() const { return PtrIt{std::end(iterable_)}; }

  auto cbegin() const { return PtrIt{std::cbegin(iterable_)}; }
  auto cend() const { return PtrIt{std::cend(iterable_)}; }

  auto rbegin() const { return PtrIt{std::rbegin(iterable_)}; }
  auto rend() const { return PtrIt{std::rend(iterable_)}; }

  auto crbegin() const { return PtrIt{std::crbegin(iterable_)}; }
  auto crend() const { return PtrIt{std::crend(iterable_)}; }

 private:
  const T& iterable_;
};

template <typename T>
class PtrIterable {
 public:
  explicit PtrIterable(T& iterable) noexcept : iterable_{iterable} {}

  auto begin() { return PtrIt{std::begin(iterable_)}; }
  auto end() { return PtrIt{std::end(iterable_)}; }

  auto begin() const { return PtrIt{std::begin(iterable_)}; }
  auto end() const { return PtrIt{std::end(iterable_)}; }

  auto cbegin() const { return PtrIt{std::cbegin(iterable_)}; }
  auto cend() const { return PtrIt{std::cend(iterable_)}; }

  auto rbegin() { return PtrIt{std::rbegin(iterable_)}; }
  auto rend() { return PtrIt{std::rend(iterable_)}; }

  auto rbegin() const { return PtrIt{std::rbegin(iterable_)}; }
  auto rend() const { return PtrIt{std::rend(iterable_)}; }

  auto crbegin() const { return PtrIt{std::crbegin(iterable_)}; }
  auto crend() const { return PtrIt{std::crend(iterable_)}; }

 private:
  T& iterable_;
};

template <typename T>
class OwningPtrIterable {
 public:
  explicit OwningPtrIterable(T&& iterable) noexcept
      : iterable_{std::move(iterable)} {}

  auto begin() { return PtrIt{std::begin(iterable_)}; }
  auto end() { return PtrIt{std::end(iterable_)}; }

  auto begin() const { return PtrIt{std::begin(iterable_)}; }
  auto end() const { return PtrIt{std::end(iterable_)}; }

  auto cbegin() const { return PtrIt{std::cbegin(iterable_)}; }
  auto cend() const { return PtrIt{std::cend(iterable_)}; }

  auto rbegin() { return PtrIt{std::rbegin(iterable_)}; }
  auto rend() { return PtrIt{std::rend(iterable_)}; }

  auto rbegin() const { return PtrIt{std::rbegin(iterable_)}; }
  auto rend() const { return PtrIt{std::rend(iterable_)}; }

  auto crbegin() const { return PtrIt{std::crbegin(iterable_)}; }
  auto crend() const { return PtrIt{std::crend(iterable_)}; }

 private:
  T iterable_;
};

template <typename T>
auto ptr_iter(T&& iterable) {
  return OwningPtrIterable{std::move(iterable)};
}

template <typename T>
auto ptr_iter(T& iterable) {
  return PtrIterable{iterable};
}

template <typename T>
auto ptr_iter(const T& iterable) {
  return CPtrIterable{iterable};
}

inline std::uint64_t fn(std::uint64_t x) {
  const std::uint64_t m = (std::uint64_t(0xe9846af) << 32) + 0x9b1a615d;

  x ^= x >> 32;
  x *= m;
  x ^= x >> 32;
  x *= m;
  x ^= x >> 28;

  return x;
}

inline std::uint32_t hash_mix(std::uint32_t x) {
  const std::uint32_t m1 = 0x21f0aaad;
  const std::uint32_t m2 = 0x735a2d97;

  x ^= x >> 16;
  x *= m1;
  x ^= x >> 15;
  x *= m2;
  x ^= x >> 15;

  return x;
}

template <typename T>
void combine_hash(std::size_t& seed, const T& data) {
  seed = hash_mix(seed + 0x9e3779b9 + std::hash<T>{}(data));
}

template <typename U, typename V>
struct std::hash<std::pair<U, V>> {
  constexpr std::uint64_t operator()(const std::pair<U, V>& pair) const {
    std::size_t h = std::hash<U>{}(pair.first);
    combine_hash(h, pair.second);
    return h;
  }
};

template <typename... Args>
struct std::hash<std::tuple<Args...>> {
  constexpr std::uint64_t operator()(const std::tuple<Args...>& tuple) const {
    return impl(tuple, std::index_sequence_for<Args...>{});
  }

 private:
  template <std::size_t... Idx>
  constexpr std::uint64_t impl(const std::tuple<Args...>& tuple,
                               std::index_sequence<Idx...>) const {
    std::size_t h = 0;
    (combine_hash(h, std::get<Idx>(tuple)), ...);
    return h;
  }
};
