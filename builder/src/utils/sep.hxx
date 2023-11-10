#pragma once

// hpp
#include "sep.hpp"

// STL
#include <iterator>
#include <sstream>
#include <type_traits>
#include <utility>

struct identity {
  using is_transparent = std::true_type;

  template <class T>
  constexpr T&& operator()(T&& t) const noexcept {
    return std::forward<T>(t);
  }
};

template <typename Iterable, typename Separator, typename Projection = identity>
class Sep {
 public:
  inline Sep(const Iterable& iterable,
             const Separator& separator) noexcept(noexcept(Projection{}))
      : iterable_{iterable}, separator_{separator}, projection_{} {}

  inline Sep(const Iterable& iterable, const Separator& separator,
             Projection projection) noexcept(noexcept(Projection{
      std::move(std::declval<Projection>())}))
      : iterable_{iterable},
        separator_{separator},
        projection_{std::move(projection)} {}

  inline std::string str() const {
    std::ostringstream oss;
    oss << *this;
    return std::move(oss).str();
  }

  friend std::ostream& operator<< <>(
      std::ostream&, const Sep<Iterable, Separator, Projection>&);

 private:
  const Iterable& iterable_;
  const Separator& separator_;
  Projection projection_;
};

template <typename Iterable, typename Separator, typename Projection>
inline std::ostream& operator<<(
    std::ostream& os, const Sep<Iterable, Separator, Projection>& sep) {
  auto it = std::begin(sep.iterable_);
  if (it == std::end(sep.iterable_)) return os;
  os << sep.projection_(*it);
  for (++it; it != std::end(sep.iterable_); ++it)
    os << sep.separator_ << sep.projection_(*it);
  return os;
}
