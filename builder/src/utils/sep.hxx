#pragma once

// hpp
#include "sep.hpp"

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
