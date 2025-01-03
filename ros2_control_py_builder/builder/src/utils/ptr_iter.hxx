#pragma once

// hpp
#include "ptr_iter.hpp"

// STL
#include <iterator>

template <typename T>
class PtrIt {
 public:
  explicit PtrIt(T it) : it_{it} {}

  decltype(std::declval<T>()->operator*()) operator*() const {
    return it_->operator*();
  }
  auto operator->() const { return std::addressof(operator*()); }

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
OwningPtrIterable<T> ptr_iter(T&& iterable) {
  return OwningPtrIterable{std::move(iterable)};
}

template <typename T>
PtrIterable<T> ptr_iter(T& iterable) {
  return PtrIterable{iterable};
}

template <typename T>
CPtrIterable<T> ptr_iter(const T& iterable) {
  return CPtrIterable{iterable};
}
