#pragma once

/// @brief wrapper over an iterator `T` to pointer-like that acts as an iterator
/// to the pointed objects
template <typename T>
class PtrIt;
/// @brief range of `const T&` container with PtrIt as iterators
template <typename T>
class CPtrIterable;
/// @brief range of `T&` container with PtrIt as iterators
template <typename T>
class PtrIterable;
/// @brief range of `T` container with PtrIt as iterators
template <typename T>
class OwningPtrIterable;

/// @brief creates the right kind of PtrIterable
template <typename T>
OwningPtrIterable<T> ptr_iter(T&& iterable);
/// @brief creates the right kind of PtrIterable
template <typename T>
PtrIterable<T> ptr_iter(T& iterable);
/// @brief creates the right kind of PtrIterable
template <typename T>
CPtrIterable<T> ptr_iter(const T& iterable);

// hxx
#include "ptr_iter.hxx"
