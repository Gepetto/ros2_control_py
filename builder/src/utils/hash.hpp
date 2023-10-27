#pragma once

// STL
#include <functional>

/// @brief combines seed with `std::hash<T>{}(data)`
template <typename T>
void combine_hash(std::size_t& seed, const T& data);

/// @brief std::hash for all std::pair<U, V>
template <typename U, typename V>
struct std::hash<std::pair<U, V>>;

/// @brief std::hash for all std::tuple<Args...>
template <typename... Args>
struct std::hash<std::tuple<Args...>>;

// hxx
#include "hash.hxx"
