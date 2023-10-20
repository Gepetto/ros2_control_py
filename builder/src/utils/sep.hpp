#pragma once

// STL
#include <ostream>

/// @brief wrapper over a `const T&` and a `const U&`, display all elements of a
/// `T` with a 'U' as a separator
/// @example std::cout << Sep{std::vector<int>{1, 2, 3}, ", "} << std::endl;
/// // output: `1, 2, 3`
template <typename T, typename U>
class Sep;
/// @brief ostream writer for `Sep<T, U>`
template <typename T, typename U>
std::ostream& operator<<(std::ostream& os, const Sep<T, U>& sep);

// hxx
#include "sep.hxx"
