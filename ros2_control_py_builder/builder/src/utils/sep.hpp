#pragma once

// STL
#include <ostream>

/// @brief wrapper over a `const Iterable&`, a `const Separator&` and a
/// `Projection`, display all elements of a `Iterable` projected by a
/// `Projection` with a 'Separator' as a separator
/// @example std::cout << Sep{std::vector<int>{1, 2, 3}, ", "} << std::endl;
/// // output: `1, 2, 3`
template <typename Iterable, typename Separator, typename Projection>
class Sep;
/// @brief ostream writer for `Sep<T, U>`
template <typename Iterable, typename Separator, typename Projection>
inline std::ostream& operator<<(
    std::ostream& os, const Sep<Iterable, Separator, Projection>& sep);

// hxx
#include "sep.hxx"
