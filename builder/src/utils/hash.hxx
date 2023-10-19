#pragma once

// hpp
#include "hash.hpp"

// STL
#include <cstdint>

inline std::uint64_t hash_mix(std::uint64_t x);
inline std::uint32_t hash_mix(std::uint32_t x);

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

inline std::uint64_t hash_mix(std::uint64_t x) {
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
