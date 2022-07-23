#ifndef WAVEMAP_COMMON_UTILS_BIT_MANIPULATION_H_
#define WAVEMAP_COMMON_UTILS_BIT_MANIPULATION_H_

#include "wavemap_common/common.h"

namespace wavemap::bit_manip {
template <typename T, T width = 8 * sizeof(T)>
constexpr T rotate_left(T bitstring, T shift) {
  static_assert(width <= 8 * sizeof(T));
  constexpr auto mask = ~std::make_unsigned_t<T>{} >> (8 * sizeof(T) - width);

  bitstring &= mask;
  bitstring = (bitstring << shift) |
              (std::make_unsigned_t<T>(bitstring) >> (width - shift));
  return bitstring & mask;
}

template <typename T, T width = 8 * sizeof(T)>
constexpr T rotate_right(T bitstring, T shift) {
  static_assert(width <= 8 * sizeof(T));
  constexpr auto mask = ~std::make_unsigned_t<T>{} >> (8 * sizeof(T) - width);

  bitstring &= mask;
  bitstring = (std::make_unsigned_t<T>(bitstring) >> shift) |
              (bitstring << (width - shift));
  return bitstring & mask;
}

template <typename T>
constexpr int popcount(T bitstring) {
  return __builtin_popcount(bitstring);
}

template <typename T>
constexpr int parity(T bitstring) {
  return __builtin_parity(bitstring);
}
}  // namespace wavemap::bit_manip

#endif  // WAVEMAP_COMMON_UTILS_BIT_MANIPULATION_H_
