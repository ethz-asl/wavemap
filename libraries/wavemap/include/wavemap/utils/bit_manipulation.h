#ifndef WAVEMAP_UTILS_BIT_MANIPULATION_H_
#define WAVEMAP_UTILS_BIT_MANIPULATION_H_

#include "wavemap/common.h"

#if defined(__BMI2__) || defined(__AVX2__)
#include <immintrin.h>
#endif

namespace wavemap::bit_manip {
namespace detail {
inline constexpr uint32_t popcount(uint32_t bitstring) {
  return __builtin_popcount(bitstring);
}
inline constexpr uint64_t popcount(uint64_t bitstring) {
  return __builtin_popcountll(bitstring);
}

inline constexpr uint32_t parity(uint32_t bitstring) {
  return __builtin_parity(bitstring);
}
inline constexpr uint64_t parity(uint64_t bitstring) {
  return __builtin_parityll(bitstring);
}

inline constexpr uint32_t clz(uint32_t bitstring) {
  return __builtin_clz(bitstring);
}

inline constexpr uint64_t clz(uint64_t bitstring) {
  return __builtin_clzll(bitstring);
}

#if defined(__BMI2__) || defined(__AVX2__)
inline uint32_t expand(uint32_t source, uint32_t selector) {
  return _pdep_u32(source, selector);
}
inline uint64_t expand(uint64_t source, uint64_t selector) {
  return _pdep_u64(source, selector);
}

inline uint32_t compress(uint32_t source, uint32_t selector) {
  return _pext_u32(source, selector);
}
inline uint64_t compress(uint64_t source, uint64_t selector) {
  return _pext_u64(source, selector);
}
#endif
}  // namespace detail

template <class R, class T>
R bit_cast(T bitstring) {
  static_assert(sizeof(T) == sizeof(R), "Types must be of equal size");
  static_assert(std::is_pod<T>::value, "Input must be of POD type");
  static_assert(std::is_pod<R>::value, "Output must be of POD type");

  R out;
  std::memcpy(std::addressof(out), std::addressof(bitstring), sizeof(T));
  return out;
}

template <typename T>
std::make_unsigned_t<T> bit_cast_unsigned(T bitstring) {
  return bit_cast<std::make_unsigned_t<T>>(bitstring);
}

template <typename T>
std::make_signed_t<T> bit_cast_signed(T bitstring) {
  return bit_cast<std::make_signed_t<T>>(bitstring);
}

template <typename T, T width = 8 * sizeof(T)>
constexpr T rotate_left(T bitstring, T shift) {
  static_assert(width <= 8 * sizeof(T));
  constexpr auto mask = ~std::make_unsigned_t<T>{} >> (8 * sizeof(T) - width);

  auto result = bit_cast_unsigned(bitstring) & mask;
  result = (result << shift) | (result >> (width - shift));
  return result & mask;
}

template <typename T, T width = 8 * sizeof(T)>
constexpr T rotate_right(T bitstring, T shift) {
  static_assert(width <= 8 * sizeof(T));
  constexpr auto mask = ~std::make_unsigned_t<T>{} >> (8 * sizeof(T) - width);

  auto result = bit_cast_unsigned(bitstring) & mask;
  result = (result >> shift) | (result << (width - shift));
  return result & mask;
}

template <typename T>
constexpr T squeeze_in(T bitstring, bool bit, T position) {
  const auto mask = (1 << position) - 1;
  const auto lower = bitstring & mask;
  const std::make_unsigned_t<T> upper = bitstring & ~mask;
  return (upper << 1) | (static_cast<T>(bit) << position) | lower;
}

template <typename T>
constexpr T popcount(T bitstring) {
  return detail::popcount(bit_cast_unsigned(bitstring));
}

template <typename T>
constexpr T parity(T bitstring) {
  return detail::parity(bit_cast_unsigned(bitstring));
}

template <typename T>
constexpr T clz(T bitstring) {
  DCHECK_NE(bitstring, static_cast<T>(0));
  return detail::clz(bit_cast_unsigned(bitstring));
}

template <typename T>
constexpr T repeat_block(int block_width, T block_contents) {
  constexpr int type_width = 8 * sizeof(T);
  const int num_blocks = type_width / block_width +
                         (type_width % block_width != 0);  // Div rounding up
  // Clip off potential stray bits outside the block
  if (block_width < type_width) {
    constexpr auto kOne = static_cast<std::make_unsigned_t<T>>(1);
    block_contents &= (kOne << block_width) - kOne;
  }
  // Stack the block until all bits of type T are covered
  std::make_unsigned_t<T> result{};
  for (int idx = 0; idx < num_blocks; ++idx) {
    result |= block_contents << (idx * block_width);
  }
  return result;
}

template <typename T>
T expand(T source, T selector) {
  return detail::expand(bit_cast_unsigned(source), bit_cast_unsigned(selector));
}

template <typename T>
T compress(T source, T selector) {
  return detail::compress(bit_cast_unsigned(source),
                          bit_cast_unsigned(selector));
}
}  // namespace wavemap::bit_manip

#endif  // WAVEMAP_UTILS_BIT_MANIPULATION_H_
