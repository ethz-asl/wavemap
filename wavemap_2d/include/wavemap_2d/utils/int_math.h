#ifndef WAVEMAP_2D_UTILS_INT_MATH_H_
#define WAVEMAP_2D_UTILS_INT_MATH_H_

#include <limits>

namespace wavemap_2d::int_math {
constexpr int exp2(int exponent) { return 1 << exponent; }

constexpr int log2_floor(int value) {
  return std::numeric_limits<int>::digits - __builtin_clz(value);
}

constexpr int log2_ceil(int value) {
  const int log2_floored = log2_floor(value);
  if (__builtin_popcount(value) == 1) {
    return log2_floored;
  } else {
    return log2_floored + 1;
  }
}
}  // namespace wavemap_2d::int_math

#endif  // WAVEMAP_2D_UTILS_INT_MATH_H_
