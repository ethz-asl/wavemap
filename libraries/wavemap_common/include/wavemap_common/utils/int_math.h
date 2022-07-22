#ifndef WAVEMAP_COMMON_UTILS_INT_MATH_H_
#define WAVEMAP_COMMON_UTILS_INT_MATH_H_

#include <limits>

#include "wavemap_common/common.h"

namespace wavemap::int_math {
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

template <typename T>
constexpr T mult_exp2(T value, int exp) {
  return value * exp2(exp);
}

constexpr int div_exp2(int value, int exp) { return value >> exp; }

template <int dim>
Eigen::Matrix<int, dim, 1> div_exp2(Eigen::Matrix<int, dim, 1> vector,
                                    int exp) {
  DCHECK_GE(exp, 0);
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    vector[dim_idx] >>= exp;
  }
  return vector;
}
}  // namespace wavemap::int_math

#endif  // WAVEMAP_COMMON_UTILS_INT_MATH_H_
